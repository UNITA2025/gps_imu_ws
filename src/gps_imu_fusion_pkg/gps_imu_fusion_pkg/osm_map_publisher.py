#!/usr/bin/env python3

#========================================================================================
# 기능: OSM(XML)을 파싱해 도로/장애물을 로컬 평면(미터)로 투영, OccupancyGrid로 변환해 /map으로 주기 발행.
# 동작: use_gps_center면 첫 NavSatFix로 기준 위경도 결정 →  모든 node/way 파싱 →  equirectangular 투영(+옵션 회전)
#       →  도로는 두껍게, 장애물 외곽선은 occupied(100)로 그려 그리드 구성 →  헤더/원점 설정 후 퍼블리시.
# TODO : 
# 최종 수정일: 2025.08.18
# 편집자 : 송준상, 이다빈
#========================================================================================

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import xml.etree.ElementTree as ET
import numpy as np
import math

class OSMMapPublisher(Node):
    def __init__(self):
        super().__init__('osm_map_publisher')
        
        # Parameters
        self.declare_parameter('osm_file', 'map.osm')
        self.declare_parameter('resolution', 0.1)  # meters per pixel
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('map_width', 500.0)  # 지도 폭 (미터)
        self.declare_parameter('map_height', 500.0)  # 지도 높이 (미터)
        self.declare_parameter('rotation_angle', 0.0)  # 지도 회전 각도 (도 단위)
        self.declare_parameter('gps_topic', '/gps/fix')  # GPS 토픽 이름
        self.declare_parameter('use_gps_center', True)  # GPS로 중심 설정할지 여부
        
        # Get parameters
        self.osm_file = self.get_parameter('osm_file').get_parameter_value().string_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.map_width = self.get_parameter('map_width').get_parameter_value().double_value
        self.map_height = self.get_parameter('map_height').get_parameter_value().double_value
        self.rotation_angle = self.get_parameter('rotation_angle').get_parameter_value().double_value
        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.use_gps_center = self.get_parameter('use_gps_center').get_parameter_value().bool_value
        
        # GPS 관련 변수
        self.gps_center_lat = None
        self.gps_center_lon = None
        self.map_generated = False
        
        # Publisher
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # GPS Subscriber (GPS 중심 사용할 때만)
        if self.use_gps_center:
            self.gps_subscriber = self.create_subscription(
                NavSatFix,
                self.gps_topic,
                self.gps_callback,
                10
            )
            self.get_logger().info(f'Waiting for GPS fix from {self.gps_topic}...')
        else:
            # GPS 사용하지 않으면 바로 지도 생성
            self.occupancy_grid = self.load_osm_file()
            self.map_generated = True
        
        # Timer
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_map)
        
        self.get_logger().info(f'OSM Map Publisher started with file: {self.osm_file}')
        self.get_logger().info(f'Resolution: {self.resolution} m/pixel')
        self.get_logger().info(f'Map frame: {self.map_frame}')
        if self.use_gps_center:
            self.get_logger().info(f'GPS center mode enabled - subscribing to {self.gps_topic}')

    def gps_callback(self, msg):
        """GPS 콜백 - 첫 번째 유효한 GPS 위치를 지도 중심으로 설정"""
        if self.gps_center_lat is None and self.gps_center_lon is None:
            # GPS 신호가 유효한지 확인
            if (msg.status.status >= 0 and  # GPS 상태 확인
                not math.isnan(msg.latitude) and not math.isnan(msg.longitude) and
                msg.latitude != 0.0 and msg.longitude != 0.0):
                
                self.gps_center_lat = msg.latitude
                self.gps_center_lon = msg.longitude
                
                self.get_logger().info(f'GPS center set: ({self.gps_center_lat:.6f}, {self.gps_center_lon:.6f})')
                
                # GPS 중심이 설정되면 지도 생성
                self.occupancy_grid = self.load_osm_file()
                self.map_generated = True
                
                # GPS 구독 중지 (첫 번째 값만 필요)
                self.destroy_subscription(self.gps_subscriber)
                self.get_logger().info('GPS subscription stopped - map generation complete')

    def load_osm_file(self):
        """Load OSM file and convert to occupancy grid"""
        try:
            self.get_logger().info(f"Loading OSM file: {self.osm_file}")
            tree = ET.parse(self.osm_file)
            root = tree.getroot()
            
            # Extract nodes (points)
            nodes = {}
            for node in root.findall('node'):
                node_id = node.get('id')
                lat = float(node.get('lat'))
                lon = float(node.get('lon'))
                nodes[node_id] = (lat, lon)
            
            self.get_logger().info(f"Found {len(nodes)} nodes")
            
            # Extract ways (lines/polygons)
            ways = []
            roads = []
            obstacle_count = 0
            road_count = 0
            
            for way in root.findall('way'):
                way_nodes = []
                for nd in way.findall('nd'):
                    ref = nd.get('ref')
                    if ref in nodes:
                        way_nodes.append(nodes[ref])
                
                # Check if it's a road or building
                tags = {tag.get('k'): tag.get('v') for tag in way.findall('tag')}
                self.get_logger().info(f"Way tags: {tags}")
                
                if self.is_obstacle(tags):
                    ways.append(way_nodes)
                    obstacle_count += 1
                    self.get_logger().info(f"Added obstacle way with {len(way_nodes)} nodes")
                elif self.is_road(tags):
                    roads.append((way_nodes, tags))
                    road_count += 1
                    self.get_logger().info(f"Added road way with {len(way_nodes)} nodes: {tags.get('highway', 'unknown')}")
            
            self.get_logger().info(f"Found {obstacle_count} obstacle ways and {road_count} road ways out of {len(root.findall('way'))} total ways")
            
            if not nodes:
                self.get_logger().error("No nodes found in OSM file!")
                return self.create_empty_grid()
            
            # Convert to occupancy grid
            return self.create_occupancy_grid(ways, roads, nodes)
            
        except Exception as e:
            self.get_logger().error(f"Failed to load OSM file: {str(e)}")
            return self.create_empty_grid()

    def is_obstacle(self, tags):
        """Determine if a way represents an obstacle (building, wall, etc.)"""
        obstacle_tags = {
            'building': True,  # 모든 건물
            'wall': True,      # 벽
            'barrier': True,   # 장벽
            'amenity': ['research_institute', 'school', 'hospital', 'parking'],  # 특정 편의시설
            'natural': ['water', 'cliff', 'tree_row'],  # 자연 장애물 (물 포함)
            'waterway': True,  # 물길
            'landuse': ['forest', 'farmland', 'industrial'],
            'highway': False   # 도로는 자유공간
        }
        
        for key, values in obstacle_tags.items():
            if key in tags:
                if isinstance(values, bool):
                    return values
                elif isinstance(values, list):
                    return tags[key] in values
        
        # 기본적으로 태그가 없거나 알 수 없는 경우는 자유공간으로 처리
        return False

    def is_road(self, tags):
        """Determine if a way represents a road"""
        return 'highway' in tags

    def lat_lon_to_meters(self, lat, lon, ref_lat, ref_lon):
        """Convert lat/lon to local meters using simple projection"""
        # Simple equirectangular projection
        lat_rad = math.radians(ref_lat)
        
        x = math.radians(lon - ref_lon) * 6378137.0 * math.cos(lat_rad)
        y = math.radians(lat - ref_lat) * 6378137.0
        
        return x, y

    def rotate_point(self, x, y, angle_degrees):
        """회전 변환 적용 (좌표계 보정 포함)"""
        import math
        angle_rad = math.radians(-angle_degrees)  # 각도 방향 반전
        cos_angle = math.cos(angle_rad)
        sin_angle = math.sin(angle_rad)
        
        x_rotated = x * cos_angle - y * sin_angle
        y_rotated = x * sin_angle + y * cos_angle
        
        return x_rotated, y_rotated

    def create_occupancy_grid(self, ways, roads, nodes):
        """Create occupancy grid from OSM data"""
        if not nodes:
            return self.create_empty_grid()
        
        # Find bounds
        lats = [lat for lat, lon in nodes.values()]
        lons = [lon for lat, lon in nodes.values()]
        
        # 사용자가 중심 좌표를 지정했는지 또는 GPS 중심을 사용하는지 확인
        if self.use_gps_center and self.gps_center_lat is not None and self.gps_center_lon is not None:
            # GPS로 받은 중심 좌표 사용
            ref_lat = self.gps_center_lat
            ref_lon = self.gps_center_lon
            self.get_logger().info(f"Using GPS center: ({ref_lat:.6f}, {ref_lon:.6f})")
        else:
            # 자동 중심 계산
            min_lat, max_lat = min(lats), max(lats)
            min_lon, max_lon = min(lons), max(lons)
            ref_lat = (min_lat + max_lat) / 2
            ref_lon = (min_lon + max_lon) / 2
            self.get_logger().info(f"Using auto-calculated center: ({ref_lat:.6f}, {ref_lon:.6f})")
        
        # Convert all points to meters
        points_meters = {}
        for node_id, (lat, lon) in nodes.items():
            x, y = self.lat_lon_to_meters(lat, lon, ref_lat, ref_lon)
            # 회전 적용
            if self.rotation_angle != 0.0:
                x, y = self.rotate_point(x, y, self.rotation_angle)
            points_meters[node_id] = (x, y)
        
        # 사용자가 지도 크기를 지정했는지 확인
        if self.map_width > 0 and self.map_height > 0:
            # 사용자 지정 크기 사용
            half_width = self.map_width / 2
            half_height = self.map_height / 2
            min_x, max_x = -half_width, half_width
            min_y, max_y = -half_height, half_height
            self.get_logger().info(f"Using user-specified map size: {self.map_width}x{self.map_height}m")
        else:
            # 자동 크기 계산
            xs = [x for x, y in points_meters.values()]
            ys = [y for x, y in points_meters.values()]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            
            # Add padding
            padding = 50.0  # meters
            min_x -= padding
            max_x += padding
            min_y -= padding
            max_y += padding
        
        # Calculate grid size (원본 크기 유지하되 적절히 제한)
        original_width = int((max_x - min_x) / self.resolution)
        original_height = int((max_y - min_y) / self.resolution)
        
        # 지도가 너무 크면 해상도를 자동으로 조정
        max_pixels = 4000000  # 400만 픽셀 제한
        total_pixels = original_width * original_height
        
        if total_pixels > max_pixels:
            scale_factor = (max_pixels / total_pixels) ** 0.5
            self.resolution = self.resolution / scale_factor
            self.get_logger().info(f"Resolution adjusted to {self.resolution:.3f} to fit memory limits")
            width = int((max_x - min_x) / self.resolution)
            height = int((max_y - min_y) / self.resolution)
        else:
            width = original_width
            height = original_height
        
        # Origin을 (0,0)으로 설정하여 지도 중심이 RViz2 원점에 오도록 함
        origin_x = min_x
        origin_y = min_y
        
        self.get_logger().info(f"Grid size: {width}x{height} pixels")
        self.get_logger().info(f"Reference coordinates: ({ref_lat:.6f}, {ref_lon:.6f})")
        self.get_logger().info(f"Map bounds: X[{min_x:.1f}, {max_x:.1f}], Y[{min_y:.1f}, {max_y:.1f}]")
        self.get_logger().info(f"Origin: ({origin_x:.1f}, {origin_y:.1f})")
        
        # Initialize grid (unknown = -1, free = 0, occupied = 100)
        grid = np.full((height, width), -1, dtype=np.int8)
        
        # 먼저 도로를 자유공간으로 그리기
        for road_nodes, road_tags in roads:
            road_points_meters = []
            for lat, lon in road_nodes:
                x, y = self.lat_lon_to_meters(lat, lon, ref_lat, ref_lon)
                # 회전 적용
                if self.rotation_angle != 0.0:
                    x, y = self.rotate_point(x, y, self.rotation_angle)
                road_points_meters.append((x, y))
            
            # 도로 폭 계산
            road_width = self.get_road_width(road_tags)
            
            # 도로를 두껍게 그리기
            for i in range(len(road_points_meters) - 1):
                x1, y1 = road_points_meters[i]
                x2, y2 = road_points_meters[i + 1]
                self.draw_thick_line(grid, x1, y1, x2, y2, origin_x, origin_y, width, height, road_width, 0)  # 0 = free space
        
        # 그 다음 장애물 그리기
        for way in ways:
            way_points_meters = []
            for lat, lon in way:
                x, y = self.lat_lon_to_meters(lat, lon, ref_lat, ref_lon)
                # 회전 적용
                if self.rotation_angle != 0.0:
                    x, y = self.rotate_point(x, y, self.rotation_angle)
                way_points_meters.append((x, y))
            
            # Draw lines between consecutive points
            for i in range(len(way_points_meters) - 1):
                x1, y1 = way_points_meters[i]
                x2, y2 = way_points_meters[i + 1]
                self.draw_line(grid, x1, y1, x2, y2, origin_x, origin_y, width, height)
        
        # Create OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = self.map_frame
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        
        # Set origin (지도가 RViz2 중심에 오도록)
        occupancy_grid.info.origin.position.x = origin_x
        occupancy_grid.info.origin.position.y = origin_y
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        
        occupancy_grid.data = grid.flatten().tolist()
        
        self.get_logger().info(f'Created occupancy grid: {width}x{height} pixels')
        
        return occupancy_grid

    def get_road_width(self, tags):
        """Get road width based on highway type and lanes"""
        # 기본 차선 폭 3.5m
        lane_width = 3.5
        
        # 차선 수 확인
        lanes = tags.get('lanes', '1')
        try:
            num_lanes = int(lanes)
        except:
            num_lanes = 1
        
        # 도로 유형별 기본 차선 수
        highway_type = tags.get('highway', 'service')
        if highway_type in ['motorway', 'trunk']:
            num_lanes = max(num_lanes, 4)
        elif highway_type in ['primary', 'secondary']:
            num_lanes = max(num_lanes, 2)
        elif highway_type == 'residential':
            num_lanes = max(num_lanes, 2)
        elif highway_type == 'raceway':
            num_lanes = max(num_lanes, 3)
        
        return num_lanes * lane_width / self.resolution  # 픽셀 단위로 변환

    def draw_thick_line(self, grid, x1, y1, x2, y2, origin_x, origin_y, width, height, thickness, value):
        """Draw a thick line in the grid"""
        # Convert to pixel coordinates
        px1 = int((x1 - origin_x) / self.resolution)
        py1 = int((y1 - origin_y) / self.resolution)
        px2 = int((x2 - origin_x) / self.resolution)
        py2 = int((y2 - origin_y) / self.resolution)
        
        # 두께의 절반
        half_thickness = int(thickness / 2)
        
        # Bresenham's line algorithm with thickness
        dx = abs(px2 - px1)
        dy = abs(py2 - py1)
        x, y = px1, py1
        
        x_inc = 1 if px1 < px2 else -1
        y_inc = 1 if py1 < py2 else -1
        error = dx - dy
        
        while True:
            # 두꺼운 선을 위해 주변 픽셀들도 칠하기
            for dx_offset in range(-half_thickness, half_thickness + 1):
                for dy_offset in range(-half_thickness, half_thickness + 1):
                    px = x + dx_offset
                    py = y + dy_offset
                    if 0 <= px < width and 0 <= py < height:
                        grid[height - 1 - py, width - 1 - px] = value  # X축도 뒤집기
            
            if x == px2 and y == py2:
                break
                
            error2 = 2 * error
            if error2 > -dy:
                error -= dy
                x += x_inc
            if error2 < dx:
                error += dx
                y += y_inc

    def draw_line(self, grid, x1, y1, x2, y2, origin_x, origin_y, width, height):
        """Draw a line in the grid using Bresenham's algorithm"""
        # Convert to pixel coordinates
        px1 = int((x1 - origin_x) / self.resolution)
        py1 = int((y1 - origin_y) / self.resolution)
        px2 = int((x2 - origin_x) / self.resolution)
        py2 = int((y2 - origin_y) / self.resolution)
        
        # Bresenham's line algorithm
        dx = abs(px2 - px1)
        dy = abs(py2 - py1)
        x, y = px1, py1
        
        x_inc = 1 if px1 < px2 else -1
        y_inc = 1 if py1 < py2 else -1
        error = dx - dy
        
        while True:
            # Set pixel as occupied if within bounds
            if 0 <= x < width and 0 <= y < height:
                grid[height - 1 - y, width - 1 - x] = 100  # X축도 뒤집기
            
            if x == px2 and y == py2:
                break
                
            error2 = 2 * error
            if error2 > -dy:
                error -= dy
                x += x_inc
            if error2 < dx:
                error += dx
                y += y_inc

    def create_empty_grid(self):
        """Create an empty occupancy grid"""
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = self.map_frame
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = 100
        occupancy_grid.info.height = 100
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.x = 0.0
        occupancy_grid.info.origin.orientation.y = 0.0
        occupancy_grid.info.origin.orientation.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        occupancy_grid.data = [0] * (100 * 100)
        
        return occupancy_grid

    def publish_map(self):
        """Publish the occupancy grid"""
        if self.map_generated and hasattr(self, 'occupancy_grid') and self.occupancy_grid:
            self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
            self.map_publisher.publish(self.occupancy_grid)

def main(args=None):
    rclpy.init(args=args)
    
    osm_map_publisher = OSMMapPublisher()
    
    try:
        rclpy.spin(osm_map_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        osm_map_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()