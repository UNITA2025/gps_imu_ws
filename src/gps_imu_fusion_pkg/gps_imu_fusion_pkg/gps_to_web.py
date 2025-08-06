#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import json
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading

class GPSWebViewer(Node):
    def __init__(self):
        super().__init__('gps_web_viewer')
        self.subscription = self.create_subscription(
            NavSatFix, '/fix', self.gps_callback, 10)

        self.gps_data = []
        self.create_html_file()
        self.start_web_server()

    def gps_callback(self, msg):
        point = {
            'lat': msg.latitude,
            'lng': msg.longitude,
            'timestamp': time.time()
        }
        self.gps_data.append(point)

        # JSON 파일 업데이트
        with open('gps_data.json', 'w') as f:
            json.dump(self.gps_data, f)

    def create_html_file(self):
        html_content = """
<!DOCTYPE html>
<html>
<head>
    <title>GPS Tracker</title>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
</head>
<body>
    <div id="map" style="width: 100%; height: 100vh;"></div>

    <script>
        var map = L.map('map').setView([37.3746745, 126.6323098], 15);

        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);

        var polyline = L.polyline([], {color: 'red'}).addTo(map);

        function updateGPS() {
            fetch('gps_data.json')
                .then(response => response.json())
                .then(data => {
                    var latlngs = data.map(point => [point.lat, point.lng]);
                    polyline.setLatLngs(latlngs);

                    if (latlngs.length > 0) {
                        map.setView(latlngs[latlngs.length - 1], map.getZoom());
                    }
                })
                .catch(err => console.log('Error:', err));
        }

        // 2초마다 업데이트
        setInterval(updateGPS, 2000);
        updateGPS();
    </script>
</body>
</html>
        """

        with open('gps_viewer.html', 'w') as f:
            f.write(html_content)

    def start_web_server(self):
        def run_server():
            server = HTTPServer(('localhost', 8000), SimpleHTTPRequestHandler)
            print("웹 서버 시작: http://localhost:8000/gps_viewer.html")
            server.serve_forever()

        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()

def main():
    rclpy.init()
    node = GPSWebViewer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
