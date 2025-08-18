from pyproj import Proj, transform

# WGS84 좌표 (예시, 실제 GPS 좌표 사용)
lon = 126.6322996  # 서울 중심 경도
lat = 37.3743535   # 서울 중심 위도


# latitude: 37.241946399999996
# longitude: 126.7745803
# altitude: 29.893
# 학교
# latitude: 37.3743535
# longitude: 126.6322996
# altitude: 31.533

# WGS84와 UTM zone 52N 정의

wgs84 = Proj(init='epsg:4326')
utm52n = Proj(init='epsg:32652')

# 변환
x, y = transform(wgs84, utm52n, lon, lat)
print(f"UTM Zone 52N 좌표: {x},{y}")