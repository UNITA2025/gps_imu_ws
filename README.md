# GPS-IMU Fusion Package

ROS2 기반 GPS와 IMU 센서 데이터 융합을 통한 로봇 위치 추정 패키지입니다.

## 📋 개요

본 패키지는 GPS와 IMU 센서를 융합하여 정확하고 안정적인 로봇 위치 추정을 제공합니다. Extended Kalman Filter(EKF)를 활용하여 각 센서의 장점을 살리고 단점을 보완합니다.

### 주요 기능
- **GPS-IMU 센서 융합**: robot_localization 패키지의 EKF를 활용
- **다중 좌표계 지원**: 글로벌(GPS) 및 로컬(ENU) 좌표계
- **실시간 헤딩 보정**: 직진 구간 감지를 통한 자동 헤딩 드리프트 보정
- **센서 동기화**: 0.1초 이내 시간차로 GPS-IMU 데이터 동기화
- **OSM 지도 발행**: OpenStreetMap 기반 지도 시각화

## 🏗️ 아키텍처

### 센서 구성
- **GPS**: U-blox GPS 수신기 (NavSatFix 메시지)
- **IMU**: UM7 센서 (Imu 메시지)

### 노드 구성

#### 1. 센서 데이터 발행 노드들 (`gps_imu_pub.launch.py`)
```
um7_driver → imu_repub → /imure/data
ublox_gps → gps_repub → /gps/fix
                     ↓
               gps_imu_sync (동기화 확인)
```

#### 2. 위치 추정 노드들 (`gps_imu_robot_localization.launch.py`)
```
/gps/fix ──→ navsat_transform ──→ /odometry/gps
                                        ↓
/imure/data ──→ ekf_local ──→ /odometry/filtered
             ↘                          ↓
               ekf_global ──→ /odometry/global
                                        ↓
               local_origin_setter ──→ /odometry/local_enu
```

## 📦 패키지 구성

### Python 노드들
- **`gps_repub.py`**: GPS 데이터 재발행
- **`imu_repub.py`**: IMU 데이터 재발행  
- **`gps_imu_sync.py`**: GPS-IMU 데이터 동기화 확인
- **`local_origin_setter.py`**: 로컬 원점 설정 및 헤딩 보정
- **`osm_map_publisher.py`**: OSM 지도 발행
- **`odom_to_txt.py`**: Odometry 데이터를 텍스트 파일로 저장

### 설정 파일들
- **`ekf_local.yaml`**: IMU 전용 로컬 EKF 설정
- **`ekf_global.yaml`**: GPS+IMU 글로벌 EKF 설정  
- **`navsat_transform.yaml`**: GPS 좌표 변환 설정
- **`map_kcity.osm`**: K-City 지도 파일
- **`map_inu.osm`**: 인천대학교 지도 파일

### Launch 파일들
- **`gps_imu_pub.launch.py`**: 센서 데이터 발행
- **`gps_imu_robot_localization.launch.py`**: 위치 추정 파이프라인

## 🚀 사용 방법

### 1. 의존성 설치

```bash
# robot_localization 설치
sudo apt install ros-humble-robot-localization

# ublox GPS 드라이버 설치  
sudo apt install ros-humble-ublox-gps

# 기타 의존성
sudo apt install ros-humble-message-filters
```

### 2. 빌드

```bash
cd ~/gps_imu_ws
colcon build --packages-select gps_imu_fusion_pkg
source install/setup.bash
```

### 3. 실행

#### 센서 데이터 발행 시작
```bash
ros2 launch gps_imu_fusion_pkg gps_imu_pub.launch.py
```

#### 위치 추정 시작  
```bash
ros2 launch gps_imu_fusion_pkg gps_imu_robot_localization.launch.py
```

### 4. 토픽 확인

```bash
# 주요 출력 토픽들
ros2 topic echo /odometry/filtered    # 로컬 EKF 결과
ros2 topic echo /odometry/global      # 글로벌 EKF 결과  
ros2 topic echo /odometry/local_enu   # 로컬 원점 기준 결과
```

## 🔧 설정 가이드

### EKF 튜닝

#### 로컬 EKF (`ekf_local.yaml`)
- **용도**: IMU 단독 사용, GPS 불가 환경
- **특징**: 드리프트 발생하지만 부드러운 추정
- **주요 파라미터**:
  ```yaml
  frequency: 100.0  # 높은 주파수로 부드러운 추정
  process_noise_covariance: [낮은 값들]  # 예측 모델 신뢰
  ```

#### 글로벌 EKF (`ekf_global.yaml`)  
- **용도**: GPS+IMU 융합
- **특징**: 절대 위치 정확도 높음
- **주요 파라미터**:
  ```yaml
  frequency: 30.0  # GPS 주파수에 맞춤
  process_noise_covariance: [높은 값들]  # 센서 측정값 더 신뢰
  ```

### 헤딩 보정 튜닝

`local_origin_setter.py`에서 다음 파라미터 조정:

```python
self.min_straight_distance = 0.3      # 직진 최소 거리 (m)
self.max_direction_deviation = 15.0   # 허용 방향 변화 (도)  
self.min_calibration_distance = 0.3   # 보정 간 최소 거리 (m)
```

## 📊 성능 특성

### 정확도
- **GPS 단독**: ±2-5m (환경에 따라 변동)
- **IMU 단독**: 초기 정확하나 시간에 따라 드리프트
- **GPS+IMU 융합**: ±1-2m (안정적)

### 업데이트 주파수
- **IMU**: 100Hz (부드러운 자세 추정)
- **GPS**: 1-10Hz (절대 위치 보정)
- **융합 결과**: 30Hz (실시간 활용 가능)

### 지연시간
- **센서 동기화**: 0.1초 이내
- **전체 파이프라인**: 0.1-0.2초

## 🗺️ 좌표계 설명

### 프레임 구성
```
map (글로벌 지도 좌표계)
└── odom (주행거리계 좌표계)  
    └── base_link (로봇 중심)
        ├── gps (GPS 센서 위치)
        └── imure_link (IMU 센서 위치)
```

### 변환 관계
- **GPS 글로벌 → 로컬**: `navsat_transform_node`
- **원점 설정**: `local_origin_setter` (첫 GPS 위치를 (0,0)으로)
- **헤딩 보정**: 직진 구간에서 자동 보정

## 🔍 트러블슈팅

### 일반적인 문제들

#### 1. GPS 신호 수신 불량
```bash
# GPS 상태 확인
ros2 topic echo /gps/fix

# 해결방법
- 실외 개방된 곳에서 테스트
- GPS 안테나 위치 조정
- 위성 개수 확인 (최소 4개 이상)
```

#### 2. IMU 캘리브레이션
```bash
# IMU 데이터 확인  
ros2 topic echo /imure/data

# 해결방법
- 센서 초기 정적 상태에서 캘리브레이션
- 자기장 간섭 확인
- 센서 장착 방향 확인
```

#### 3. 헤딩 드리프트
```bash
# 헤딩 보정 로그 확인
ros2 topic echo /odometry/local_enu

# 해결방법  
- 직진 구간에서 자동 보정 대기
- 보정 파라미터 조정
- IMU 캘리브레이션 재확인
```

### 디버깅 도구

```bash
# 센서 동기화 상태 확인
ros2 run gps_imu_fusion_pkg gps_imu_sync

# TF 트리 확인
ros2 run tf2_tools view_frames

# RViz로 시각화
rviz2 -d config/gps_imu_visualization.rviz
```

## 📈 개발 히스토리

- **2025.08.18**: 초기 버전 완성 (송준상, 이다빈, 신민규)
- 헤딩 자동 보정 기능 추가
- OSM 지도 발행 기능 구현
- 다중 EKF 파이프라인 구축

## 📄 라이선스

Apache License 2.0

## 👥 기여자

- **송준상**
- **이다빈**  
- **신민규**

---

## 🔗 관련 링크

- [robot_localization 공식 문서](http://docs.ros.org/en/humble/p/robot_localization/)
- [ROS2 Navigation Stack](https://navigation.ros.org/)
- [UM7 IMU 사용자 가이드](https://github.com/ch-robotics/um7)