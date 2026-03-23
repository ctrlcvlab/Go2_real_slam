# Go2 실행 계획

## 목표

* 현재 조건: `Unitree Go2 EDU`, 내장 `L1 LiDAR`, 내장 lidar IMU, 외부 PC, `Ubuntu 22.04 + ROS 2 Humble`
* 현재 목표: `Point-LIO` 경로를 안정적으로 유지하고 입력 시간축/센서 설정을 계속 검증
* 메인 프로젝트: `/home/cvr/Desktop/sj/go2_slam_nav`
* 외부 dependency: `/home/cvr/Desktop/sj/go2_stack_ws/src/point_lio_ros2`

## 현재 원칙

* 외부 `point_lio_ros2`는 원본 clone 상태로 유지
* Go2용 설정과 launch는 모두 메인 프로젝트에서 관리
* `RTAB-Map`/하이브리드 경로는 현재 폐기

## 현재 구성

* `time_sync_bridge.py`
  * `/utlidar/pointlio_imu_synced`
  * `/utlidar/pointlio_cloud_synced`
  * 일반 `_synced` 토픽들 생성
* 로컬 설정 파일
  * [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml)
* 로컬 launch 파일
  * [mapping_utlidar_synced.launch.py](/home/cvr/Desktop/sj/go2_slam_nav/launch/mapping_utlidar_synced.launch.py)
  * [correct_odom_utlidar_synced.launch.py](/home/cvr/Desktop/sj/go2_slam_nav/launch/correct_odom_utlidar_synced.launch.py)
  * [go2_pointlio_rawsynced_mapping.launch.py](/home/cvr/Desktop/sj/go2_slam_nav/launch/go2_pointlio_rawsynced_mapping.launch.py)

## 현재 판단

* Point-LIO 초기화와 3D map 출력은 확인됨
* 먼저 볼 이슈는 시간축, IMU 초기화, extrinsic, IMU 가속도 입력 품질
* 외부 repo에 우리 파일을 다시 추가하지 않고, 이 저장소에서만 조정한다
