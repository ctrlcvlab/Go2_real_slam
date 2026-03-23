# go2_slam_nav

`Unitree Go2 EDU + built-in L1 LiDAR` 기반 Point-LIO 실험용 메인 폴더입니다.

이 폴더는 메인 프로젝트이고, 외부 `point_lio_ros2`는 원본 그대로 클론한 dependency로 유지합니다.
우리 Go2 전용 설정과 launch는 모두 이 저장소에서 관리합니다.

## 현재 구조

* `time_sync_bridge.py`: Go2 과거 timestamp를 현재 시간대로 보정하고, Point-LIO용 전용 입력 토픽을 생성
* `config/utlidar_synced.yaml`: Go2용 Point-LIO 기본 파라미터
* `launch/mapping_utlidar_synced.launch.py`: 기본 Point-LIO 매핑 launch
* `launch/correct_odom_utlidar_synced.launch.py`: odom-only Point-LIO launch
* `launch/go2_pointlio_rawsynced_mapping.launch.py`: raw synced cloud+imu와 명시적 extrinsic을 쓰는 비교 launch
* `docs/go2_execution_plan.md`: 현재 실행 구조와 운영 원칙
* `docs/pointlio_tuning_checklist.md`: Point-LIO 맵 품질 개선용 A/B 테스트 체크리스트

## 운영 원칙

* 외부 `point_lio_ros2`는 원본 clone 상태로 유지
* Go2용 launch/config는 모두 이 저장소에서 관리
* 외부 dependency patch는 현재 사용하지 않음

## 준비

1. 외부 패키지 준비

```bash
cd /home/cvr/Desktop/sj/go2_stack_ws/src
git clone <point_lio_ros2 원본 저장소>
```

2. 빌드

```bash
cd /home/cvr/Desktop/sj/go2_stack_ws
colcon build --packages-select point_lio --symlink-install
```

## 기본 실행

1. 브리지 실행

```bash
cd /home/cvr/Desktop/sj/go2_slam_nav
python3 time_sync_bridge.py
```

2. Point-LIO 실행

```bash
source /opt/ros/humble/setup.bash
source /home/cvr/Desktop/sj/go2_stack_ws/install/setup.bash
ros2 launch /home/cvr/Desktop/sj/go2_slam_nav/launch/mapping_utlidar_synced.launch.py
```

기본 설정 파일:

```bash
/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml
```

## raw synced 비교 실험

```bash
source /opt/ros/humble/setup.bash
source /home/cvr/Desktop/sj/go2_stack_ws/install/setup.bash
ros2 launch /home/cvr/Desktop/sj/go2_slam_nav/launch/go2_pointlio_rawsynced_mapping.launch.py
```

이 launch는 아래 입력을 그대로 사용합니다.

```bash
/utlidar/cloud_synced
/utlidar/imu_synced
```

즉 `time_sync_bridge.py`의 `pointlio_*` 추가 가공 토픽을 우회하고,
시간만 맞춘 raw 센서 입력과 명시적 extrinsic으로 Point-LIO를 비교하기 위한 경로입니다.

## Point-LIO 입력 안정화 메모

예전에 `Point-LIO` odom이 갑자기 미끄러지며 맵이 쭉 날아가던 문제는
파라미터보다 `입력 시간축 붕괴`가 원인이었습니다.

핵심 원인:

* `time_sync_bridge.py`가 2개 이상 실행되어 같은 `_synced` 토픽에 publisher가 2개 붙음
* `/tf`, `/tf_static`의 과거 timestamp가 IMU/cloud 시간축까지 흔듦
* Point-LIO가 raw `_synced`보다 자기 기준에 맞는 전용 cloud/imu를 쓰는 편이 더 안정적이었음

현재 Point-LIO 기본 입력:

```bash
/utlidar/pointlio_cloud_synced
/utlidar/pointlio_imu_synced
```

같은 증상이 다시 나오면 먼저 아래를 확인합니다.

```bash
ps -ef | grep time_sync_bridge.py
ros2 topic info -v /utlidar/pointlio_imu_synced
ros2 topic info -v /utlidar/pointlio_cloud_synced
```

정상 기준:

* `time_sync_bridge.py` 프로세스 1개
* 각 Point-LIO 입력 토픽 publisher 1개

맵 품질 튜닝 순서와 판정 기준은 아래 문서를 따릅니다.

* [docs/pointlio_tuning_checklist.md](/home/cvr/Desktop/sj/go2_slam_nav/docs/pointlio_tuning_checklist.md)
