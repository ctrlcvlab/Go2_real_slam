# go2_slam_nav

`Unitree Go2 EDU + built-in L1 LiDAR` 기반 SLAM 실험용 메인 폴더입니다.

이 폴더는 메인 프로젝트이고, `point_lio_ros2`는 외부 워크스페이스에 따로 클론한 dependency로 취급합니다.

## 현재 구조

* `time_sync_bridge.py`: Go2 과거 timestamp를 현재 시간대로 보정하고, Point-LIO용 전용 토픽도 같이 생성
* `lidar_scan_bridge.py`: `PointCloud2 -> LaserScan` 변환
* `run_track_a_slam.sh`: Track A 일괄 실행 스크립트
* `slam_toolbox_go2_async.yaml`: Track A용 `slam_toolbox` 설정
* `docs/go2_execution_plan.md`: 현재 트랙별 상태와 판단
* `patches/point_lio_utlidar_synced.patch`: 외부 `point_lio_ros2`에 적용할 우리 변경분

## 현재 트랙 상태

* `Track A`: `/utlidar/cloud_base_synced -> /lidar_scan -> slam_toolbox`
  * 2D `/map` 생성 확인 완료
* `Track B`: `autonomy_stack_go2` 방향
  * 여전히 메인 후보
* `Track C`: `point_lio_ros2 + pointlio_*_synced`
  * Point-LIO 초기화와 3D 맵 출력 확인
  * 정지 상태 drift / 자세 불안정이 남아 있어 실사용 경로로는 아직 보류

## Track A 실행

```bash
cd /home/cvr/Desktop/sj/go2_slam_nav
./run_track_a_slam.sh
```

## Track C 실행

1. 외부 패키지 준비

```bash
cd /home/cvr/Desktop/sj/go2_stack_ws/src
git clone <point_lio_ros2 원본 저장소>
cd point_lio_ros2
git apply /home/cvr/Desktop/sj/go2_slam_nav/patches/point_lio_utlidar_synced.patch
```

2. 빌드

```bash
cd /home/cvr/Desktop/sj/go2_stack_ws
colcon build --packages-select point_lio --symlink-install
```

3. 브리지 실행

```bash
cd /home/cvr/Desktop/sj/go2_slam_nav
python3 time_sync_bridge.py
```

4. Point-LIO 실행

```bash
cd /home/cvr/Desktop/sj/go2_stack_ws
source install/setup.bash
ros2 launch point_lio mapping_utlidar_synced.launch.py
```

## 참고

* `point_lio_ros2` 쪽 우리 변경은 patch로만 관리합니다.
* 실제 Go2 적응 로직의 핵심은 메인 프로젝트의 `time_sync_bridge.py`에 있습니다.
