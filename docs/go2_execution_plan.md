# Go2 실행 계획

## 목표

* 현재 조건: `Unitree Go2 EDU`, 내장 `L1 LiDAR`, 내장 lidar IMU, 외부 PC, `Ubuntu 22.04 + ROS 2 Humble`
* 최종 목표: `3D map`과 `2D nav map`을 같은 좌표계에서 다루고, 이후 `Nav2`까지 연결
* 메인 프로젝트: `/home/cvr/Desktop/sj/go2_slam_nav`
* 외부 dependency: `/home/cvr/Desktop/sj/go2_stack_ws/src/point_lio_ros2`

---

## Track A (검증 완료)
**구현 사항: `/utlidar/cloud_base_synced -> /lidar_scan -> slam_toolbox`**
👉 **상태: 2D map 생성은 성공**

* `time_sync_bridge.py + lidar_scan_bridge.py + slam_toolbox` 조합으로 `/map`, `/map_metadata` publish까지 확인
* 빠른 파이프라인 검증용으로는 유효
* 다만 synthetic 2D scan 품질과 odom drift 때문에 최종 경로로는 비추천

---

## Track B (메인 후보 유지)
**구현 사항: `/utlidar/cloud + /utlidar/imu` 기반 3D SLAM**
👉 **상태: 여전히 메인 후보**

* 현재 공개 근거가 가장 직접적인 경로는 `autonomy_stack_go2`
* `Go2 built-in L1 + Ethernet` 기준으로 재현 근거가 가장 강함
* Humble에서 지연 리스크가 있어도, 구조적으로는 현재 가장 자연스러운 경로

---

## Track C (실험 진행 중)
**구현 사항: `point_lio_ros2 + pointlio_*_synced`**
👉 **상태: 초기화와 3D map 출력은 성공, 품질은 아직 불안정**

### 현재 구성

* 메인 프로젝트의 `time_sync_bridge.py`가 아래 토픽을 생성
  * 일반 보정 토픽: `/utlidar/imu_synced`, `/utlidar/cloud_synced`, `/utlidar/cloud_base_synced`
  * Point-LIO 전용 토픽: `/utlidar/pointlio_imu_synced`, `/utlidar/pointlio_cloud_synced`
* 외부 `point_lio_ros2`에는 우리용 설정/launch 3개만 추가
  * `config/utlidar_synced.yaml`
  * `launch/mapping_utlidar_synced.launch.py`
  * `launch/correct_odom_utlidar_synced.launch.py`
* 위 3개는 메인 프로젝트의 patch 파일로 관리
  * `patches/point_lio_utlidar_synced.patch`

### 현재까지 확인된 점

* Point-LIO 초기화는 정상적으로 완료됨
  * `first lidar time ...`
  * `IMU Initializing: 100%`
* 3D map은 실제로 표시됨
* `time_sync_bridge.py` 중복 실행 때문에 발생하던 `imu loop back`, `lidar loop back` 문제는 정리됨

### 현재 남은 문제

* 정지 상태에서도 로봇 위치가 RViz에서 조금씩 움직이는 drift가 남아 있음
* 바닥 plane이 기울거나 map이 약간 흐트러지는 경우가 있음
* RViz의 `TF_OLD_DATA` 경고는 여전히 남을 수 있음
  * raw Go2 `/tf`와 현재 시간대 TF가 섞이는 경고
  * Point-LIO 초기화 실패와는 분리해서 봐야 함

### 실무 판단

* Track C는 “완전 실패” 단계는 지났고, 현재는 “실험 가능한 상태”까지 왔음
* 하지만 정지 drift / 자세 불안정 때문에 아직 최종 경로로 채택하긴 이릅니다
* 따라서 현재 우선순위는 여전히 `Track B > Track C > Track A`

---

## 현재 폴더 역할

### 메인 프로젝트

* `time_sync_bridge.py`: 시간 보정 + Point-LIO 전용 입력 생성
* `lidar_scan_bridge.py`: Track A용 2D scan 변환
* `run_track_a_slam.sh`: Track A 일괄 실행
* `docs/go2_execution_plan.md`: 현재 문서
* `patches/point_lio_utlidar_synced.patch`: 외부 Point-LIO patch

### 외부 패키지

* `point_lio_ros2`: 외부 clone 상태 유지
* 메인 프로젝트에는 전체 복사하지 않고 patch만 보관

---

## 현재 결론

* `Track A`: 검증 완료, 디버그/데모용으로 유지
* `Track B`: 현재 기준 메인 후보
* `Track C`: Point-LIO 초기화와 3D map 출력은 확인했지만, 품질 안정화가 더 필요
