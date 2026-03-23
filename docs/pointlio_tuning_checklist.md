# Point-LIO 튜닝 체크리스트

## 목표

`Point-LIO`에서 아래 두 문제를 줄이기 위한 A/B 테스트 절차를 정리한다.

* 정지 또는 짧은 구간에서도 벽이 두꺼워지거나 두 겹으로 보임
* 같은 곳을 다시 지날 때 벽이 옆으로 복제됨

첫 번째는 `front-end 오차`, 두 번째는 `누적 drift` 성격이 강하다.

---

## 전제

현재 기본 설정 파일:

* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml)

현재 주요 값:

* `imu_time_inte: 0.01`
* `gravity_align: false`
* `extrinsic_est_en: false`
* `publish_odometry_without_downsample: false`
* `pcd_save_en: true`

주의:

* 현재 Go2 기본 설정은 이 저장소의 `config/utlidar_synced.yaml`로 관리한다.
* 외부 `point_lio_ros2`는 원본 clone 상태로 유지한다.
* raw 입력 비교용 launch:
  * [go2_pointlio_rawsynced_mapping.launch.py](/home/cvr/Desktop/sj/go2_slam_nav/launch/go2_pointlio_rawsynced_mapping.launch.py)

---

## 먼저 확인할 것

### 1. 시간축

아래가 정상이어야 한다.

```bash
ps -ef | grep time_sync_bridge.py
ros2 topic info -v /utlidar/pointlio_imu_synced
ros2 topic info -v /utlidar/pointlio_cloud_synced
```

정상 기준:

* `time_sync_bridge.py` 1개
* 각 Point-LIO 입력 토픽 publisher 1개

이게 틀리면 나머지 튜닝은 의미가 작다.

### 1-1. 입력 경로 선택

현재 Point-LIO 비교 경로는 둘로 나뉜다.

* 기존 경로
  * `/utlidar/pointlio_cloud_synced`
  * `/utlidar/pointlio_imu_synced`
* 단순화 경로
  * `/utlidar/cloud_synced`
  * `/utlidar/imu_synced`
  * launch에서 extrinsic 명시

빠른 직진/횡이동/회전에서만 map이 찢어지면,
기존 `pointlio_*` 가공 입력보다 단순화 경로를 먼저 비교한다.

### 2. IMU 초기화

실행 직후:

* `IMU Initializing: 100.0 %`까지 완료될 때까지 로봇을 가만히 둔다.

초기화 중 흔들리면 이후 모든 비교가 흐려진다.

---

## 테스트 순서

### A. 정지 테스트

방법:

1. Point-LIO 실행
2. 로봇 제자리 정지 10초
3. RViz에서 벽 두께와 점군 흔들림 관찰

실패 신호:

* 벽이 서서히 두꺼워짐
* 기둥이 퍼짐
* 정지 중에도 벽 가장자리가 번짐

이 경우는 `front-end 오차`를 먼저 줄여야 한다.

### B. 직선 왕복 테스트

방법:

1. 벽을 따라 3~5 m 직진
2. 같은 경로로 복귀

실패 신호:

* 복귀 후 같은 벽이 옆에 또 생김
* path는 비슷한데 벽 두께가 크게 증가

이 경우는 `front-end 오차` 또는 약한 누적 drift다.

### C. 작은 loop 테스트

방법:

1. 실내에서 작은 사각 루프 한 바퀴
2. 시작점으로 복귀

실패 신호:

* 시작점 벽이 평행하게 두 겹
* 시작점 바닥/벽이 옆으로 복제

이 경우는 `loop closure 없는 Point-LIO 단독 한계`일 가능성이 크다.

---

## A/B 항목

### 1. `imu_time_inte`

파일:

* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L22)

의미:

* IMU propagation 주기와 관련
* 실제 IMU rate와 어긋나면 벽 두께와 자세 오차가 커질 수 있음

추천 비교:

* A: `0.01`
* B: `0.005`

판정:

* 정지 상태 벽 두께가 더 얇은 쪽 채택
* 회전 후 벽 모서리가 덜 번지는 쪽 채택

### 2. `gravity_align`

파일:

* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L39)

의미:

* 월드 Z축을 중력 방향에 정렬

추천 비교:

* A: `false`
* B: `true`

판정:

* 평평한 실내에서 벽/바닥이 더 안정적인 쪽 채택
* `true`로 했을 때 맵 전체가 이상하게 기울면 바로 제외

### 3. `extrinsic`

파일:

* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L43)

의미:

* LiDAR가 IMU body frame에서 어디 달려 있는지

관찰 포인트:

* 회전할 때 벽이 옆으로 번짐
* 기둥이 원통 대신 찢어진 타원처럼 보임
* 바닥과 벽 경계가 흐림

판정:

* 직진보다 회전에서 오차가 커지면 extrinsic 의심

참고:

* 이건 값이 조금만 틀려도 영향이 크다.
* 여기부터는 단순 파라미터 튜닝보다 센서 장착 기준 재확인이 더 중요하다.

### 4. IMU noise 관련 항목

파일:

* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L27)
* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L28)
* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L29)
* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L30)
* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L31)
* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L32)
* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L33)
* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L34)

의미:

* IMU 측정값과 bias drift를 얼마나 신뢰할지

우선순위:

* `imu_time_inte`
* `gravity_align`
* `extrinsic`

를 먼저 보고, 그래도 남을 때만 만진다.

이유:

* noise 파라미터는 영향이 넓고, 잘못 건드리면 전체 거동이 더 불안정해지기 쉽다.

### 5. downsample / 출력 주기

관련 선언:

* [parameters.cpp](/home/cvr/Desktop/sj/go2_stack_ws/src/point_lio_ros2/src/parameters.cpp#L51)
* [parameters.cpp](/home/cvr/Desktop/sj/go2_stack_ws/src/point_lio_ros2/src/parameters.cpp#L59)
* [parameters.cpp](/home/cvr/Desktop/sj/go2_stack_ws/src/point_lio_ros2/src/parameters.cpp#L60)
* [utlidar_synced.yaml](/home/cvr/Desktop/sj/go2_slam_nav/config/utlidar_synced.yaml#L49)

의미:

* 시각적으로 더 매끈해 보이게 만들 수는 있음
* 하지만 누적 drift 자체를 해결하지는 못함

이 항목은 마지막에 본다.

---

## 해석 기준

### 정지 중에도 벽이 계속 두꺼워진다

우선순위:

1. sync
2. `imu_time_inte`
3. `gravity_align`
4. `extrinsic`

### 직진은 괜찮은데 회전 후 벽이 퍼진다

우선순위:

1. `extrinsic`
2. IMU noise
3. `gravity_align`

### 다시 같은 곳을 지나면 벽이 두 겹이다

이건 보통 `Point-LIO 단독 한계`다.

즉:

* front-end는 충분히 좋더라도
* loop closure가 없으면 재방문 drift는 남을 수 있다.

이 경우 다음 단계는:

* Point-LIO를 front-end로 유지
* map save/load/localization/loop closure는 backend를 추가

---

## 기록 템플릿

실험마다 아래만 남긴다.

```text
실험명:
변경값:
정지 벽 두께:
회전 후 벽 퍼짐:
직선 왕복 겹침:
작은 loop 복귀 오차:
채택 여부:
```

---

## 현재 권장 순서

1. `imu_time_inte: 0.01` vs `0.005`
2. `gravity_align: false` vs `true`
3. extrinsic 재확인
4. 그 뒤에도 재방문 벽 이중화가 남으면 backend 추가 판단

즉, `Point-LIO`에서 줄일 수 있는 오차와
`Point-LIO 단독이라 남는 오차`를 분리해서 봐야 한다.
