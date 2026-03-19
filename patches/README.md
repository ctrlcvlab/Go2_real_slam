# patches

외부 dependency에 대한 로컬 변경분만 patch로 보관합니다.

현재 포함된 patch:

* `point_lio_utlidar_synced.patch`
  * 대상: 외부 `point_lio_ros2` 저장소
  * 내용: Go2용 `utlidar_synced` 설정/launch 3개 파일 추가

적용 예시:

```bash
cd /home/cvr/Desktop/sj/go2_stack_ws/src/point_lio_ros2
git apply /home/cvr/Desktop/sj/go2_slam_nav/patches/point_lio_utlidar_synced.patch
```
