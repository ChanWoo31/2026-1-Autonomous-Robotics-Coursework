# TurtleBot3 Maze Navigation Tuning Log

## 상황

- TurtleBot3 Burger로 장애물과 미로 형태의 맵을 탐색 중.
- `slam_toolbox`, `nav2`, `maze_explorer`를 함께 사용.
- 주요 문제:
  - 장애물 근처에서 회피하지 못하고 정지 또는 회전 반복.
  - 너무 느리거나 앞으로 못 감.
  - 중간에 뒤돌아서 입구 쪽으로 되돌아감.
  - 같은 장애물 위치에서 반복적으로 멈춤.

## 실행 구성

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/han/ros2_ws/src/final_pkg/config/mapper_params_online_async.yaml

ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/home/han/ros2_ws/src/final_pkg/config/nav2_params.yaml

ros2 run final_pkg maze_explorer
```

라이다 TF는 실제 장착 방향 때문에 yaw 3.14159 사용.

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 3.14159 \
  --frame-id base_link \
  --child-frame-id laser
```

## `mapper_params_online_async.yaml` 변경

탐색 중 오래된 `/map` 기준으로 goal을 찍는 문제를 줄이기 위해 SLAM 반응성을 높임.

- `map_update_interval: 5.0 -> 1.0`
- `minimum_time_interval: 0.5 -> 0.1`
- `minimum_travel_distance: 0.1 -> 0.03`
- `minimum_travel_heading: 0.1 -> 0.05`
- `max_laser_range: 20.0 -> 4.0`
- `scan_buffer_size: 10 -> 20`
- `scan_buffer_maximum_scan_distance: 10.0 -> 4.0`
- `do_loop_closing: true -> false`

의도:
- 미로 안에서 현재 장애물과 통로가 빠르게 지도에 반영되게 함.
- 먼 거리 라이다 노이즈 영향을 줄임.
- 주행 중 loop closing으로 `map -> odom`이 튀는 현상을 줄임.

## `nav2_params.yaml` 변경

TurtleBot3 Burger 기준으로 반경과 local planner 설정을 정리.

- TEB local planner 사용.
- `robot_radius`를 `0.105` 근처로 통일.
- TEB `footprint_model.radius: 0.11`.
- local/global costmap 해상도 `0.02`.
- inflation radius `0.12`.
- `collision_monitor` 비활성화.
- `planner_server` 추가:
  - `NavfnPlanner`
  - `allow_unknown: true`
  - `use_astar: true`
- `velocity_smoother` 추가.

의도:
- 좁은 통로에서 collision monitor가 `/cmd_vel`을 0으로 만드는 상황을 줄임.
- costmap과 TEB가 같은 로봇 크기를 기준으로 판단하게 함.
- 탐색 중 unknown 경계로 경로를 만들 수 있게 함.

## `maze_explorer.cpp` 변경

### 1. TF 기준 로봇 위치 사용

`/odom` 대신 `map -> base_link` TF로 현재 위치를 구함.

의도:
- `/map` 기준 frontier goal과 로봇 위치 프레임을 맞춤.

### 2. Frontier goal을 바로 찍지 않음

frontier 자체가 아니라, frontier에서 로봇 쪽으로 `GOAL_STANDOFF = 0.18m` 물러난 지점을 goal 후보로 사용.

의도:
- 미지 영역 바로 앞이나 벽 가까이에 goal이 찍혀 Nav2가 실패하는 상황을 줄임.

### 3. 목표 주변 안전 검사

`GOAL_CLEARANCE = 0.12m` 안에 장애물이 있으면 goal 후보에서 제외.

의도:
- goal이 벽/장애물에 너무 붙어 local planner가 도착하지 못하는 상황 방지.

### 4. 작은 frontier cluster 제거

`MIN_FRONTIER_CLUSTER_SIZE = 6`보다 작은 frontier는 노이즈로 보고 제외.

의도:
- 라이다 노이즈나 얇은 지도 깨짐 때문에 이상한 goal을 찍는 것 방지.

### 5. 실패 goal blacklist

실패하거나 Nav2가 거부한 goal 주변 `BLACK_LIST_RADIUS = 0.40m`는 다시 선택하지 않음.

의도:
- 같은 장애물 근처 goal을 반복 선택하는 현상 줄임.

### 6. 입구 방향 방지

시작 위치와 시작 yaw를 저장하고, 시작 방향 기준 뒤쪽 goal은 제외.

- `START_LINE_MARGIN = 0.05`

의도:
- 입구 밖으로 되돌아가는 goal 선택 방지.

### 7. 지나온 길로 크게 되돌아가는 것 방지

지금까지 미로 안쪽으로 가장 깊이 들어간 거리 `max_forward_projection_`를 저장.

- `BACKTRACK_GOAL_ALLOWANCE = 0.25`
- `BACKTRACK_CANCEL_DISTANCE = 0.35`

의도:
- 이미 깊이 들어갔는데 갑자기 지나온 길/입구 쪽 goal을 다시 잡는 것 방지.
- 실제 주행 중 35cm 이상 뒤로 돌아가면 현재 goal 취소.

### 8. Stuck watchdog

1초마다 로봇 위치를 확인.

- `STUCK_GOAL_IMPROVEMENT = 0.03`
- `STUCK_TIMEOUT = 8.0`

8초 동안 목표까지 거리가 3cm 이상 줄지 않으면:
- 현재 goal 취소.
- 현재 goal blacklist 추가.
- 다음 frontier 선택.

의도:
- 장애물 앞에서 계속 회전하거나 맴도는 상황 탈출.

### 9. BFS reachable distance 추가

로봇 현재 위치에서 free cell을 따라 BFS 거리맵을 만들고, 실제로 연결된 goal만 후보로 사용.

- `PATH_CLEARANCE = 0.10`

의도:
- 직선거리로는 가까워 보여도 장애물에 막힌 frontier를 선택하지 않게 함.
- goal 점수를 직선거리 대신 실제 free cell 경로거리 기준으로 계산.

## 현재 튜닝 포인트

같은 장애물 위치에서 계속 멈추는 경우:

- 너무 좁은 통로를 못 지나가면:
  - `PATH_CLEARANCE: 0.10 -> 0.08`
  - `GOAL_CLEARANCE: 0.12 -> 0.10`

- 장애물에 너무 붙으면:
  - `PATH_CLEARANCE: 0.10 -> 0.12`
  - `GOAL_CLEARANCE: 0.12 -> 0.14`

- stuck 판단이 너무 늦으면:
  - `STUCK_TIMEOUT: 8.0 -> 6.0`

- 너무 빨리 goal을 포기하면:
  - `STUCK_TIMEOUT: 8.0 -> 10.0`

- 입구 쪽으로 여전히 돌아가면:
  - `BACKTRACK_GOAL_ALLOWANCE: 0.25 -> 0.15`
  - `BACKTRACK_CANCEL_DISTANCE: 0.35 -> 0.20`

## 빌드 확인

마지막 확인 명령:

```bash
colcon --log-base /tmp/final_pkg_log build \
  --packages-select final_pkg \
  --build-base /tmp/final_pkg_build \
  --install-base /tmp/final_pkg_install
```

결과:

```text
Summary: 1 package finished
```

## 다음에 확인할 것

- RViz에서 `/local_plan`이 장애물을 끼고 같은 곳을 계속 향하는지 확인.
- `maze_explorer` 로그에서 다음 메시지가 뜨는지 확인:
  - `장애물 근처에서 목표 접근 실패`
  - `정체된 목표 취소 완료`
  - `입구 방향으로 되돌아가는 경로 감지`
- 같은 장소에서 반복되면 blacklist 반경과 BFS clearance를 함께 조정.

## 기록 방식

이 대화 내용 전체가 자동으로 다음 세션까지 보존된다고 가정하지 않는다.
중요한 변경사항, 실험 결과, 잘 된 파라미터는 이 파일에 계속 기록한다.

다음에 이어서 작업할 때는 먼저 이 파일을 읽고 현재 상태를 파악한다.

현재 기준으로 기록된 핵심 내용:

- `nav2_params.yaml` 튜닝 내용.
- `mapper_params_online_async.yaml` 튜닝 내용.
- `maze_explorer.cpp` frontier 선택 로직 변경 내용.
- stuck watchdog 추가 내용.
- 입구 방향/되돌아가기 방지 로직.
- BFS 기반 reachable goal 선택 로직.
- ROS parameter화는 시도했지만, 현재는 다시 C++ 고정 상수 방식으로 되돌림.

대화에서 확인한 운영 원칙:

- 라이다 TF는 실제 장착 방향상 `base_link -> laser` yaw `3.14159`를 유지한다.
- 로봇이 잘 가는 상태에서는 큰 구조 변경보다 관찰 가능성과 재현성을 우선한다.
- 새 세션에서 이어서 작업하려면 이 파일을 기준 문서로 사용한다.

## `maze_explorer` 튜닝값 상태

현재 `maze_explorer.cpp`의 주요 튜닝값은 ROS parameter가 아니라 C++ 고정 상수다.
값을 바꾸려면 `maze_explorer.cpp`에서 상수를 수정한 뒤 다시 빌드해야 한다.

기본 실행:

```bash
ros2 run final_pkg maze_explorer
```

현재 주요 상수:

| Constant | Default | 의미 |
| --- | ---: | --- |
| `BLACKLIST_RADIUS` | `0.40` | 실패한 goal 주변을 다시 선택하지 않는 반경 |
| `GOAL_CLEARANCE` | `0.12` | goal 주변에 필요한 최소 빈 공간 |
| `PATH_CLEARANCE` | `0.10` | BFS 경로 탐색 때 벽에서 떨어질 거리 |
| `GOAL_STANDOFF` | `0.18` | frontier 바로 앞이 아니라 로봇 쪽으로 물러나는 거리 |
| `MIN_GOAL_DISTANCE` | `0.35` | 너무 가까운 goal을 무시하는 거리 |
| `START_LINE_MARGIN` | `0.05` | 시작선보다 뒤쪽 goal을 제외하는 기준 |
| `BACKTRACK_GOAL_ALLOWANCE` | `0.25` | 지나온 길 뒤쪽 goal 허용 범위 |
| `BACKTRACK_CANCEL_DISTANCE` | `0.35` | 주행 중 되돌아간다고 판단하는 거리 |
| `STUCK_GOAL_IMPROVEMENT` | `0.03` | goal까지 이만큼 가까워져야 진행 중으로 판단 |
| `STUCK_TIMEOUT` | `8.0` | 진행이 없을 때 goal을 포기하는 시간 |
| `MIN_FRONTIER_CLUSTER_SIZE` | `6` | 작은 frontier 노이즈 제거 기준 |

ROS parameter화는 한 번 적용했지만, 사용자가 원해서 현재는 되돌렸다.
