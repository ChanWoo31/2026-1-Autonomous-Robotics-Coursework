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

## 2026-05-28 벽 스침 대응

현재 주행은 잘 되지만 벽에 살짝 닿는 경우가 있어, 통과 우선 설정에서 안전 여유를 한 단계만 되돌렸다.

- `maze_explorer.cpp`
  - `GOAL_CLEARANCE: 0.05 -> 0.07`
  - `PATH_CLEARANCE: 0.03 -> 0.05`
  - `GOAL_STANDOFF: 0.16 -> 0.20`
  - `MIN_FINAL_GOAL_CLEARANCE: 0.10 -> 0.12`
- `nav2_params.yaml`
  - TEB/local/global `robot_radius` 및 footprint radius `0.075 -> 0.085`
  - TEB `min_obstacle_dist: 0.015 -> 0.02`
  - TEB `weight_obstacle: 10.0 -> 14.0`
  - local inflation `0.05 -> 0.07`, `cost_scaling_factor: 25.0 -> 22.0`
  - global inflation `0.08 -> 0.10`, `cost_scaling_factor: 12.0 -> 14.0`

의도:
- goal이 벽/미지영역 바로 옆에 찍히는 것을 줄인다.
- TEB가 통로 중앙 궤적을 조금 더 선호하게 한다.
- 가장 좁은 통로가 TurtleBot 약 1.5대 폭이면 아직 통과 가능한 수준의 보수값으로 유지한다.

후속 튜닝:
- 여전히 벽에 닿으면 radius를 `0.09`, TEB `min_obstacle_dist`를 `0.025`로 올린다.
- 좁은 통로에서 다시 막히면 radius는 유지하고 TEB `min_obstacle_dist`만 `0.015`로 되돌리는 것을 먼저 시도한다.

## 2026-05-28 막힌 쪽 goal 선택 대응

RViz에서 열린 통로가 있는데도 가까운 막힌 쪽 frontier/벽 옆 goal을 선택하고 멈추는 패턴이 확인됐다.

- `maze_explorer.cpp`
  - `computeReachableDistances()`를 `computeReachableMetrics()`로 변경
  - goal까지의 단순 최단거리뿐 아니라 경로 전체의 최소 장애물 거리인 `bottleneck_clearance`를 계산
  - `PATH_BOTTLENECK_CLEARANCE = 0.085`보다 좁은 병목을 통과해야 하는 goal은 제외
  - `PATH_BOTTLENECK_WEIGHT = 1.4`로 더 넓게 갈 수 있는 경로를 점수에서 선호
  - `FRONTIER_CLUSTER_LENGTH_WEIGHT = 0.2`로 작은 노이즈 frontier보다 큰 frontier를 선호
  - 시작 yaw 기준 `max_forward_projection`보다 뒤쪽 goal을 제외하던 필터 제거
  - watchdog에서 `BACKTRACK_CANCEL_DISTANCE` 기준으로 현재 goal을 취소하던 로직 제거

의도:
- costmap 상으로는 거의 막힌 것처럼 보이는 얇은 free-cell 줄을 reachable로 과신하지 않는다.
- 가까운 막다른 후보보다 실제로 넓게 이어지는 통로 쪽 frontier를 고르게 한다.
- 미로가 꺾이는 코너에서 정상적인 우회/회전을 되돌아감으로 오판해 갑자기 멈추지 않게 한다.

## 2026-05-28 입구 복귀 절대 금지

사용자가 입구로 다시 돌아오는 것을 절대 막아달라고 요청했다.

- `maze_explorer.cpp`
  - `max_start_distance_` 추가
  - 시작점에서 `0.90m` 이상 멀어진 뒤에는 입구를 떠난 것으로 판단
  - 입구를 떠난 뒤 시작점 반경 `0.90m` 안의 goal 후보는 전부 제외
  - reachable 계산에서도 시작점 반경 `0.90m` 안을 금지 구역처럼 취급
  - 입구를 떠난 뒤 실제 로봇이 시작점 반경 `0.85m` 안으로 들어오면 현재 goal 즉시 취소 및 blacklist

의도:
- 미로가 꺾이는 구간의 정상적인 우회는 허용한다.
- 단, 시작점/입구 주변으로 돌아가는 goal 선택과 실제 복귀는 강하게 차단한다.

## 2026-05-28 초반 정지와 벽 스침 재대응

입구에서 약 5초 거리만 이동한 뒤 멈추고, 코너에서 벽을 계속 건드리는 문제가 재발했다.

- `maze_explorer.cpp`
  - 입구 복귀 금지 발동 거리 `0.90m -> 1.80m`
  - 입구 goal 금지 반경 `0.90m -> 0.75m`
  - 입구 복귀 취소 반경 `0.85m -> 0.65m`
- `nav2_params.yaml`
  - TEB/Costmap radius `0.085 -> 0.095`
  - TEB `min_obstacle_dist: 0.02 -> 0.025`
  - TEB `weight_obstacle: 14.0 -> 22.0`
  - TEB/velocity smoother `max_vel_x: 0.20 -> 0.16`
  - local inflation `0.07 -> 0.10`, `cost_scaling_factor: 22.0 -> 18.0`
  - global inflation `0.10 -> 0.12`, `cost_scaling_factor: 14.0 -> 12.0`

의도:
- 초반 코너에서 입구 복귀 금지가 너무 빨리 발동해 스스로 멈추는 것을 막는다.
- 코너 진입 속도를 낮추고 로봇 반경/장애물 가중치를 올려 벽에 붙는 궤적을 줄인다.

## 2026-05-28 시작 방향 반전 오판 수정

사용자는 항상 가야 하는 방향으로 로봇을 놓고 시작했는데, goal 선택이 갑자기 반대쪽 frontier를 잡아 로봇이 반대 방향으로 틀었다.

- `maze_explorer.cpp`
  - 잘못 넣었던 `M_PI` 방향 반전 기준 제거
  - 시작 yaw를 그대로 정방향 기준으로 사용
  - 초반 `1.20m` 이동 전에는 시작 yaw 기준 앞쪽 goal만 허용
  - 시작 방향 projection을 goal 점수에 강하게 반영

의도:
- 로봇을 놓은 방향을 그대로 미로 진입 방향으로 믿는다.
- 입구 초반에 옆/뒤쪽 frontier가 가까워도 선택하지 않는다.

## 2026-05-28 경로거리 기반 입구 복귀 차단

로봇이 앞으로 진행했다가 벽 옆 goal에서 실패하고 다시 입구 쪽으로 되감기는 문제가 계속 발생했다.

- `maze_explorer.cpp`
  - 시작점 기준 BFS 거리맵을 추가 계산
  - 현재까지 도달한 최대 시작점 경로거리 `max_start_path_distance_` 추가
  - goal의 시작점 경로거리가 현재 최대 진행 깊이보다 `0.15m` 이상 뒤면 제외
  - 시작점 경로거리 자체를 점수에 강하게 반영해 먼 goal을 우선
  - explorer clearance 상향:
    - `GOAL_CLEARANCE: 0.07 -> 0.09`
    - `PATH_CLEARANCE: 0.05 -> 0.06`
    - `PATH_BOTTLENECK_CLEARANCE: 0.085 -> 0.10`
    - `MIN_FINAL_GOAL_CLEARANCE: 0.12 -> 0.13`

의도:
- yaw/원형 반경 기준이 아니라 실제 free-cell 경로 기준으로 입구 복귀를 차단한다.
- 가까운 벽 옆 frontier보다 시작점에서 더 깊은 frontier를 고른다.
- 벽에 너무 붙은 goal/path 후보를 더 일찍 버린다.

## 2026-05-28 costmap 과보수 완화

벽 접촉을 막기 위해 radius/inflation/clearance를 한 번에 올렸더니 RViz에서 벽이 두껍게 보이고, 로봇이 앞으로 조금 가다 통로를 못 지나가는 문제가 생겼다.

- `maze_explorer.cpp`
  - `GOAL_CLEARANCE: 0.09 -> 0.07`
  - `PATH_CLEARANCE: 0.06 -> 0.04`
  - `PATH_BOTTLENECK_CLEARANCE: 0.10 -> 0.085`
  - `MIN_FINAL_GOAL_CLEARANCE: 0.13 -> 0.11`
- `nav2_params.yaml`
  - TEB/Costmap radius `0.095 -> 0.085`
  - TEB `min_obstacle_dist: 0.025 -> 0.018`
  - TEB `weight_obstacle: 22.0 -> 16.0`
  - TEB/velocity smoother `max_vel_x: 0.16 -> 0.18`
  - local inflation `0.10 -> 0.07`, `cost_scaling_factor: 18.0 -> 22.0`
  - global inflation `0.12 -> 0.09`, `cost_scaling_factor: 12.0 -> 14.0`

의도:
- 입구 복귀 차단은 경로거리 기반으로 유지한다.
- 벽/장애물 inflation이 통로를 막지 않게 하여 좁은 장애물 사이를 다시 통과하게 한다.

## 2026-05-28 전역 경로가 못 지나가는 곳으로 짜이는 문제

RViz에서 global path가 실제 local costmap에서는 막혀 보이는 장애물/벽 옆을 통과하도록 생성됐다.

- `nav2_params.yaml`
  - global costmap plugin 목록에 `obstacle_layer`를 실제로 추가
  - planner `tolerance: 0.3 -> 0.08`
  - planner `allow_unknown: true -> false`

의도:
- 전역 플래너가 SLAM `/map` 빈틈만 보고 live scan 장애물을 무시하는 것을 막는다.
- goal 근처 30cm 대충 허용 때문에 벽 옆/막힌 쪽으로 경로가 빨려 들어가는 것을 줄인다.
- unknown 영역을 경유하는 전역 경로를 막고, known free space 안에서만 경로를 만들게 한다.

## 2026-05-28 통합 재정리

현재 증상:
- 장애물 사이 좁은 길을 잘 통과하지 못함.
- 앞으로 가다가 중간에 틀어서 입구로 돌아옴.
- 벽과 충돌이 많음.
- `goal failed` 이후 탐색이 끝난 것처럼 멈춤.

수정:
- `maze_explorer.cpp`
  - 최신 `/map`을 저장하고, `goal failed`/goal reject 이후 즉시 다음 frontier goal을 다시 선택
  - callback 내부 `sleep_for` 제거
  - 주행 중에도 시작점 기준 BFS 경로거리 `current_start_path_distance_`를 갱신
  - 현재 경로거리가 최대 진입 깊이보다 `0.30m` 이상 줄면 입구 방향 되감기로 판단하고 현재 goal 취소
  - failed/cancel goal은 blacklist 후 다음 goal로 진행
- `nav2_params.yaml`
  - TEB 후진 속도 `0.03 -> 0.0`
  - velocity smoother 후진 허용 `-0.03 -> 0.0`
  - frontier goal checker tolerance `0.06 -> 0.09`
  - global costmap plugin은 다시 static + inflation 중심으로 유지하여 좁은 통로가 live scan 잡음으로 닫히지 않게 함

의도:
- 좁은 통로 통과성은 유지한다.
- 로봇이 뒤로 빠지거나 입구 쪽으로 되감기는 것을 명령 단계와 goal 선택 단계에서 모두 막는다.
- `goal failed`가 탐색 종료처럼 보이지 않게 즉시 다음 후보를 선택한다.

## 2026-05-28 goal succeeded 후 중간 정지 대응

좁은 장애물 사이에 들어간 뒤 Nav2가 `goal succeeded`를 띄우고, 실제 탈출은 하지 않은 채 멈추는 문제가 발생했다.

- `maze_explorer.cpp`
  - `GOAL_STANDOFF: 0.16 -> 0.10`
  - `MIN_FINAL_GOAL_DISTANCE: 0.40 -> 0.55`
  - `goal succeeded`가 뜬 goal도 blacklist에 추가
  - `goal succeeded`, `goal failed`, goal reject 이후 즉시 다음 goal 선택
  - frontier 후보가 없으면 시작점 기준 경로거리가 더 깊어지는 known free cell을 fallback goal로 선택
  - fallback goal은 현재 최대 시작점 경로거리보다 `0.25m` 이상 더 깊은 cell만 사용

의도:
- 좁은 통로 중간 waypoint에 도착했다고 탐색을 끝내지 않는다.
- 이미 도착한 중간 지점을 다시 goal로 잡지 않는다.
- frontier가 애매해도 출구 방향으로 더 깊은 known free space를 따라 계속 진행한다.

## 2026-05-28 출발 방향 전진-only 하드 제약

사용자는 로봇을 항상 가야 하는 방향으로 놓고 시작하므로, 시작 yaw를 미로 진입 방향으로 확정했다.

- `maze_explorer.cpp`
  - 시작 yaw 기준 전진 투영값 `current_start_projection_`, `max_start_projection_` 추가
  - goal 후보는 `current_start_projection_ + 0.10m` 이상, 그리고 지금까지 최대 전진값보다 `0.04m` 이상 앞에 있을 때만 허용
  - 로봇 현재 위치에서 reachable BFS를 만들 때도 현재 전진값보다 `0.08m` 이상 뒤쪽 cell은 확장하지 않음
  - 주행 중 전진 투영값이 시작점 뒤로 `0.08m` 이상 가거나, 최대 전진값보다 `0.18m` 이상 줄면 현재 goal 즉시 취소
  - frontier score와 fallback known-free-cell score 모두 전진 투영값이 큰 goal을 강하게 선호

의도:
- 입구 쪽/출발 뒤쪽 goal을 점수로만 피하지 않고 후보군에서 제거한다.
- global path가 뒤로 돌아서 입구 방향으로 나가는 후보를 reachable 단계에서 차단한다.
- `goal succeeded`/`goal failed` 뒤에도 다음 goal은 반드시 출발 지점보다 앞쪽으로만 이어진다.

## 2026-05-28 중간 goal succeeded 후 정지 방지

Nav2가 통로 중간 waypoint에 도착해 `Goal succeeded`를 낸 뒤, 전진-only/frontier/deep-progress 조건을 모두 만족하는 다음 goal이 없어 `선택 가능한 탈출 목표를 찾지 못했습니다`만 반복되는 문제가 발생했다.

- `maze_explorer.cpp`
  - `makeKeepMovingGoal()` 비상 fallback 추가
  - frontier goal이 없고 deep-progress goal도 없으면, 현재 reachable known free cell 안에서 작은 다음 이동 목표를 강제로 선택
  - 비상 fallback goal 조건:
    - 로봇에서 `0.25m ~ 1.20m` 거리
    - 시작점보다 앞쪽 projection 유지
    - 현재 projection보다 `0.12m` 이상 뒤로 가지 않음
    - 최소 clearance `0.06m`, bottleneck `0.04m`
    - 성공했던 같은 goal을 바로 다시 찍지 않도록 작은 blacklist 반경 `0.08m` 적용

의도:
- `Goal succeeded`가 탈출 완료가 아니면 멈추지 않고 다음 작은 목표를 계속 생성한다.
- frontier가 아직 안 생겼거나 너무 엄격한 조건 때문에 후보가 없을 때도 로봇이 정지 상태로 끝나지 않게 한다.

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
| `BLACKLIST_RADIUS` | `0.15` | 실패한 goal 주변을 다시 선택하지 않는 반경 |
| `GOAL_CLEARANCE` | `0.05` | goal 주변에 필요한 최소 빈 공간 |
| `PATH_CLEARANCE` | `0.03` | BFS 경로 탐색 때 벽에서 떨어질 거리 |
| `GOAL_STANDOFF` | `0.20` | frontier 바로 앞이 아니라 로봇 쪽으로 물러나는 거리 |
| `GOAL_SEARCH_RADIUS` | `0.22` | frontier 후보 주변에서 더 안전한 goal cell을 찾는 반경 |
| `PREFERRED_GOAL_CLEARANCE` | `0.12` | 가능하면 선호하는 goal 주변 여유 |
| `PATH_DISTANCE_WEIGHT` | `1.0` | 실제 free cell 경로가 짧은 goal을 우선하는 가중치 |
| `CLUSTER_SIZE_WEIGHT` | `0.03` | 작은 벽 옆 노이즈보다 큰 frontier를 선호하는 가중치 |
| `MIN_GOAL_DISTANCE` | `0.35` | 너무 가까운 goal을 무시하는 거리 |
| `START_LINE_MARGIN` | `0.05` | 시작선보다 뒤쪽 goal을 제외하는 기준 |
| `BACKTRACK_GOAL_ALLOWANCE` | `0.25` | 지나온 길 뒤쪽 goal 허용 범위 |
| `BACKTRACK_CANCEL_DISTANCE` | `0.35` | 주행 중 되돌아간다고 판단하는 거리 |
| `STUCK_GOAL_IMPROVEMENT` | `0.03` | goal까지 이만큼 가까워져야 진행 중으로 판단 |
| `STUCK_TIMEOUT` | `12.0` | 진행이 없을 때 goal을 포기하는 시간 |
| `MIN_FRONTIER_CLUSTER_SIZE` | `6` | 작은 frontier 노이즈 제거 기준 |

ROS parameter화는 한 번 적용했지만, 사용자가 원해서 현재는 되돌렸다.

## 좁은 통로 통과 튜닝

좁은 통로에서 멈추는 증상 때문에 아래처럼 통로 통과 여유를 완화했다.

- `maze_explorer.cpp`
  - `PATH_CLEARANCE: 0.10 -> 0.08`
  - `GOAL_CLEARANCE: 0.12 -> 0.10`
- `nav2_params.yaml`
  - TEB `min_obstacle_dist: 0.06 -> 0.03`
  - TEB `weight_obstacle: 40.0 -> 25.0`
  - local/global inflation `cost_scaling_factor: 8.0 -> 10.0`

스크린샷 기준으로 아직 통로를 통과하지 못해 통과 우선 값으로 한 번 더 완화했다.

- `maze_explorer.cpp`
  - `PATH_CLEARANCE: 0.08 -> 0.06`
  - `GOAL_CLEARANCE: 0.10 -> 0.08`
- `nav2_params.yaml`
  - TEB `max_vel_x: 0.22 -> 0.14`
  - TEB `max_vel_x_backwards: 0.05 -> 0.10`
  - TEB `footprint_model.radius: 0.11 -> 0.09`
  - TEB `allow_init_with_backwards_motion: True`
  - TEB `max_global_plan_lookahead_dist: 0.7`
  - TEB `global_plan_prune_distance: 0.25`
  - TEB `feasibility_check_lookahead_distance: 0.25`
  - TEB `min_obstacle_dist: 0.03 -> 0.01`
  - TEB `penalty_epsilon: 0.05 -> 0.02`
  - TEB `weight_kinematics_forward_drive: 30.0 -> 5.0`
  - TEB `weight_obstacle: 25.0 -> 12.0`
  - velocity smoother `min_velocity.x: -0.05 -> -0.10`
  - local/global `robot_radius: 0.105 -> 0.09`
  - local/global `inflation_radius: 0.12 -> 0.09`
  - local/global `cost_scaling_factor: 10.0 -> 15.0`

주의:
- 이 설정은 안전 여유를 줄이고 통과를 우선한다.
- 벽에 스치면 `robot_radius`와 TEB `footprint_model.radius`를 `0.095` 또는 `0.10`으로 되돌린다.

의도:
- frontier 후보와 BFS가 좁은 통로를 도달 불가로 버리지 않게 한다.
- TEB가 로봇 footprint 바깥에 과한 여유를 요구해서 통로 중앙에서 멈추는 현상을 줄인다.
- inflation cost가 더 빨리 낮아지게 해서 global/local planner가 좁은 free space를 더 잘 사용하게 한다.

## 통로가 지도상 막혀 보이는 경우

스크린샷에서 통로 입구가 검은 점유 셀/두꺼운 wall로 닫혀 보여 추가 조정했다.

- `mapper_params_online_async.yaml`
  - `occupancy_threshold: 0.10 -> 0.25`
- `nav2_params.yaml`
  - global costmap에서 live scan `obstacle_layer` 제거
  - global costmap은 `/map` 기반 static layer + inflation만 사용
- `maze_explorer.cpp`
  - `BLACKLIST_RADIUS: 0.40 -> 0.15`
  - `PATH_CLEARANCE: 0.06 -> 0.03`
  - `GOAL_CLEARANCE: 0.08 -> 0.05`

의도:
- SLAM이 약한 점유 확률까지 벽으로 내보내 통로가 닫히는 현상을 줄인다.
- global plan이 live scan 잡음으로 막히지 않게 하고, 실제 장애물 회피는 local costmap에 맡긴다.
- 한 번 실패한 goal 때문에 좁은 doorway 전체가 blacklist 되는 것을 막는다.

## `goal failed` 재발 대응

RViz에서 goal이 costmap의 벽/인플레이션 영역에 너무 붙어 실패하는 패턴이 보여 goal 선택 방식을 바꿨다.

- `GOAL_STANDOFF: 0.18 -> 0.24`
- `GOAL_SEARCH_RADIUS = 0.22` 추가
- `PREFERRED_GOAL_CLEARANCE = 0.12` 추가
- frontier에서 바로 계산한 한 점을 goal로 쓰지 않고, 주변 reachable free cell 중 벽에서 더 떨어진 cell로 보정한다.

의도:
- frontier 탐색 방향은 유지하면서 Nav2가 거부하기 쉬운 벽 옆 goal을 줄인다.
- 좁은 통로에서는 통로 중앙에 가까운 cell을 goal로 잡게 한다.

## 전진 속도 회복

좁은 통로 대응 중 너무 느려진 주행감을 개선했다.

- `nav2_params.yaml`
  - `controller_frequency: 10.0 -> 15.0`
  - TEB `max_vel_x: 0.14 -> 0.20`
  - TEB `max_vel_x_backwards: 0.10 -> 0.08`
  - TEB `acc_lim_x: 1.0 -> 1.4`
  - TEB `acc_lim_theta: 1.5 -> 2.0`
  - TEB `weight_kinematics_forward_drive: 5.0 -> 12.0`
  - velocity smoother `max_velocity.x: 0.22 -> 0.20`
  - velocity smoother `min_velocity.x: -0.10 -> -0.08`
  - velocity smoother `max_accel.x: 1.0 -> 1.4`

의도:
- 목표 선택이 안정된 뒤에는 전진 속도를 회복한다.
- 후진은 끼었을 때의 탈출용으로만 남기고 전진 주행을 더 선호한다.

## goal이 옆 벽으로 찍히는 문제

사용자가 말한 "앞으로 팍팍 못 감"은 속도보다 goal이 전방이 아니라 벽쪽 frontier로 찍히는 문제였다.

- `maze_explorer.cpp`
  - `/goal_pose` publisher를 `transient_local + reliable` QoS로 변경
  - `FORWARD_PROGRESS_WEIGHT = 1.2` 추가
  - `HEADING_ALIGNMENT_WEIGHT = 0.8` 추가
  - `MIN_HEADING_ALIGNMENT = -0.15` 추가
  - goal 점수에서 현재 로봇 진행 방향과 미로 안쪽 projection을 더 강하게 반영
- `final_navigation.rviz`
  - TF 표시를 `base_link`만 켜진 상태로 정리
  - `/goal_pose` Pose display 추가

의도:
- 가까운 옆벽 frontier보다 전방으로 진행하는 frontier를 우선한다.
- RViz를 나중에 켜도 마지막 `/goal_pose`가 보이게 한다.

후속 수정:
- 전방/yaw 기준을 hard filter로 넣으니 회전 중 후보가 사라져 더 실패했다.
- `GOAL_STANDOFF: 0.24 -> 0.20`
- `FORWARD_PROGRESS_WEIGHT: 1.2 -> 0.35`
- `HEADING_ALIGNMENT_WEIGHT: 0.8 -> 0.20`
- `MIN_HEADING_ALIGNMENT` 제외 필터 제거

의도:
- goal 후보를 지우지 않고, 전방 후보를 약하게 선호만 한다.

재수정:
- 가까운 후보를 너무 우선해서 벽쪽 goal이 계속 찍혔다.
- yaw 기준 선호는 회전 중 로봇에게 맞지 않아 제거했다.
- `MIN_GOAL_DISTANCE: 0.35 -> 0.55`
- `PATH_DISTANCE_WEIGHT = 0.45`
- `FORWARD_PROGRESS_WEIGHT: 0.35 -> 1.10`
- `NEW_PROGRESS_WEIGHT = 0.60`
- 선택 로그에 시작 방향 기준 진행값을 출력한다.

의도:
- 단순히 가까운 frontier가 아니라 시작 방향 기준으로 더 깊이 들어가는 frontier를 우선한다.
- 작은 벽 옆 frontier보다 진행 가능한 큰 frontier 덩어리를 선호한다.

## 코너에서 못 빠져나가는 문제

RViz에서 로봇 주변 local/global costmap inflation이 통로를 너무 두껍게 먹어 TEB가 진행하지 못하는 패턴이 보였다.

- `nav2_params.yaml`
  - TEB `footprint_model.radius: 0.09 -> 0.075`
  - TEB `min_obstacle_dist: 0.01 -> 0.0`
  - TEB `weight_obstacle: 12.0 -> 6.0`
  - local/global `robot_radius: 0.09 -> 0.075`
  - local/global `inflation_radius: 0.09 -> 0.05`
  - local/global `cost_scaling_factor: 15.0 -> 25.0`
- `final_navigation.rviz`
  - TF frame 목록에 wheel/caster/imu/base_scan을 명시적으로 false 처리
  - TF 이름 표시 비활성화

주의:
- 이 설정은 안전 여유를 많이 줄인 통과 우선 설정이다.
- 실제 로봇이 벽에 닿으면 `robot_radius`와 TEB radius를 `0.085~0.09`로 되돌린다.

## 중간에서 이상하게 멈추는 문제

시작 yaw 기준 `forwardProjection`으로 앞으로/뒤를 판단하는 로직이 미로의 꺾임에서 오판을 만들었다.

- `maze_explorer.cpp`
  - goal 선택에서 `START_LINE_MARGIN`, `BACKTRACK_GOAL_ALLOWANCE` projection 필터 제거
  - watchdog의 `BACKTRACK_CANCEL_DISTANCE` 기반 goal 취소 제거
  - `STUCK_TIMEOUT: 8.0 -> 12.0`
  - 점수식을 path distance + clearance + cluster size 중심으로 단순화
- `final_navigation.rviz`
  - TF display 자체를 비활성화

의도:
- 미로가 꺾여도 정상적인 우회/회전을 되돌아감으로 오판하지 않는다.
- 중간에 멋대로 goal을 취소해서 멈추는 상황을 줄인다.

## 직진 가능 구간에서 후진하는 문제

직진하면 통과할 수 있는 상황에서 TEB가 후진 trajectory를 고르는 문제가 있었다.

- `nav2_params.yaml`
  - TEB `max_vel_x_backwards: 0.08 -> 0.03`
  - TEB `allow_init_with_backwards_motion: True -> False`
  - TEB `weight_kinematics_forward_drive: 12.0 -> 100.0`
  - velocity smoother `min_velocity.x: -0.08 -> 0.0`
- `maze_explorer.cpp`
  - 입구 바깥쪽 frontier만 막도록 `START_LINE_MARGIN` 필터를 goal 선택에 복원
  - max-progress/backtrack cancel 로직은 계속 제거 상태 유지

의도:
- 직진 가능한 구간에서 후진으로 빠져나가는 해법을 억제한다.
- 현재 미로 내부에서 회전/우회하는 것은 막지 않되, 입구 밖 goal은 다시 선택하지 않는다.

## 2026-05-28 코너 정지 재발 대응

RViz 스크린샷에서 로봇 footprint가 코너 inflation 영역에 붙고, global path가 벽 cost를 타면서 TEB가 유효한 local trajectory를 찾지 못하는 패턴이 다시 보였다.

- `maze_explorer.cpp`
  - `BLACKLIST_RADIUS: 0.40 -> 0.15`
  - `GOAL_CLEARANCE: 0.12 -> 0.05`
  - `PATH_CLEARANCE: 0.10 -> 0.03`
  - `GOAL_STANDOFF: 0.18 -> 0.20`
  - `STUCK_TIMEOUT: 8.0 -> 12.0`
  - goal 선택의 `BACKTRACK_GOAL_ALLOWANCE` 필터 제거
  - watchdog의 `BACKTRACK_CANCEL_DISTANCE` 기반 goal 취소 제거
- `nav2_params.yaml`
  - TEB/local/global radius `0.09 -> 0.075`
  - local/global inflation `0.08 -> 0.05`
  - local/global `cost_scaling_factor: 12.0 -> 25.0`
  - TEB `min_obstacle_dist: 0.02 -> 0.0`
  - TEB `weight_obstacle: 20.0 -> 6.0`
  - TEB `max_vel_x_backwards: 0.05 -> 0.03`
  - TEB `allow_init_with_backwards_motion: False`
  - global costmap plugin에서 live scan `obstacle_layer` 제외
  - velocity smoother 후진 속도 `min_velocity.x: -0.05 -> 0.0`
- `mapper_params_online_async.yaml`
  - `occupancy_threshold: 0.10 -> 0.25`

의도:
- 지도/전역 costmap이 좁은 코너를 막힌 길처럼 만들지 않게 한다.
- 코너에서 정상적인 회전이나 짧은 자세 조정을 되돌아감으로 오판해 goal을 취소하지 않는다.
- 직진 가능한 코너에서 TEB가 후진 궤적을 고르는 것을 억제한다.

주의:
- 이 값은 통과 우선이라 실제 로봇이 벽에 닿으면 `robot_radius`와 TEB `footprint_model.radius`를 `0.085~0.09`로 되돌린다.

## 2026-05-28 final_pkg_2 방식 교체

`final_pkg_2`는 `final_pkg`의 Nav2 frontier 방식이 잘 안 되는 문제를 피하기 위한 새 실험 패키지다.
따라서 기존 `maze_explorer` 복사본을 계속 튜닝하는 대신, 새 노드 `maze_grid_navigator`를 추가했다.

삭제:
- `maze_escape.cpp`
- `maze_escape_launch.py`

삭제 이유:
- `maze_escape`는 빠르게 통과시키기 위해 만든 라이다 기반 벽 추종 노드였다.
- 교수님 조건상 우수법/좌수법 금지에 걸릴 가능성이 크므로 제출/실험 기본 경로에서 제거했다.

새 방식:
- 노드: `maze_grid_navigator`
- 실행:

```bash
ros2 launch final_pkg_2 maze_grid_navigator_launch.py
```

알고리즘:
- SLAM `/map` occupancy grid를 구독한다.
- `map -> base_link` TF로 현재 위치를 얻는다.
- 점유 셀 주변에 로봇 반경만큼 clearance를 계산한다.
- free cell 중 실제 로봇이 지나갈 수 있는 cell만 traversable grid로 만든다.
- 현재 위치에서 BFS/Dijkstra로 도달 가능한 영역을 계산한다.
- unknown 영역과 맞닿은 frontier cell을 찾는다.
- 후보 goal은 아래 기준으로 점수화한다.
  - 시작점 기준 경로거리가 더 깊은가
  - 현재 위치에서 실제로 도달 가능한가
  - 벽과의 clearance가 충분한가
  - frontier cluster가 너무 작은 노이즈가 아닌가
  - 입구로 되돌아가는 후보가 아닌가
- 선택된 goal까지 A* 경로를 직접 만든다.
- Nav2 action을 쓰지 않고 `/cmd_vel`을 직접 발행한다.
- 경로 추종은 pure pursuit 방식이다.
- 라이다 `/scan`은 전방/측면 충돌 직전 안전 보정에만 사용한다.
- 충분히 진행한 뒤 전방과 양옆이 넓게 열리면 출구로 판단하고 `EXIT_RUN`으로 빠져나간다.

이 방식이 기존 문제에 대응하는 방식:
- Nav2/TEB가 좁은 통로를 inflation 때문에 막힌 길로 판단하는 문제를 피한다.
- goal succeeded가 중간 waypoint에서 뜨고 멈추는 문제를 줄인다.
- frontier goal을 Nav2에 넘기지 않고 직접 A* path를 생성하므로 goal reject/failed 반복을 줄인다.
- 시작점 주변은 일정 거리 이상 진행 후 planning grid에서 차단하여 입구 복귀를 막는다.
- 정체 watchdog이 target을 blacklist하고 즉시 재계획하여 미로 중간 정지를 줄인다.

주의:
- 이 방식도 SLAM `/map` 품질에 의존한다.
- 실제 로봇이 벽에 닿으면 launch parameter에서 `planning_clearance`, `goal_clearance`, `side_guard_distance`를 조금 올린다.
- 좁은 통로를 못 지나가면 `planning_clearance`를 `0.075~0.080`으로 낮춘다.

## 2026-05-29 입구 이탈 방지 보강

질문:
- "입구에서 안 빠져나가게 한 것 맞나?"

확인 결과:
- 기존 `maze_grid_navigator`에는 시작점 주변을 막는 로직이 있었다.
- 하지만 frontier가 없을 때 쓰는 `deep-free` fallback은 초반 뒤쪽 후보를 점수로만 낮추고 완전히 금지하지 않았다.

보강:
- 시작 yaw 기준으로 시작점 뒤쪽 projection `< -0.06m`인 cell은 planning grid에서 완전히 제외한다.
- frontier target과 deep-free target 모두 입구 금지 영역 후보를 버린다.
- 로봇이 충분히 미로 안으로 들어간 뒤 시작점 반경 `entrance_block_radius` 안으로 되돌아오거나 시작선 뒤로 가면 `EntranceGuard` 동작으로 시작 yaw 방향을 다시 바라보고 전진한다.

의도:
- 목표 선택 단계, A* 경로 생성 단계, 실제 주행 단계에서 모두 입구 이탈을 막는다.
- 단, 시작 직후 로봇 주변 몇 cm는 localization/지도 초기화를 위해 예외로 둔다.
