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

## 2026-05-29 실제 실행 파일 기준 지속 이동 fallback 재적용

실제 `/home/han/ros2_ws/install/final_pkg/lib/final_pkg/maze_explorer`가 실행 중인 것을 확인했다.
현재 소스에는 이전에 넣은 `makeKeepMovingGoal()`이 없어서, `Goal succeeded` 뒤 `findBestFrontier()`가 실패하면 그대로 정지하는 상태였다.

- `maze_explorer.cpp`
  - `trySendNextGoal()`에서 frontier 실패 시 바로 종료하지 않고 `makeKeepMovingGoal()` 호출
  - `makeKeepMovingGoal()`은 reachable free cell 중 다음 작은 이동 goal을 생성
  - goal 후보 거리: `0.25m ~ 1.60m`
  - 1차: 정상 clearance/bottleneck/blacklist 적용
  - 2차: clearance/bottleneck 완화
  - 3차: blacklist까지 무시하되 시작점 뒤/입구 복귀 구역은 계속 제외

의도:
- 중간 waypoint에서 `Goal succeeded`가 떠도 탐색 노드가 끝난 것처럼 멈추지 않게 한다.
- frontier가 없어도 지도상 갈 수 있는 free cell을 따라 계속 움직이게 한다.
- 출구에서도 별도 종료 판정 없이 계속 바깥쪽 free cell을 향해 진행한다.

## 2026-05-29 모서리 경유 및 벽 스침 완화

좁은 통로는 통과했지만, 코너/모서리 쪽 waypoint를 경유하면서 벽을 살짝 스치는 상황이 발생했다.

- `maze_explorer.cpp`
  - `GOAL_CLEARANCE: 0.07 -> 0.08`
  - `PATH_BOTTLENECK_CLEARANCE: 0.085 -> 0.095`
  - `PREFERRED_GOAL_CLEARANCE: 0.12 -> 0.14`
  - `PATH_BOTTLENECK_WEIGHT: 1.4 -> 1.9`
  - `MIN_FINAL_GOAL_CLEARANCE: 0.11 -> 0.12`
  - `WALL_CLEARANCE_COMFORT = 0.13` 추가
  - frontier/keep-moving goal scoring에 낮은 clearance/bottleneck penalty 추가
  - keep-moving 정상 clearance/bottleneck도 소폭 상향
- `nav2_params.yaml`
  - TEB `min_obstacle_dist: 0.018 -> 0.022`
  - TEB `weight_obstacle: 16.0 -> 18.0`

의도:
- 모서리 바로 옆 waypoint를 덜 고른다.
- 통로를 완전히 막을 정도로 inflation/radius를 키우지 않고, 후보 선택과 TEB 궤적만 중앙 쪽으로 살짝 민다.

## 2026-05-29 실제 장애물 접촉 후 안전 우선 재조정

모서리 스침 수준이 아니라 실제 장애물 접촉이 발생했다.
따라서 "계속 이동"보다 충돌 방지를 우선하도록 explorer 후보와 TEB local planner를 더 보수적으로 조정했다.

- `maze_explorer.cpp`
  - `GOAL_CLEARANCE: 0.08 -> 0.09`
  - `PATH_CLEARANCE: 0.04 -> 0.05`
  - `PATH_BOTTLENECK_CLEARANCE: 0.095 -> 0.11`
  - `PREFERRED_GOAL_CLEARANCE: 0.14 -> 0.16`
  - `PATH_BOTTLENECK_WEIGHT: 1.9 -> 2.4`
  - `WALL_CLEARANCE_COMFORT: 0.13 -> 0.16`
  - `WALL_CLEARANCE_PENALTY_WEIGHT: 3.0 -> 5.0`
  - `MIN_FINAL_GOAL_DISTANCE: 0.40 -> 0.50`
  - `MIN_FINAL_GOAL_CLEARANCE: 0.12 -> 0.13`
  - keep-moving relaxed clearance/bottleneck도 하한 상향
- `nav2_params.yaml`
  - TEB `max_vel_x: 0.18 -> 0.15`
  - TEB `acc_lim_x: 1.4 -> 1.0`
  - TEB footprint radius `0.085 -> 0.09`
  - local/global `robot_radius: 0.085 -> 0.09`
  - TEB `min_obstacle_dist: 0.022 -> 0.03`
  - TEB `weight_obstacle: 18.0 -> 25.0`
  - velocity smoother max/accel/decel도 동일하게 낮춤

의도:
- 좁은 통로 통과성은 남기되, 벽 바로 옆 waypoint와 빠른 코너 진입을 줄인다.
- 장애물 접촉이 난 현재 상태에서는 멈춤보다 충돌 방지를 우선한다.

## 2026-05-29 장애물 2회 접촉 후 local costmap/TEB 추가 보수화

최신 바이너리와 launch parameter가 적용된 상태에서도 장애물 접촉이 2회 발생했다.
확인 결과 local costmap inflation이 `0.07`로 얇아, TEB가 코너에서 벽 가까운 궤적을 만들 여지가 있었다.

- `maze_explorer.cpp`
  - `GOAL_CLEARANCE: 0.09 -> 0.10`
  - `PATH_CLEARANCE: 0.05 -> 0.06`
  - `PATH_BOTTLENECK_CLEARANCE: 0.11 -> 0.12`
  - `PREFERRED_GOAL_CLEARANCE: 0.16 -> 0.17`
  - `PATH_BOTTLENECK_WEIGHT: 2.4 -> 3.0`
  - `WALL_CLEARANCE_PENALTY_WEIGHT: 5.0 -> 7.0`
  - `MIN_FINAL_GOAL_CLEARANCE: 0.13 -> 0.14`
  - keep-moving relaxed clearance/bottleneck도 다시 상향
- `nav2_params.yaml`
  - TEB `max_vel_x: 0.15 -> 0.12`
  - TEB `acc_lim_x: 1.0 -> 0.8`
  - TEB footprint radius `0.09 -> 0.095`
  - local/global `robot_radius: 0.09 -> 0.095`
  - TEB `min_obstacle_dist: 0.03 -> 0.04`
  - TEB `weight_obstacle: 25.0 -> 35.0`
  - local inflation `0.07 -> 0.10`, `cost_scaling_factor: 22.0 -> 18.0`
  - global inflation `0.09 -> 0.10`
  - velocity smoother max/accel/decel도 `0.12`, `0.8` 기준으로 낮춤

의도:
- 실제 접촉이 반복되므로 통과성보다 충돌 방지를 우선한다.
- 특히 local planner가 벽 가까운 trajectory를 만들지 못하게 local inflation과 TEB obstacle penalty를 함께 올린다.

## 2026-05-29 바퀴 걸림 대응

좁은 길을 통과하려는 방향은 좋았지만, 몸통 중심은 지나가고 바퀴 가장자리가 벽/장애물에 살짝 걸리는 증상이 발생했다.

- `nav2_params.yaml`
  - TEB footprint radius `0.095 -> 0.105`
  - local/global `robot_radius: 0.095 -> 0.105`
  - TEB `min_obstacle_dist: 0.04 -> 0.03`
  - TEB `weight_obstacle: 35.0 -> 45.0`
- `maze_explorer.cpp`
  - `PATH_CLEARANCE: 0.06 -> 0.065`
  - `PATH_BOTTLENECK_CLEARANCE: 0.12 -> 0.13`
  - `PREFERRED_GOAL_CLEARANCE: 0.17 -> 0.18`
  - `PATH_BOTTLENECK_WEIGHT: 3.0 -> 3.5`
  - `WALL_CLEARANCE_COMFORT: 0.16 -> 0.18`
  - `WALL_CLEARANCE_PENALTY_WEIGHT: 7.0 -> 8.0`
  - `MIN_FINAL_GOAL_CLEARANCE: 0.14 -> 0.15`
  - keep-moving clearance/bottleneck 하한 소폭 상향

의도:
- planner가 바퀴 폭을 포함한 실제 로봇 외곽을 보게 한다.
- `min_obstacle_dist`를 과하게 키우지 않아 좁은 길 통과성은 유지한다.
- goal 후보와 local trajectory는 벽 중앙 쪽으로 더 강하게 유도한다.

## 2026-05-29 통로 통과성 회복 절충

충돌 방지를 위해 너무 보수적으로 올린 값 때문에 좁은 통로가 costmap/planner 상에서 거의 닫혀 보이고 통과하지 못했다.
속도는 빠르면 코너 충돌 위험이 있지만, 너무 느리면 TEB가 좁은 통로에서 정지/회전으로 빠지는 단점도 있어 중간값으로 되돌렸다.

- `nav2_params.yaml`
  - TEB `max_vel_x: 0.12 -> 0.16`
  - TEB `acc_lim_x: 0.8 -> 1.1`
  - TEB/local/global radius `0.105 -> 0.10`
  - TEB `weight_obstacle: 45.0 -> 30.0`
  - local inflation `0.10 -> 0.08`, `cost_scaling_factor: 18.0 -> 22.0`
  - global inflation `0.10 -> 0.09`, `cost_scaling_factor: 14.0 -> 16.0`
  - velocity smoother도 `0.16` 기준으로 조정
- `maze_explorer.cpp`
  - `GOAL_CLEARANCE: 0.10 -> 0.085`
  - `PATH_CLEARANCE: 0.065 -> 0.05`
  - `PATH_BOTTLENECK_CLEARANCE: 0.13 -> 0.105`
  - `PREFERRED_GOAL_CLEARANCE: 0.18 -> 0.15`
  - `PATH_BOTTLENECK_WEIGHT: 3.5 -> 2.2`
  - `WALL_CLEARANCE_COMFORT: 0.18 -> 0.15`
  - keep-moving clearance/bottleneck도 통과성 쪽으로 완화

의도:
- 저번 빠른 버전의 통과성을 일부 회복한다.
- 바퀴 걸림 방지는 radius `0.10`으로 남기고, 과한 inflation/clearance로 통로가 닫히는 문제를 줄인다.

## 2026-05-29 centerline 기반 새 explorer 추가

파라미터 튜닝만으로는 "좁은 통로 중앙을 따라가면서 빠르게 탈출" 요구를 안정적으로 만족시키기 어렵다고 판단했다.
기존 `maze_explorer.cpp`는 유지하고, 새 알고리즘을 별도 실행 파일로 추가했다.

- 새 파일
  - `src/maze_explorer_centerline.cpp`
- 새 실행 파일
  - `ros2 run final_pkg maze_explorer_centerline`
- `CMakeLists.txt`
  - `maze_explorer_centerline` target 추가

알고리즘:
- `/map` occupancy grid에서 obstacle distance transform을 계산한다.
- 낮은 clearance cell은 경로 비용을 크게 올리고, 높은 clearance cell을 선호하는 Dijkstra를 수행한다.
- frontier를 바로 goal로 찍지 않고, center-biased path 위의 짧은 lookahead waypoint를 Nav2 goal로 보낸다.
- 좁은 통로에서는 양쪽 벽에서 최대한 비슷하게 떨어진 medial/centerline 쪽 cell이 선택된다.
- frontier가 없으면 known free cell 중 시작점에서 더 깊은 cell을 선택해 계속 전진한다.
- 실패 goal은 blacklist에 넣고, succeeded는 탐색 종료로 보지 않고 다음 waypoint를 바로 생성한다.

의도:
- 좁은 통로를 못 지나가는 문제를 inflation만 키워서 해결하지 않는다.
- 장애물 사이 중앙을 따라가는 알고리즘적 기준을 goal 생성 단계에 넣는다.
- 기존 노드를 망가뜨리지 않고 새 노드로 비교 테스트할 수 있게 한다.

## 2026-05-29 centerline 반복 abort 수정

터미널 확인 결과 `maze_explorer_centerline`은 실행 중이고 Nav2 lifecycle도 active였지만,
같은 goal `(-0.90, 0.37)`을 반복 전송한 뒤 Nav2가 계속 abort/recovery로 빠지고 있었다.

- `maze_explorer_centerline.cpp`
  - 실패/거부/cancel된 waypoint뿐 아니라 그 waypoint를 만든 최종 target frontier도 blacklist에 추가
  - center snap 단계에서 blacklist된 lookahead cell을 제외
  - blacklist된 goal pose는 전송하지 않음
  - waypoint에 성공했을 때도 같은 중간 waypoint를 바로 다시 선택하지 않도록 해당 waypoint를 blacklist

의도:
- 실패한 target 주변으로 다시 goal을 찍는 반복 고리를 끊는다.
- Nav2가 출발하기 전에 같은 불가능 goal을 계속 abort하는 상태를 막는다.
- 성공한 중간 waypoint를 탈출 완료로 보지 않고 다음 centerline waypoint로 계속 진행한다.

## 2026-05-29 centerline 폐기, 기존 maze_explorer 개선

`maze_explorer_centerline`은 실제 주행에서 길 선택이 불안정하여 테스트용으로만 두고,
기존 `maze_explorer`를 다시 기준 노드로 사용한다.

현재 문제 정리:
- 기존 explorer는 centerline보다 전체 방향 선택은 좋다.
- 다만 좁은 통로/코너에서 TEB가 global path를 짧게 자르며 벽이나 장애물에 스칠 수 있다.
- 성공한 중간 goal을 blacklist하지 않아 다음 goal 선택이 같은 구역에 머물 수 있다.
- explorer에서 후진을 금지해도 Nav2 recovery의 `BackUp` behavior가 뒤로 빠질 수 있다.

수정:
- `maze_explorer.cpp`
  - 성공한 goal도 blacklist에 추가해 같은 waypoint 주변을 반복 선택하지 않게 함
  - `findCenteredReachableGoalCell()`에서 후보 cell 자체 clearance뿐 아니라 그 cell까지 가는 경로의 bottleneck clearance도 같이 점수화
  - 벽 옆에 점 하나가 좋아 보이는 후보보다, 실제 경로 전체가 조금 더 넓은 후보를 선택
- `nav2_params.yaml`
  - goal checker/TEB `xy_goal_tolerance: 0.09 -> 0.07`
  - TEB `max_global_plan_lookahead_dist: 0.45 -> 0.55`
  - TEB `global_plan_prune_distance: 0.20 -> 0.10`
  - TEB `global_plan_viapoint_sep: 0.08`, `weight_viapoint: 8.0` 추가
  - TEB `weight_obstacle: 30.0 -> 34.0`
  - `NavigateToPose` BT를 후진 recovery가 없는 `navigate_w_replanning_time.xml`로 변경
  - behavior server의 `backup` action server는 BT navigator 활성화에 필요하므로 유지

의도:
- 기존 explorer의 좋은 방향 선택은 유지한다.
- 좁은 통로 통과를 막을 정도로 radius/inflation을 키우지 않는다.
- 코너에서 local planner가 global path를 과하게 shortcut해서 벽을 긁는 현상을 줄인다.
- 중간 goal 성공 후에도 같은 곳에 머물지 않고 다음 후보로 계속 진행한다.

## 2026-05-29 bt_navigator inactive 수정

현장에서 `maze_explorer` 프로세스는 실행 중이고 `/map`도 정상 구독 중이었지만,
`/bt_navigator`만 `inactive [2]` 상태라서 `/goal_pose`, `/cmd_vel_nav`가 새로 나오지 않았다.

로그 원인:
- `bt_navigator`: `"backup" action server not available`
- 기본 `navigate_through_poses_w_replanning_and_recovery.xml`이 `BackUp` 노드를 로딩하는데,
  behavior server에서 `backup` plugin을 제거해 action server가 없어졌다.

수정:
- `behavior_plugins`에 `backup`을 다시 추가
- 단, `maze_explorer`가 쓰는 `NavigateToPose`는 후진 recovery 없는
  `navigate_w_replanning_time.xml`을 사용하도록 변경

의도:
- BT navigator lifecycle이 정상 active가 되게 한다.
- 기존 탐색 주행에서는 후진 recovery를 직접 쓰지 않고, 실패하면 explorer가 다음 goal을 선택하게 한다.

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
