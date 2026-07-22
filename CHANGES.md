# swan9910 fork 변경사항 & 사용법

## 배경

기존 upstream 4개 submodule 모두 개인 fork 로 옮겨 튜닝/알고리즘 수정 백업.

| Submodule | 이전 upstream | 신규 origin (swan9910 fork) |
|---|---|---|
| algorithm_test | swan9910/A4VAI-TestCode_swan | (동일, main 커밋 추가) |
| ego-planner-a4vai | swan9910/ego-planner-a4vai | (동일) |
| pathfollowing | dailydi2/A4VAI-PathFollowing | **swan9910/A4VAI-PathFollowing** |
| pathplanning | Brightestsama/A4VAI-PathPlanning | **swan9910/A4VAI-PathPlanning** |

pathfollowing / pathplanning 은 dailydi2 / Brightestsama 원본을 `upstream` 리모트로 보존.

---

## 각 submodule 변경 내역

### 1. pathfollowing (swan9910/A4VAI-PathFollowing)

두 브랜치로 분리:

#### `main` — 알고리즘 변경
커밋: `algo: restore original Kp_pos formula, tilt limit 30°, yaw slew + NDO freeze`

- **guidance_path_following.py**
  - `Kp_pos = desired_speed / max(err, desired_speed)` — 원본 공식 복원 (terminal 감속)
  - `sintheta` clip [-1, 1] — tilt 30° 원본 복원 (이전에 실수로 0.5236 로 강제)
- **node_att_ctrl.py**
  - Yaw rate slew limiter (`MAX_YAW_RATE = 1 rad/s`) — 이륙 초기 요 폭주 완화
  - NDO freeze when `fusion_weight < 0.5` — CA 중 disturbance 잘못 학습 방지

#### `swan-tune` — 튜닝값
커밋: `tune: desired_speed 5→4 m/s, virtual_target_distance 6→12 m`

- **quadrotor_iris_parameters.py**
  - `desired_speed`: 5.0 → 4.0
  - `virtual_target_distance`: 6.0 → 12.0

**용도**: 알고리즘 변경 (main) 과 실험적 튜닝 (swan-tune) 을 분리 관리. 실기체 배포는 main + 필요 튜닝 cherry-pick.

---

### 2. pathplanning (swan9910/A4VAI-PathPlanning)

브랜치: `main`
커밋: `fix(env): skip dead imports, use IMG_0268_1000.png heightmap`

- **Plan2WP.py**
  - `tensorrt`, `pycuda`, `torch.nn.functional` dead imports 주석 (컨테이너에 미설치)
  - `mpl_toolkits.mplot3d` 명시 import 제거 (신 matplotlib 자동 등록)
  - `plot_path_3d` 호출 주석 (3D projection 이슈 회피)
  - `image_path`: `expanded-1000.png` → `IMG_0268_1000.png` (실사용 heightmap)
- **setup.py**: version `sac-v2.0.0` → `2.0.0`

**모두 환경 셋업 fix** — 알고리즘 로직 변경 없음.

---

### 3. algorithm_test (swan9910/A4VAI-TestCode_swan)

브랜치: `main`
커밋: `feat: mode_switcher tuning + realtime UDP viz (mode/flight/PSO)`

#### 수정 파일

- **`ca_pf_integrated_test/launch_integration_cpp.sh`**
  - `--pf-cap 1.0`, `--ramp-time 0.5s`
  - `--dist-ca-enter 8.0 --dist-pf-enter 10.0` (히스테리시스)
  - `flight_streamer` / `mode_broadcaster` 자동 실행 (수신부: 100.68.0.70)

- **`ca_pf_integrated_test/mode_switcher.py`**
  - **비대칭 z-slab**: 위쪽 SLAB_UP=4/3m, 아래쪽 SLAB_DOWN=2.5/1.5m (PF/CA 모드별)
  - **body-y filter** (SLAB_BODY_Y=4/3m) — 드론 진행 방향 좌우 폭 제한
  - `_cb_odom` 에서 quaternion → yaw 추출 후 body frame 회전 반영

- **`lib/plot_overlay.py`** — 실비행 궤적 오버레이 개선

- **`path_planning_unit_test/pp_only_integration.sh`**
  - **한번에 여러 GPS 좌표 paste 지원** (`36.72, 127.44 / 36.73, 127.45 / ...`)
  - PSO 완료 시 `pp_path_broadcaster.py` 자동 호출

- **`path_planning_unit_test/convert_waypoints.py`** — 파라미터/포맷 정리

#### 신규 파일 (실시간 UDP 시각화 시스템)

- **`ca_pf_integrated_test/mode_broadcaster.py`**
  - `fusion_weight` 구독 → UDP 로 `MODE|pf` 또는 `MODE|ca` broadcast
  - 수신부 매니저: `~/Pictures/mode_display.py`

- **`lib/flight_streamer.py`**
  - 10Hz UDP `POS|ned_x,ned_y,fusion` broadcast
  - 수신부: `~/Pictures/flight_viz.py` (matplotlib 실시간 궤적)

- **`lib/pp_path_broadcaster.py`**
  - PSO 완료 시 한 번 `PSO|row,col;row,col;...` UDP 발행
  - flight_viz 가 이 경로를 heightmap 위 오버레이

---

### 4. ego-planner-a4vai
변경 없음 (COLCON_IGNORE 만 추가). 이 상태로 유지.

---

## Parent (A4VAI-Algorithms-ROS2)

- `.gitmodules` 에서 pathfollowing / pathplanning URL 을 swan9910 fork 로 변경
- 4개 서브모듈 SHA 를 최신 커밋으로 업데이트
- swan9910 fork (`swan` 리모트) 에 push

---

## 사용법

### 클린 clone (fresh checkout)
```bash
git clone --recursive git@github.com:swan9910/A4VAI-Algorithms-ROS2.git
cd A4VAI-Algorithms-ROS2
# 각 submodule 은 main 브랜치가 이미 checkout 되어있음
```

### 튜닝 브랜치 사용 (pathfollowing 만)
```bash
cd pathfollowing
git checkout swan-tune   # desired_speed 4, VT dist 12
# 원상복귀:
git checkout main
```

### upstream 동기화 (원본 리모트 반영)
```bash
cd pathfollowing   # 또는 pathplanning
git fetch upstream
git merge upstream/main
git push origin main
```

### 실시간 시각화 실행 (수신 PC)
```bash
# 수신 PC (100.68.0.70) 에서
python3 ~/Pictures/mode_display.py     # PF/CA 모드 표시 (45678)
python3 ~/Pictures/flight_viz.py       # PSO+실비행 궤적 (45680)
```

송신은 `launch_integration_cpp.sh` 실행 시 자동 시작 (docker 안).

### 통합 시나리오 실행
```bash
# docker container 안
bash /home/user/a4vai_ws/algorithm_test/algorithm_test/ca_pf_integrated_test/launch_pp_pf_ca_integration.sh
# → PSO 경로 계획 → CA/PF 통합 비행 → 자동 overlay 이미지
```

---

## 관련 링크

- Fork: https://github.com/swan9910/A4VAI-PathFollowing
- Fork: https://github.com/swan9910/A4VAI-PathPlanning
- swan9910/A4VAI-TestCode_swan
- swan9910/ego-planner-a4vai
- swan9910/A4VAI-Algorithms-ROS2 (parent)
