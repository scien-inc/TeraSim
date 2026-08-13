# Immediate moveToXY / lane-change authority handoff (2026-08-13)

## 結論

現行 `6d14661` を基準に、追加APIをpose専用の `moveToXYImmediate` へ置換した。Phase AではSUMO時間を進めずにCARLA poseを即時同化し、speed/accelerationは既存TraCI APIで反映する。追加APIはSUMO lane-change modelの状態をclear・resetしない。

専用SUMO build、service unit test、Docker integration test、lane-change parityを含む小規模試験は合格した。一方、Odaiba guarded field runは2回ともSUMO時刻524.70秒、`vehicle1167` でCARLA実位置がcurrent laneとSUMO target laneの双方からmatch thresholdを超え、fail closedした。したがってfield acceptanceは未達であり、長時間実走・本採用へは進めていない。

## Git状態とcommit

- branch: `kawai/experiment/sumo-two-phase-movetoxy-20260810`
- baseline: `6d14661 fix: restore bounded restart target for authoritative launch`
- `ba28bb8 refactor: replace external state with pose-only immediate moveToXY`
- `30f1c58 fix: preserve SUMO lane-change state during Phase A`
- `d6eedd2 fix: default external-state assimilation to strict lane projection`
- 本レポートを `docs: record immediate moveToXY authority and validation results` としてcommitする。

## 実装したAPI契約

```python
traci.vehicle.moveToXYImmediate(
    vehID,
    edgeID,
    laneIndex,
    x,
    y,
    angle=INVALID_DOUBLE_VALUE,
    keepRoute=1,
    matchThreshold=100,
    strictLaneHint=False,
)
```

- TraCI variable ID `0xf8`を再利用した。
- 旧実験API `setExternalState` とaliasは削除した。
- APIが扱うのはx/y/yawとlane mappingだけであり、speed/accelerationは扱わない。
- SUMO timeを進めず、remote-control queueの対象vehicleを即時適用してqueueから除去する。
- 同一時刻のremote-control latchだけを解除し、直後のPhase Bで通常のlane-change計算と車両移動を実行可能にする。
- `resetForExternalState()` と `MSAbstractLaneChangeModel` への追加変更は削除した。
- target lane、strategic intent、gap negotiation、shadow lane、completion、lateral speed、angle offset、`alreadyChanged`をclear・上書きしない。
- left-hand/off-centerのgeometry-side符号正規化と、lane終端外の縦超過を`posLat`へ混入させない投影修正はAPI専用処理として維持した。
- 通常 `moveToXY` は変更していない。

### standard mode

- upstream SUMO v1.23.1の `moveToXY` mapperをそのまま使用する。
- mapperが選択したlane/routeを時間進行なしで即時適用する。
- Phase A readbackで得たobserved laneを後続処理に採用する。

### strict mode

- supplied edge/laneが現在のSUMO primary laneと一致することを要求する。
- supplied current laneだけへ投影し、隣lane、predecessor edge、別internal laneを検索しない。
- routeを変更しない。
- invalid lane、primary lane不一致、match threshold超過は例外にしてfail closedする。

API自体の既定値は後方互換なstandard動作を提供するため `strictLaneHint=False` のままである。Odaiba serviceとcomposeの既定値は、下記standard-first試験結果に基づき `true` とした。

## Service Phase A

全入力を有限値検証した後、次の順序で呼び出す。

```text
moveToXYImmediate(...)
setSpeed(vehicle_id, -1)
setPreviousSpeed(vehicle_id, measured_speed, measured_acceleration)
```

pose mappingに失敗した場合はspeed timelineを変更しない。いずれかのAPI呼び出しまたはreadback検証に失敗した場合はPhase Bへ進まずfail closedする。

Phase A直後にSUMO time、x/y、yaw、speed、acceleration、route、observed laneをreadbackする。strict時だけrequested laneとの一致を必須にし、standard時はmapperが選択したobserved laneを採用する。Phase Bは0.05秒の `simulationStep()` を1回だけ実行する。

## 主な変更ファイル

- `apps/sumo_external_state/sumo-v1.23.1-set-external-state.patch`
- `Dockerfile.sumo-external-state`
- `docker-compose.ackermann-odaiba-feedback-gui.yml`
- `packages/terasim-service/terasim_service/plugins/cosim.py`
- `tests/test_integration/test_sumo_external_state.py`
- `tests/test_service/test_carla_ackermann_feedback.py`

## Standard-first判定

strategic lane-change parityはAV/BVとも合格した。blockerを20 cycles維持してから解除した試験で、intentを保持し、解除後に正規lane changeが成立した。

ただしjunction predecessor/internal境界では、AV/BVの両方で次を再現した。

- Phase B後のprimary lane: internal lane `:junction_0_0`
- 次Phase Aに連続したpredecessor側CARLA相当poseを入力
- standard modeは即時に `incoming_0` へremap
- x/yも約9.63 mm変化
- 通常 `moveToXY` comparatorも同じ `incoming_0` を選択
- strict modeは20 cyclesにわたり元のinternal laneを保持

これは計画に定めた「junction通過後にpredecessor/internal laneへ巻き戻る」に該当し、standard matchingだけではco-simulation契約を満たせないと判定した。そのためoptional strictを残し、service既定だけをtrueへ変更した。

## Build・小規模検証結果

| 検証 | 結果 |
|---|---|
| `git diff --check` | PASS |
| clean SUMO v1.23.1へのpatch apply check | PASS |
| 専用SUMO v1.23.1 build | PASS |
| TraCI/libsumo `moveToXYImmediate` self-check | PASS |
| service unit tests | 143 passed |
| Docker integration tests | 23 passed |
| strategic blocker 20 cycles + release、AV/BV | PASS |
| junction standard/strict、AV/BV | PASS（standardの既知remapを検出、strict保持） |
| yaw全象限、left-hand/off-center、stale speed timeline | PASS |
| `edge_426`、`edge_99`、`edge_0_0 -> edge_3_0` | PASS |

最終専用image:

```text
terasim-service:sumo-external-state-v1.23.1
sha256:b8f4226478fd1abc7fb3c3759de9840d43266b9b63d8d633d3bac964518e4b2e
```

## Odaiba guarded field run

### 実行条件

- `CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS='AV,*'` をliteral指定した。
- 起動ログで `feedback=apply actors=AV,*`、`actors=['*', 'AV']` を確認した。
- AckermannFeedback recordでAV/BV双方の選択と、frame-alignedなPhase A/Phase B/control traceを確認した。
- SUMO GUIはnoVNC 6093、既存chase cameraはCARLA noVNC 6092へ表示した。
- 既存 `carla-novnc-test`、`autoware-cosim`、`spectator-cam` は停止・再起動していない。

### 1回目

- container: `terasim-odaiba-immediate-movetoxy-final`
- simulation UUID: `effbbdbc-5c73-42a0-8409-3ebf70ae411f`
- artifacts: `outputs/odaiba_tlmappings_0708_ackermann/raw_data/0_0/effbbdbc-5c73-42a0-8409-3ebf70ae411f/`

### 診断確認を含む2回目

- container: `terasim-odaiba-immediate-movetoxy-final2`
- simulation UUID: `fd46bb5a-c1e8-4219-8072-c111abb0dee9`
- artifacts: `outputs/odaiba_tlmappings_0708_ackermann/raw_data/0_0/fd46bb5a-c1e8-4219-8072-c111abb0dee9/`

2回とも同一条件・同一箇所で停止した。2回目では診断目的でSUMO target laneも候補として評価したが、source laneとtarget laneの双方がthreshold外であることを確認した。この診断変更は最終実装から完全に除去している。

### 停止内容

```text
SUMO time: 524.70
actor: vehicle1167
current lane: :ia_300006_11_1
requested x/y: (89211.16294769244, 43222.64303213032)
requested angle: 191.149925 deg
match threshold: 8.0 m
reason: ackermann_feedback_external_state_current_lane_mapping_failed
```

wrapper containerのexit codeは0だが、runner statusは `error` であり、56 actorsへfail-closed brakeを適用して同期を終了した。field runとしてはFAILである。SUMO time 524.70、teleports 0で終了した。

### ログから確認した直接原因

`vehicle1167`についてSUMO lane-change判断自体は保持されていた。

- SUMO intent: `left`
- SUMO target lane: `:ia_300006_11_2`
- primary lane: `:ia_300006_11_1`
- lane position: 523.5秒から524.7秒に約3.57 mから6.997 mへ進行
- 524.7秒のCARLA yaw: 約102度
- 同時刻のPhase B target CARLA yaw: 約52.209度
- yaw差: 約49.76度
- source laneまでの距離: 約8.018 m
- SUMO target laneまでの距離: 約11.865 m

Phase B target yawは524.05秒付近で約114度から約59度へ大きく変化した一方、CARLA yawは約102度に残った。CARLA実位置はsource/target双方のcorridorから外れ、8 m thresholdを超えた。従って停止の直接原因はAPIがlane-change intentをresetしたことではなく、SUMO target yaw/lookaheadに対するCARLAの物理的なyaw・位置追従が成立しなかったことである。strict mappingはその状態を別laneへ隠して継続せず、意図どおりfail closedした。

## 合格状況

PASS:

- pose-only immediate APIとPhase A time不変
- speed/accelerationの既存APIへの分離
- lane-change model stateをresetしない実装
- blocker解除後のlane-change parity
- source patch apply、専用SUMO build、unit/integration小規模試験
- AV/BV選択とphase-aligned trace

FAIL / 未達:

- Odaiba field run完走
- sharp internal lane付近でPhase B target yaw/lookaheadへCARLAが物理追従すること
- SUMO判断にない追加lane transitionが全field runで0であることの完走確認

## 今回変更していないもの

- route terminal validation
- lane hysteresis
- SUMO lane policy / lane-change parameter
- lookahead方針
- CARLA executor制御則
- predicted collision guard
- curvature/lateral-error speed limiter
- full route-corridor latch
- 0.3 m/s restart assist
- legacy assimilationおよび通常 `moveToXY`

## 次に切り分けるべき項目

現在のAPI最小化とは別変更として、`edge_417_0 -> :ia_300006_11_1` 付近を小規模harnessで再現し、次を調べる必要がある。

1. Phase B target yawが約114度から約59度へ変化するlane/link/lookahead選択。
2. target yaw変化量に対するAckermann steering angle/rateとCARLA物理応答の実現可能性。
3. SUMO target lane `:ia_300006_11_2` とprimary lane `:ia_300006_11_1` の間で生成される連続軌道。

これは本計画で明示的に対象外としたlookahead/executor領域であるため、本commit列では修正しない。
