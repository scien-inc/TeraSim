# Odaiba restart assist and external-state lane assignment handoff

日付: 2026-08-12 JST

## 1. 結論

停止付近でSUMOが正の速度・加速度を要求してもCARLA車両が発進できない問題に対する
restart assistは、単体・周辺回帰テストと実CARLA 1車両80-frame smokeを通過した。
この変更は次のcheckpointとしてcommit済みである。

- commit: `e2728e3804332045286e9b6bacd2c52dc3d7062d`
- subject: `fix: restart Ackermann feedback actors from standstill`

一方、通常Odaiba field runで観測したAVの不要な複数lane遷移とroute dead-end停止は未解決である。
SUMO-onlyでは再現せず、Phase A external-state assimilationがCARLAの横ずれを隣接laneへの
primary-lane変更として解釈することが主因である。junctionでは現行`setExternalState`へ渡した
lane hint自体が保持されない例も確認したため、service側のcurrent-lane-only射影だけでは不十分で、
専用SUMO APIにstrict lane hintを追加するのが次の最小実装である。

小規模決定論テストを通すまでは、次のOdaiba統合走行へ進まないこと。

## 2. Repository state

- repository: `/home/h-kawai/TeraSim-ackermann-feedback-gui`
- branch: `kawai/experiment/sumo-two-phase-movetoxy-20260810`
- code checkpoint HEAD: `e2728e3804332045286e9b6bacd2c52dc3d7062d`
- original baseline: `c2aac41dca0673389cd2b01ff97cae6ce27c3e4f`

主要checkpointは次の順である。

- `c2aac41` `fix: apply direct emergency braking in CARLA co-sim`
- `f6d1a45` `feat: add SUMO external-state assimilation`
- `5041dae` `fix: preserve external-state lateral position`
- `ed9d949` `fix: stabilize external state at lane boundaries`
- `3c87d61` `fix: use route lookahead for external-state feedback`
- `e2728e3` `fix: restart Ackermann feedback actors from standstill`

`git reset --hard`を使わないこと。既存レポートとユーザー変更を一括破棄しないこと。

## 3. Restart assist checkpoint

### Problem

CARLA速度がほぼ0のとき、Phase Aでその速度をSUMOへ戻すため、Phase Bが毎tick生成する
次速度も約`0.09 m/s`に留まる場合があった。CARLA Ackermann controllerはこの小さい目標を
発進に必要な駆動へ変換できず、SUMOは正の加速度を要求しているのに車両が停止し続けた。

### Implementation

変更対象は次の5ファイルだけである。

- `docker-compose.ackermann-odaiba-feedback-gui.yml`
- `docs/carla_ackermann_feedback.md`
- `packages/terasim-service/terasim_service/utils/carla/ackermann_control.py`
- `packages/terasim-service/terasim_service/utils/carla/cosim.py`
- `tests/test_service/test_carla_ackermann_feedback.py`

CARLAがstandstill範囲にあり、SUMOの次速度と加速度がともに正のときだけ、SUMO要求加速度を
`0.05 s`ずつ積分して小さなrestart speed targetを作る。既定上限は`0.3 m/s`である。

- enter speed: `0.05 m/s`
- release speed: `0.2 m/s`
- speed epsilon: `0.001 m/s`
- maximum restart target: `0.3 m/s`

SUMOが停止または減速を要求した場合は即時解除する。CARLA速度がrelease speedへ達した場合も
通常制御へ戻る。`AV`だけでなく`*`で選択されたBVにも同じ処理を適用する。
`AckermannControlTrace`へ`restart_active`と`restart_target_speed`を追加した。

### Tests

ホストには`uv`と`pytest`がなかったため、破棄される専用imageコンテナへ
`pytest==8.3.5`を一時導入し、現在のsource/testをread-only mountして実行した。

```bash
python3 -m pytest -o addopts= -p no:cacheprovider -q \
  /app/tests/test_service/test_carla_ackermann_feedback.py
```

結果:

- `92 passed, 5 warnings in 0.50s`
- `python3 -m py_compile`成功
- `git diff --check`成功

追加テストはAV/BV双方について、target accumulation、release speedまでの保持、SUMOの
stop/decelerationによる解除、trace出力を確認する。

### Real CARLA one-vehicle smoke

- image: `terasim-service:sumo-external-state-restart-assist-20260811`
- image ID: `sha256:5563e12c824d9bd7f9f5a88054e36086d714deb8088bb6e0f71c14ea241b6505`
- SUMO label: `SUMO-v1.23.1-external-state-4`
- CARLA: 0.9.16, `odaiba_tl_mapping`
- feedback actors: `AV`
- requested frames: `80`
- run UUID: `c231e185-1496-46e3-ae9a-35bfc38287af`

結果:

- 80 framesをclean exit
- control trace: `76`
- restart active: `10` frames
- 最初のrestart: SUMO time `1.00 s`, CARLA speed `0.030045 m/s`, target `0.122045 m/s`
- 最後のrestart: SUMO time `1.45 s`, CARLA speed `0.005318 m/s`, target `0.3 m/s`
- CARLA最低速度: `0.005318 m/s`
- CARLA最終速度: `5.106593 m/s`
- 最終restart state: inactive

artifact:

- `/tmp/terasim-odaiba-external-state-smoke/restart_assist_80_console.log`
- `/tmp/terasim-odaiba-external-state-smoke/output/external_state_odaiba_smoke/raw_data/one_vehicle/c231e185-1496-46e3-ae9a-35bfc38287af/`

## 4. Latest standard Odaiba field run

### Conditions and artifacts

- run UUID: `f4d8a2f1-b400-459b-a493-6482ffcf5886`
- standard traffic cache start: SUMO time `500 s`
- final SUMO time: `749.55 s`
- CARLA actor radius: `300 m`
- physics radius: AV中心`100 m`, hysteresis `10 m`
- assimilation: `external_state`
- step: `0.05 s`
- immediate validation: enabled, position tolerance `0.001 m`
- stopped service container: `terasim-odaiba-physics100-live`, exit `143`（ユーザー指示で停止）

主要artifact:

- `/tmp/terasim-odaiba-standard-restart-assist/odaiba_tlmappings_0708_ackermann/raw_data/0_0/f4d8a2f1-b400-459b-a493-6482ffcf5886/terasim_cosim_plugin.log`
- 同じdirectoryの`.log.1`, `.log.2`, `run.log`, `collision.xml`, `fcd_all.xml`
- `/tmp/terasim-odaiba-standard-restart-assist/carla_collision_events.jsonl`
- `/tmp/feedback_lane_transition_analysis.json`

注意: output directory名には`restart-assist`が入っているが、停止済みコンテナ内sourceを
`docker cp`で確認するとroute-lookahead修正は含む一方、restart assist codeは含んでいなかった。
したがって、この長時間runをrestart assistの実走証明として使わないこと。restart assistの
実CARLA証拠は前節の80-frame runである。

また停止済みコンテナの設定は`CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS=*`であり、起動ログも
`actors=['*']`だった。現行serviceの`*`はBVだけを意味し、AVは明示的な`AV`指定が必要である。
plugin logにはAVの`feedback_external_state*` eventも存在しており、このrunでは設定表示と
command経路に不整合が残る。次回は必ずliteral `AV,*`を設定し、起動ログが
`actors=['*', 'AV']`相当であることと、AV/BV双方の`AckermannFeedback` recordを確認する。

### AV behavior

AV route:

```text
edge_1394 -> edge_459 -> edge_426 -> edge_432 -> edge_427 -> edge_0 -> edge_3 -> ...
```

coupled runでのprimary lane:

```text
570.40  edge_426_0
571.30  edge_426_1
572.10  edge_426_2
583.95  edge_432_3
615.75  edge_427_3
640.05  edge_0_1
```

AVは`edge_0_1`の終端`lanePos=402.5 m`付近でspeed 0になり、その後`edge_3`へ進めなかった。
network接続は次のとおりであり、現在laneからrouteを継続できないことが停止理由である。

- `edge_0_0 -> edge_3_0`
- `edge_0_1 -> edge_2_0`
- `edge_0_2 -> edge_2_1`

停止中もCARLA/SUMOの位置誤差は約`0.007 m`、yaw誤差は約`0.499 deg`で、Phase A validation
failureはなかった。つまりx/y/yaw/speed feedback自体は一致していたが、SUMO primary laneが
routeと異なるlaneへ変わっていた。

### Source of unwanted lane changes

lane transition logには次の順序がある。

- `edge_426_0 -> edge_426_1`: `source=feedback_external_state_lane_change`
- `edge_426_1 -> edge_426_2`: `source=feedback_external_state_lane_change`
- `edge_426_2 -> edge_426_1`: `source=sumo_step`
- `edge_426_1 -> edge_426_2`: `source=feedback_external_state_lane_change`

Phase Aがlane 0から1、1から2へ変更し、Phase BのSUMOが一度戻そうとしている。したがって、
SUMOのstrategic lane changeがAVを不要に2 lane移したという説明とは一致しない。

## 5. SUMO-only comparison

同じSUMO 1.23.1、network、route、background demand、step `0.05 s`でCARLA/TeraSim制御を外し、
AVをSUMO time `500.05 s`に同じroute、lane 0、position 5 m、speed 5 m/sで挿入した。

設定:

- laneChangeMode: `1621`
- speedMode: `31`
- type: `NDE_URBAN`
- `lcKeepRight=1`
- `lcStrategic=1`
- `lcCooperative=0`
- `lcSpeedGain=0`
- `lateral-resolution=0.5`
- `maxSpeedLat=1.0`

lane sequence:

```text
edge_1394_0 -> internal lane 1 -> edge_459_1 -> internal -> edge_426_0
-> edge_432_1 -> edge_427_1 -> internal -> edge_0_0 -> edge_3_0
```

結果:

- `edge_426`全区間をlane 0のまま通過
- `edge_3`へ約`653.0 s`で到達
- AVの`lanechanges.xml` entry: 0
- same-edge lane change: 0
- 1 tick最大変位: `0.5003 m`
- 最大絶対lateral position: `0.232 m`
- 最大絶対lateral speed: `0.45 m/s`
- multi-meter boundary warpなし

artifact:

- `/tmp/sumo-only-current-settings/metadata.json`
- `/tmp/sumo-only-current-settings/trace.csv`
- `/tmp/sumo-only-current-settings/transitions.json`
- `/tmp/sumo-only-current-settings/lanechanges.xml`
- `/tmp/sumo-only-current-settings/fcd.xml`

現在のSUMO lane-change設定だけではcoupled runのAV異常は再現しない。`lcKeepRight=0`などへ
設定だけを変更することは、この問題の最小修正ではない。

## 6. BV impact

この問題はAV専用ではない。最新plugin logのtransition event 200件を分類した。

- `sumo_step`: 147
- `feedback_external_state`: 8
- `feedback_external_state_lane_change`: 45
- external-state lane transitionを持つBV: 22台
- same-road external lane reassignmentを持つBV: 8台
- Phase A/Bのrapid reversalを持つBV: 4台
  - `vehicle2186`
  - `vehicle2170`
  - `vehicle167`
  - `vehicle2311`
- external -> SUMO reverse -> external sandwich: BVで9件
  - `vehicle2186`: 6件
  - `vehicle2170`: 3件

`vehicle2170`と`vehicle2186`はSUMO time `644.95 s`にjunction
`:ia_300001_9_0`で衝突した。lane boundary bouncingがrisk amplifierである可能性はあるが、
この衝突の単独原因とまでは確定していない。

BVでAVと完全に同じ「2 lane横断後にroute dead-end停止」までは確認していない。AV routeは
`edge_0_0`だけが`edge_3`へ接続するため、primary-lane誤選択が直ちに致命的になった。

## 7. Root cause in current code

`packages/terasim-service/terasim_service/plugins/cosim.py`では、SUMOがexportした
`lateral_speed`または`lateral_offset`だけで`lane_change_active`を立てる。

- current lines 2580-2589
- speed threshold: `0.05 m/s`
- offset threshold: `0.15 m`

command適用時、active actorは`_move_ackermann_feedback_actor_exact()`へ入り、current edgeの
全laneを候補としてCARLA x/yに最も近いlaneを選ぶ。

- current lines 1271以降: current-edge all-lane projection
- current lines 2991-3027: `lane_change_active`分岐

このため次のpositive feedback loopが成立する。

```text
CARLAの物理横ずれ・tracking lag
  -> lane_change_active
  -> Phase Aが隣接laneをprimary laneに採用
  -> live SUMO laneからlookaheadを再構築
  -> CARLAが誤ったlane方向へ追従
  -> さらに次のlaneへ再assignment
```

lane hysteresisを0にすることはこのloopを解消しない。逆にlane選択を容易にする可能性がある。

さらにjunctionではservice側がinternal laneをhintとして選んでも、現行専用SUMO APIが
`moveToXY()`の再matchingによりpredecessor laneへ戻す例を確認した。

- `vehicle2186`: requested `:ia_300001_13_2`, observed `edge_124_3`
- `vehicle2170`: requested `:ia_300001_13_2`, observed `edge_124_3`

現行Phase A validationはtime、x/y、angle、speedだけを検証し、observed laneがrequested
laneと一致するかは検証していない（current lines 1056-1075）。そのためこの誤assignmentを
成功として扱ってしまう。

## 8. Next minimal implementation

### A. Service: external_stateではcurrent SUMO laneだけをhintにする

`external_state`経路では`lane_change_active`によるcurrent-edge all-lane branchへ入れず、
Phase B直後にSUMOが持つcurrent laneだけへCARLA x/yを射影する。

概念上は次の順序にする。

```python
if use_external_state:
    projection = project_to_current_sumo_lane_only(...)
    set_external_state_strict(...)
elif lane_change_active:
    # legacy moveTo/moveToXY behavior remains unchanged
    ...
```

SUMOがPhase B中に正規のlane changeまたはroute transitionを行った場合、次Phase Aはその新しい
current laneを使う。CARLAの横ずれだけではprimary laneを変更しない。

### B. Dedicated SUMO API: optional strict lane hint

`apps/sumo_external_state/sumo-v1.23.1-set-external-state.patch`の`setExternalState`へ、後方互換な
optional `strictLaneHint` booleanを追加する。現行引数は9個なので10番目とする案が最小である。

strict modeの要件:

- exact CARLA x/y/yaw/speed/accelerationを即時反映する
- SUMO timeを進めない
- supplied edge/laneをprimary laneとして固定する
- lane positionとlateral positionはそのhint lane基準で計算する
- adjacent lane、predecessor edge、別internal laneへ再matchしない
- hint laneが存在しない、route上で無効、またはmatch threshold外なら例外にしてfail closed
- 既存remote-control latch解除、stale speed latch解除、lane-change state clearを維持する
- strict=falseの現行挙動と通常`moveToXY`を変更しない

変更箇所候補:

- `src/libsumo/Vehicle.cpp/.h`
- `src/traci-server/TraCIServerAPI_Vehicle.cpp`
- `tools/traci/_vehicle.py`
- 必要に応じて`src/libsumo/Helper.cpp/.h`および`MSVehicle::Influencer`
- repository patch: `apps/sumo_external_state/sumo-v1.23.1-set-external-state.patch`

serviceのimmediate validationへobserved lane equalityを追加する。strict request後のlaneが違えば
`ackermann_feedback_external_state_validation_failed`としてfail closedにする。

### C. Small tests before Odaiba

1. Junction predecessor/internal test
   - Phase Bでpredecessorからinternal laneへ入れる
   - 次Phase Aのx/yを境界のpredecessor側へ少し置く
   - x/y/yaw/speedはexact一致、SUMO time deltaは0
   - primary laneはinternal laneのまま
   - 20 cyclesでinternal/predecessor bounce 0

2. `edge_426` current-lane test
   - primary laneは`edge_426_0`
   - CARLA x/yをlane 1側へoffsetする
   - Phase A後もprimary laneは`edge_426_0`
   - route lookaheadもlane 0 corridorを維持
   - `0 -> 1 -> 2` external reassignment 0

3. `edge_0 -> edge_3` route test
   - `edge_0_0`を維持
   - Phase Bで`edge_3_0`へ到達できる
   - stop at `edge_0_1` endを再現しない

AVとBVの両方をparameterizeする。まずsource patch apply check、専用image build、既存integration
suite、上記1車両testを通す。その後にのみ、literal `AV,*`、AV+100 m内BV物理化条件へ戻る。

## 9. Explicitly deferred work

ユーザー指示により、次のものは今回の最小実装へ混ぜない。

- predicted collision guard
- curvature-based speed limiter
- lateral-error-based speed limiter
- full route-corridor latch / `getBestLanes` target latch
- smooth lane-change path generationの再設計
- 20 Hz処理時間最適化
- `lcKeepRight`, `lcStrategic`, `lateral-resolution`の一括変更

前走車が急カーブで壁へ衝突する問題には、route-aware lookahead commit `3c87d61`をまず維持する。
strict lane assignment後も同じcurve tracking failureが再現する場合にだけ、steering lookaheadや
曲率速度制限を別の論理変更として検討する。

## 10. Current process state

レポート作成時点:

- `terasim-odaiba-physics100-live`: stopped, exit 143
- `carla-novnc-test`: running, image `carla-novnc:0.9.16-odaiba-recovered-20260811`
- `autoware-cosim`: running
- `spectator-cam`: running

ユーザーの新しい明示指示なしに既存CARLA、Autoware、spectatorを停止・再起動しないこと。
field runを始める前にcontainer/image/configを再確認し、専用buildを含まない古いtagを誤って
使わないこと。

## 11. First actions in the next session

1. このレポートと次の3レポートを全文読む。
   - `reports/20260810_ackermann_collision_fix_handoff.md`
   - `reports/20260810_sumo_two_phase_movetoxy_experiment_handoff.md`
   - `reports/20260811_sumo_external_state_odaiba_lateral_fix_handoff.md`
2. branch、HEAD、`git status`、`git diff`を確認する。
3. `e2728e3`と本レポートcommit以外の新しい差分があれば、内容を確認して保持する。
4. current-lane-only service testを先に追加する。
5. strict lane hint APIを専用SUMO patchへ実装する。
6. dedicated imageを新しい明示tag/labelでbuildする。
7. junction、`edge_426`、`edge_0 -> edge_3`のsmall gateを通す。
8. gate通過前はOdaiba field runを行わない。

