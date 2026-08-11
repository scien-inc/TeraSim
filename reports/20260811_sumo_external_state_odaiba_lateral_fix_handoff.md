# SUMO external_state Odaiba lateral fix handoff

日付: 2026-08-11 JST

## 1. Repository

- branch: `kawai/experiment/sumo-two-phase-movetoxy-20260810`
- starting HEAD: `f6d1a456e2bd208627fc6814ae32a497f6b976d4`
- lateral fix / 30-frame checkpoint commit: `5041dae309f72bd11dc60fc30614779c978f236a`
- 通常の`moveToXY`とlegacy assimilation経路は変更していない

## 2. 前回Odaiba 30-frame failure

前回run `caa67ba8-989d-44f6-95c7-8c56fb2eeeae`では、低速時に次を観測した。

- Phase B変位最大: `0.120108 m`
- 次Phase A補正最大: `0.122854 m`
- raw probeでlane centerを挟む`0.132762 m`の鏡像移動
- Phase A内部lane lateral: `+0.066330 m`
- Phase B後のsigned lateral: `-0.066330 m`

Odaiba netは`lefthand="true"`である。SUMO 1.23.1の`moveToXY`が算出する
幾何学的なlane lateral符号と、remote latch解除後の通常`MSVehicle`位置再構成が使う
left-hand内部符号が一致していなかった。Phase Aのexact x/y cacheがこの不整合を隠し、
Phase Bでcacheが通常位置へ戻ると鏡像側へ移動していた。

## 3. 実装

### SUMO専用API

`apps/sumo_external_state/sumo-v1.23.1-set-external-state.patch`の専用
`completeRemoteControl`に`MSVehicle*`を渡すよう変更した。

remote state適用後、latch解除前に次を実行する。

1. 現在のlane lateral符号と反転符号から、通常lane geometry位置をそれぞれ計算する。
2. external raw x/yに近い候補を内部lane lateral stateとして保持する。
3. exact Phase A x/y cacheは維持したままremote latchを解除する。

この処理は専用`setExternalState`から呼ぶ`Helper::applyRemoteControl`内だけで実行される。
通常のqueued `moveToXY`処理順と動作は変更していない。

### Service lane-relative export

SUMOの`VAR_LANEPOSITION_LAT`はnetのdrive sideによって内部符号が変わる一方、lane shape
単体にはdrive-side metadataがない。このためlane geometry復元時にraw SUMO x/yを参照し、
横オフセットの2候補からraw位置に近い側を選ぶようにした。declared lane lengthとshape
lengthのnormalized progress修正はそのまま保持している。

### Self-contained image

`Dockerfile.sumo-external-state`を`external-state-3`へ更新し、対応する
`terasim_service` sourceも最終imageへcopyするようにした。working-tree package mountなしで
SUMO側とservice側の両修正を利用できる。

## 4. Build

- SUMO tag: `v1_23_1`
- upstream commit: `676720d13f6f42d8c79d156e9d67001f8c22f6f6`
- image: `terasim-service:sumo-external-state-v1.23.1`
- image label: `SUMO-v1.23.1-external-state-3`
- final image digest: `sha256:6eed5c82f26b0af8128852411a6173b155008557322e18f2b26537715c97cc2c`
- CARLA image: `carla-novnc:0.9.16-odaiba-tl`
- CARLA image digest: `sha256:59a372bbada3b9fff50a85a929859e28d6cd7d540efe9158614857d4c07fb47c`

clean SUMO checkoutへの`git apply --check`は成功した。C++ build、patched TraCI、libsumoの
`setExternalState`存在確認も成功した。

## 5. Small gates

### Tests

- service unit suite: `80 passed`
- SUMO integration gate: `4 passed`
- self-contained final image combined gate: `84 passed, 5 warnings`

追加したleft-hand off-center testは1車両をlane centerから`0.4 m`ずらし、6周期にわたり
次を確認する。

- Phase Aは時間を進めずx/yを即時一致させる
- left-hand内部lane lateral符号が通常位置再構成と一致する
- Phase B後も同じ横側に残る
- 次Phase Aで鏡像側から巻き戻らない

### Odaiba raw TraCI probe

前回と同じlane `edge_1394_0`、x/y/yaw/speedを新buildへ直接適用した。

- requested x/y: `(89437.79033579212, 43195.52763432823)`
- requested yaw/speed: `207.308013916 deg`, `0.011940050 m/s`
- Phase A x/y error: `0 m`
- Phase A time delta: `0 s`
- normalized internal lane lateral: `-0.066330369 m`
- Phase B time delta: `0.05 s`
- Phase B displacement: `0.005766927 m`
- Phase B speed: `0.103940050 m/s`

前回の`0.132762 m`鏡像変位は再現しなかった。

## 6. Final Odaiba guarded smoke

### Conditions

- run UUID: `c58be682-f697-4bdc-9c1a-5ec4b5f30604`
- final self-contained imageを使用し、service package mountなし
- CARLA 0.9.16 / `odaiba_tl_mapping` / isolated offscreen container
- AV: 1、BV: 0
- requested frames: 30
- SUMO/CARLA step: `0.05 s`
- feedback: `apply`, actor: `AV`
- assimilation: `external_state`
- immediate validation: enabled、position tolerance `0.001 m`
- feedback failure: fail closed

### Results

- container exit code: `0`
- SUMO steps: `30`
- completed SUMO time: `0.10`から`1.55 s`
- 全隣接SUMO time delta: `0.05 s`
- inserted/running vehicles: `1/1`
- feedback records: `30`
- spawn-transform-pending rejects: `4`
- queued external states: `26`
- queued CARLA frames: `30450`から`30475`、全て連続
- control traces: `26`
- Phase A validation failure: `0`
- ERROR / CRITICAL / Traceback: `0`
- accepted CARLA speed: `0.005873`から`4.958224 m/s`
- speed隣接差最大: `1.119184 m/s`
- low speedと`>10 m/s`の交互振動: `0`
- accepted yaw: `208.802300`から`210.013359 deg`
- yaw隣接差最大: `0.343811 deg`
- yaw 104/161度帯hit: `0`
- Phase B変位: `0.013424`から`0.252244 m`
- CARLA speed `<0.1 m/s`のPhase B変位最大: `0.013890 m`
- 次Phase A補正: `0.006920`から`0.058540 m`
- CARLA speed `<0.1 m/s`の次Phase A補正最大: `0.021197 m`

高速度域のPhase B最大`0.252244 m`は約`4.96 m/s * 0.05 s`と整合する。
低速域はTown01 smokeの次Phase A補正最大`0.02006 m`と同程度であり、前回Odaibaの
約`0.12 m`鏡像往復は解消した。30-frame guarded smokeはPASSと判定する。

## 7. Cleanup and artifacts

終了後の隔離CARLAは次を確認した。

- map: `odaiba_tl_mapping`
- synchronous mode: `False`
- fixed delta: `None`
- vehicle actors: `0`
- sensor actors: `0`

隔離CARLA、service container、専用networkは停止・削除した。既存の
`carla-novnc-test`、`autoware-cosim`、`spectator-cam`は変更していない。

最終run artifacts:

- `/tmp/terasim-odaiba-external-state-smoke/carla_initialization_odaiba_30_v3_final.jsonl`
- `/tmp/terasim-odaiba-external-state-smoke/output/external_state_odaiba_smoke/raw_data/one_vehicle/c58be682-f697-4bdc-9c1a-5ec4b5f30604/terasim_cosim_plugin.log`
- `/tmp/terasim-odaiba-external-state-smoke/output/external_state_odaiba_smoke/raw_data/one_vehicle/c58be682-f697-4bdc-9c1a-5ec4b5f30604/run.log`

## 8. Odaiba 60-frame extension

### Commit

実装、対応テスト、専用image定義、30-frame結果を次のcheckpointとしてcommitした。

- commit: `5041dae309f72bd11dc60fc30614779c978f236a`
- subject: `fix: preserve external-state lateral position`

### Conditions

- run UUID: `48f7a3a1-d802-4f91-886f-e7b47a868502`
- 30-frame runと同じself-contained imageを使用し、service package mountなし
- AV: 1、BV: 0（`num_cars: 1`、`max_controlled_bv: 0`）
- requested frames: 60
- SUMO/CARLA step: `0.05 s`
- feedback: `apply`, actor: `AV`
- assimilation: `external_state`
- immediate validation: enabled、position tolerance `0.001 m`
- feedback failure: fail closed

### Results

- container exit code: `0`
- SUMO steps: `60`
- completed SUMO time: `0.10`から`3.05 s`
- 全隣接SUMO time delta: `0.05 s`
- feedback records: `60`
- spawn-transform-pending rejects: `4`
- queued external states: `56`
- queued CARLA frames: `46366`から`46421`、全て連続
- control traces: `56`
- Phase A validation failure: `0`
- ERROR / CRITICAL / Traceback: `0`
- accepted CARLA speed: `0.005873`から`4.958224 m/s`
- speed隣接差最大: `1.119184 m/s`
- low speedと`>10 m/s`の交互振動: `0`
- accepted yaw: `208.802300`から`211.169746 deg`
- yaw隣接差最大: `0.343811 deg`
- yaw 104/161度帯hit: `0`
- Phase B pair数: `55`
- Phase B変位: `0.013424`から`0.252244 m`
- CARLA speed `<0.1 m/s`のPhase B変位最大: `0.013890 m`
- 次Phase A pair数: `56`
- 次Phase A補正: `0.002419`から`0.058540 m`
- CARLA speed `<0.1 m/s`の次Phase A補正最大: `0.021197 m`

30-frame後半を含む追加30フレームでも、速度/yawの交互振動と約`0.12 m`の低速鏡像往復は
再現しなかった。AV 1台・BV 0台の60-frame guarded smokeはPASSと判定する。

### Cleanup and artifacts

終了後、隔離CARLAがasync、fixed deltaなし、vehicle/sensor actorとも0であることを確認した。
60-frame専用CARLA/service containerとnetworkだけを削除し、既存の`carla-novnc-test`、
`autoware-cosim`、`spectator-cam`は変更していない。

- `/tmp/terasim-odaiba-external-state-smoke/carla_initialization_odaiba_60_v3.jsonl`
- `/tmp/terasim-odaiba-external-state-smoke/output/external_state_odaiba_smoke/raw_data/one_vehicle/48f7a3a1-d802-4f91-886f-e7b47a868502/terasim_cosim_plugin.log`
- `/tmp/terasim-odaiba-external-state-smoke/output/external_state_odaiba_smoke/raw_data/one_vehicle/48f7a3a1-d802-4f91-886f-e7b47a868502/run.log`

## 9. Next step

次は同じfail-closed条件でBVを少数（まず1台）だけ追加する。AVのexternal state即時一致、
Phase B後の状態取得、次Phase Aの低速補正、速度/yaw振動を同じ指標で確認してから、
通常規模・長時間Odaibaへ進む。

## 10. Existing CARLA / 100 m physics-radius field run

### Conditions

- CARLA: 既存`carla-novnc-test`、0.9.16、`odaiba_tl_mapping`
- TeraSim/SUMO image: `terasim-service:sumo-external-state-v1.23.1`
- standard Odaiba traffic cache at SUMO time `500 s`
- CARLA actor radius: `300 m`、hysteresis: `20 m`
- physics radius: AV中心`100 m`、exit hysteresis: `110 m`
- feedback: `apply`、物理対象AV/BVすべて
- assimilation: `external_state`、immediate validation enabled
- feedback failure: fail closed
- gate run: `60 frames`
- extended run: `700 frames`
- extended run UUID: `975da4e7-8fb4-4584-b585-7f161c65c554`

### Successful portions

- service exit code: `0`
- SUMO steps: `700`、completed time `500.10`から`535.05 s`
- 全隣接SUMO time delta: `0.05 s`
- physics vehicles: `2`から`9`へ増加
- 物理化actor: AVとBV 8台
- 全9台で`physics_enabled`と`stable`を確認
- BVの物理化開始時AV距離: `97.443`から`98.913 m`
- 物理BVの観測距離最大: `98.913 m`
- CARLA vehicle actors: `105`から`131`、それ以外はteleport mode
- external-state feedback commands: `2772`
- immediate validation failure: `0`
- 全物理actorのcontrol traceで`geometry_from_physics=true`
- 全物理actorのlow speedと`>10 m/s`の隣接振動: `0`
- AV yaw隣接差最大: `0.594345 deg`
- 低速時の次Phase A補正最大: `0.060994 m`

700-frame後半100フレームの処理時間はmean `72.222 ms`、p95 `83.118 ms`、
realtime factor `0.692`であり、`0.05 s` wall-clock deadlineは満たさなかった。

### Blocking findings

`vehicle1510`でSUMO time `532.45 s`に単発の大きなlane-side jumpを観測した。

- 前tick: `edge_426_1`, lateral `-1.577141 m`
- 異常tick: `edge_426_0`, lateral `+1.573150 m`
- Phase B displacement: `6.778729 m`
- 次Phase A correction: `6.795896 m`
- 次tickには`edge_426_1`へ戻った

CARLA raw位置は連続していたため、物理actorのteleportではなく、SUMO Phase Bのlane/lateral
再構成がleft-hand lane境界で反対側を一時選択した現象である。lane center射影では、raw位置は
`edge_426_1`から`1.627685 m`、異常SUMO位置は`edge_426_0`から`1.573150 m`で、両者は
lane群の反対側にある。

さらにSUMO collision logで`532.60 s`に物理BV同士の衝突を記録した。

- collider: `vehicle1510`, speed `10.52 m/s`
- victim: `vehicle2286`, speed `5.54 m/s`
- AVからの距離: `84.823 m`
- 同時刻のCARLA raw front位置間距離: `2.013 m`
- emergency brakeは`vehicle1510`で次tick `532.65 s`からactive

遠方teleport actor `vehicle1357`と`vehicle1986`にはstatic geometryによるspawn errorも
各1回あった。これは約`270 m`地点で100 m物理対象外である。ほかにAVから約3 km離れた
SUMO-only collisionが2件あった。

したがって「既存CARLAで標準Odaibaを起動し、AVと100 m内BVを物理化する」こと自体は
確認できたが、lane境界での`6.8 m`往復と物理BV衝突のため、このfield runは受入FAILとする。
この原因を修正するまで通常運用・長時間走行へは進めない。

### Cleanup and artifacts

終了後の`carla-novnc-test`はasync、fixed deltaなし、vehicle/sensor/walker actorとも0。
既存CARLA、Autoware、spectatorは停止せず、test service containerだけ削除した。

- `/tmp/terasim-odaiba-physics100-site/site_run_output/carla_profile_v3_700.jsonl`
- `/tmp/terasim-odaiba-physics100-site/site_run_output/terasim_profile_v3_700.jsonl`
- `/tmp/terasim-odaiba-physics100-site/site_run_output/carla_initialization_v3_700.jsonl`
- `/tmp/terasim-odaiba-physics100-site/odaiba_tlmappings_0708_ackermann/raw_data/0_0/975da4e7-8fb4-4584-b585-7f161c65c554/terasim_cosim_plugin.log`
- `/tmp/terasim-odaiba-physics100-site/odaiba_tlmappings_0708_ackermann/raw_data/0_0/975da4e7-8fb4-4584-b585-7f161c65c554/collision.xml`

## 11. edge_426 lane-boundary root cause and SUMO source fix

### Root cause

標準Odaiba networkは`lefthand=true`だが、`edge_426`のlane shapeは空間上で
`lane 0 -> lane 2`が通常のleft-hand lane-index方向と逆に並ぶ。従来の専用
`setExternalState` completionは、外部x/yへ最も近い幾何学的`posLat`をそのまま
lane-change modelへ渡していた。このため次が同じ値に混在していた。

- lane-change modelが使うlane-index相対の横方向
- lane shapeへx/yを再構成するときの幾何学的な横方向

さらにPhase A前のlane-change要求が残ると、Phase Bでlane-change modelが幾何学側の
`posLat`を反対向きのmaneuverとして解釈し、`edge_426_1 -> edge_426_0`の1-tick
誤遷移と約`6.8 m`の反対側再構成が発生した。単純なleft-hand全体の符号反転では、
通常lane orderingのleft-hand networkを壊すため不十分である。

### Source change

専用SUMO patchを次の構成へ変更した。

- 隣接lane centerの実geometryを調べ、lane index増加方向の物理的な符号を推定
- lane-change model用の内部`posLat`をlane-index semanticsへ正規化
- x/y、angle、back position再構成用のgeometry projection signを車両ごとに保持
- `setExternalState`直後に古いlane-change maneuver stateをclear
- assimilation済み`posLat`自体はclearせず維持
- 通常の`moveToXY`、legacy assimilation、TraCI API signatureは変更なし

対象は`MSVehicle::Influencer`、`MSVehicle`のposition/angle projection、
`MSAbstractLaneChangeModel`のexternal-state resetである。

### Deterministic one-vehicle regression

`tests/test_integration/test_sumo_external_state.py`へ実Odaiba networkの
`edge_426`を使う80-cycle testを追加した。

- 1車両をlane 1からlane 2へ連続的に横移動
- 物理位置が中央を越えた時点だけlane hintを1から2へ変更
- 各Phase A後に古い反対向きlane-0要求を意図的に再投入
- `executeMove`後も時刻が進まないことを確認
- Phase B変位、次Phase A補正をともに`1.2 m`未満に制限
- Phase B laneを`edge_426_1`または`edge_426_2`だけに制限

旧imageでは約`6.83 m`でFAILし、最終sourceではPASSした。通常lane orderingの
left-hand off-center testも同時に保持し、両方を通過させた。

## 12. external-state-4 build and small-test gate

### Build

- upstream SUMO tag: `v1_23_1`
- upstream commit: `676720d13f6f42d8c79d156e9d67001f8c22f6f6`
- image: `terasim-service:sumo-external-state-v1.23.1`
- image label: `SUMO-v1.23.1-external-state-4`
- image manifest list / ID:
  `sha256:c9bdea2f5327d1499548a9e054cdd52e7314a604c900ceb4ba79f87e6b43e9fe`

patchの一部をzero-context hunkにしたため、Dockerfileは
`git apply --unidiff-zero --check`後に同じoptionで適用する。cleanな公式sourceへの
check/apply、試験済みsourceとの主要4 fileのbyte一致、Docker内の再buildを確認した。

### Test result

再buildしたimage内で次を実行した。

```bash
python3 -m pytest -o addopts= -q \
  tests/test_integration/test_sumo_external_state.py -vv
```

結果は`5 passed, 7 warnings in 6.41s`。

- Phase A即時assimilation / 0.05秒single step
- stale TraCI speed latch解除
- 通常left-hand off-center lateral保持
- Odaiba `edge_426` lane-boundary 80-cycle
- TeraSim priority順序（Phase A -> planning -> Phase B）

## 13. Odaiba 100 m physics-radius rerun without collision guard

ユーザー指示により予測衝突guardは追加していない。既存の直接緊急ブレーキだけを維持し、
SUMO lane-boundary修正単独の効果を確認した。

### Conditions

- run UUID: `9cdb726a-383b-40fc-a26e-0a63a5a2d70c`
- CARLA: 既存`carla-novnc-test`、0.9.16、`odaiba_tl_mapping`
- SUMO image: 上記`external-state-4`
- standard Odaiba traffic cache at SUMO time `500 s`
- CARLA actor radius: `300 m`、hysteresis: `20 m`
- physics radius: AV中心`100 m`、exit hysteresis: `110 m`
- feedback: `apply`、物理対象AV/BVすべて
- assimilation: `external_state`、immediate validation enabled
- feedback failure: fail closed
- SUMO/CARLA step: `0.05 s`
- requested frames: `700`

### Results

- container exit code: `0`
- SUMO steps: `700`
- completed SUMO time: `500.10`から`535.05 s`
- 全隣接SUMO time delta: `0.05 s`
- feedback records: `2155`
- queued external states: `2130`
- rejects: `25`、全て`carla_spawn_transform_pending`
- immediate validation failure / fail-closed / traceback: `0`
- control traces: `2826`
- 物理化actor: AVとBV 8台、計9台
- 全9台で`physics_enabled`と`stable`
- 全物理actorのPhase B pair: `2122`
- 全物理actorのPhase B変位最大: `0.824492 m`
- 全物理actorの次Phase A pair: `2130`
- 全物理actorの次Phase A補正最大: `0.340884 m`
- 全物理actorの速度隣接差最大: `1.342539 m/s`
- low speedと`>10 m/s`の隣接交互振動: `0`

`vehicle1510`は物理対象になった`510.45`から`534.15 s`まで475 control recordがあり、
lane遷移はroute上の次だけだった。

- `edge_420_1 -> :ia_300005_10_1`
- `:ia_300005_10_1 -> edge_426_1`
- `edge_426_0` hit: `0`
- Phase B変位最大: `0.726920 m`
- 次Phase A補正最大: `0.059525 m`
- 速度隣接差最大: `0.778919 m/s`
- yaw隣接差最大: `0.138916 deg`

旧異常時刻`532.45 s`では次の値だった。

- lane: `edge_426_1`
- CARLA speed: `10.306509 m/s`
- CARLA/SUMO yaw: `159.757332 deg`
- Phase B変位: `0.519583 m`
- 同時刻の次Phase A補正: `0.003500 m`

`532.35`から`532.75 s`の全9 sampleでもlaneは`edge_426_1`のままで、
Phase B変位は`0.504101`から`0.581608 m`、次Phase A補正は
`0.001763`から`0.006771 m`だった。速度に0.065/16.845 m/sの交互振動はなく、
yawも約159.7から159.8度へ連続した。旧`6.778729 m` / `6.795896 m`往復と
`edge_426_0` 1-tick誤遷移は再現しなかった。

前回`532.60 s`の`vehicle1510` / `vehicle2286`物理BV衝突も再現しなかった。
`collision.xml`には次のSUMO-only junction collisionが2件あるが、両車とも物理対象外で、
AVから約3 km離れている。

- `502.20 s`: `vehicle2135` / `vehicle837`、AVから`3110.420 m`
- `525.40 s`: `vehicle454` / `vehicle2552`、AVから`2941.063 m`

したがってlane-boundary warpと、それに続いた100 m内物理BV衝突については、
予測衝突guardなしでSUMO source修正単独により解消したと判定する。

### Cleanup and artifacts

終了後の`carla-novnc-test`はasync、fixed deltaなし、vehicle/sensor/walker actorとも0。
既存CARLA、Autoware、spectatorは停止せず、停止済みtest service containerだけ削除した。

- `/tmp/terasim-odaiba-physics100-site/site_run_output/service_v4_700.log`
- `/tmp/terasim-odaiba-physics100-site/site_run_output/carla_profile_v4_700.jsonl`
- `/tmp/terasim-odaiba-physics100-site/site_run_output/terasim_profile_v4_700.jsonl`
- `/tmp/terasim-odaiba-physics100-site/site_run_output/carla_initialization_v4_700.jsonl`
- `/tmp/terasim-odaiba-physics100-site/odaiba_tlmappings_0708_ackermann/raw_data/0_0/9cdb726a-383b-40fc-a26e-0a63a5a2d70c/terasim_cosim_plugin.log`
- `/tmp/terasim-odaiba-physics100-site/odaiba_tlmappings_0708_ackermann/raw_data/0_0/9cdb726a-383b-40fc-a26e-0a63a5a2d70c/collision.xml`

## 14. Recommended next step

このcheckpointをcommitする。次は予測衝突guardを入れず、同じ100 m物理化条件で
複数seedまたはより長いrunを実施し、edge orderingが異なる別のlane境界でも同じ
projection invariantが保たれることを確認する。処理時間改善はユーザー指示どおり別課題とする。
