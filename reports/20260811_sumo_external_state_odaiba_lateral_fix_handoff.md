# SUMO external_state Odaiba lateral fix handoff

日付: 2026-08-11 JST

## 1. Repository

- branch: `kawai/experiment/sumo-two-phase-movetoxy-20260810`
- starting HEAD: `f6d1a456e2bd208627fc6814ae32a497f6b976d4`
- 本作業の変更は未commit
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

## 8. Next step

このcheckpointをreviewしてcommitした後、同じAV 1台・BV 0条件を60フレームへ延長する。
60フレームでも同じgateを通過してからのみBVを少数追加する。通常規模・長時間Odaibaへは
まだ進まない。
