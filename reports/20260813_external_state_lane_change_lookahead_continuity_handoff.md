# External-state lane-change lookahead連続化 引き継ぎ

日付: 2026-08-13
branch: `kawai/experiment/sumo-two-phase-movetoxy-20260810`

## 結論

external-state actorのCARLA操舵用lookaheadを、SUMO `getAngle()`の直接投影から、SUMOが選んだlane-change intent・target lane・lateral speedとordered route geometryの座標変換へ置換した。SUMO API、SUMO lane-change model、lane policy、legacy angle blendは変更していない。

保存済み`vehicle1167`再現とOdaiba 522–528秒窓では、旧`+0.18 -> -0.51`型およびfield初回の`-0.170 -> +0.158`型raw steer反転が解消した。修正後の最大cycle差はraw steer `0.040824 rad`、commanded steer `0.030000 rad`で、`target_behind=0`、invalid lookahead `0`、lane rollback `0`だった。

ただし、修正後field runは対象区間を通過した後の544.30秒にSUMOプロセスがエラーメッセージなしで終了し、TeraSimが`FatalTraCIError: Connection closed by SUMO`を受けた。このためlookahead課題のguarded acceptanceは通過、field run全体はpartial passとする。別課題として再現・調査が必要である。

## Commit

- `ee2fa80 test: reproduce external-state lane-change lookahead discontinuity`
- `3aba46e fix: derive external-state lookahead from SUMO lateral action`
- 本report: `docs: record external-state lane-change continuity results`

## 根本原因とfieldで追加検出した境界

従来external-state経路は、primary laneがsourceからtarget/internalへ切り替わる付近でSUMO `getAngle()`をCARLA目標headingへ混ぜていた。SUMO angleはlane-change中の物理CARLA yawではなくPhase BのSUMO目標値であり、lane/internal geometryの切替で大きく変化する。これが同一cycleのlookahead左右反転とpure-pursuit raw steer急反転を発生させていた。

最初のfield runでは、angle依存を除去した後にも次の境界を検出した。

```text
time        SUMO lateral speed   lateral preview   lookahead local Y   raw steer
523.10 s    -1.0 m/s             3.193 m           -1.777 m            -0.170 rad
523.15 s     0.0 m/s             0.000 m           +1.368 m            +0.158 rad
```

曲線route上でlateral speedが一時的に0になると、previewが1 cycleでtarget方向から`O + D`へ約3 m戻るため、angleを使わなくてもlookaheadが不連続になっていた。

## 実装

### Rolling route/lateral lookahead

`packages/terasim-service/terasim_service/utils/sumo_lane_geometry.py`へ`build_external_state_lateral_action_lookahead()`を追加した。

- CARLA rear-axle位置`O`を現在位置の正とする。
- current laneとordered next linksからcompileしたrouteへ`O`を投影し、`P0`とlookahead距離先`P1`から`D=P1-P0`を求める。
- SUMOが明示したtarget lane shape上の最近傍点`Q`だけを使用する。adjacent laneの独自検索やtarget変更は行わない。
- current route tangentに直交する`r`を求め、従来計画式を要求previewとする。

```text
requested = min(|r|, |SUMO lateral speed| * L / SUMO desired speed)
lookahead = O + D + normalize(r) * applied
```

- SUMO angleはhelperの入力に含めない。
- target geometry、route geometry、action値が欠落・非有限ならfail closedにする。
- desired speed `<=0.2 m/s`は`deferred`とし、新しい横運動やcreepを開始しない。

### Cycle間のpreview連続化

field初回で検出した0/再開境界に対し、maneuverごとのapplied lateral previewをserviceで保持する。requested値、方向、targetはSUMO値のまま、applied値の1 cycle増減だけを次で制限する。

```text
max change = maneuver中にSUMOが要求した最大|lateral speed| * 0.05 s
```

これにより、SUMO lateral speedが0または低速deferredへ変化しても、previewだけが1 cycleで数m消失・復活しない。0要求が続けばapplied previewは同じSUMO由来rateで0へ到達し、最終的に`O + D`となる。SUMOが決めたlane、方向、target、速度を変更する処理ではない。

### Maneuver lifecycle

- 新規maneuverはSUMOの明示的`left/right` intentだけで開始する。
- source/target laneを保持し、intentが一時的に`none`でも同じtargetを維持する。
- primary laneがtargetへ切り替わっても旧laneへ戻さない。
- current laneがtarget、`|lateral speed|<0.05 m/s`、物理target横誤差`<0.15 m`の全条件でのみ解除する。
- unrelated primary lane、target geometry欠落、非有限値ではangle fallbackせずfail closedにする。
- external-stateの`lookahead_lane_change_blend`は常に`0.0`。
- legacy経路は既存`legacy_angle` blendを維持する。

## Diagnostics

`AgentStateSimplified`と`AckermannControlTrace`へ次を追加した。

- route base lookahead x/y/z
- `lookahead_action_mode`: `route | sumo_lateral_velocity | deferred | legacy_angle`
- action valid/error
- lateral horizon displacement
- target laneまでの物理横距離
- 同一cycleのSUMO lane/intent/target/angle/lateral speed、Phase A pose/yaw、最終lookahead、raw/commanded steer、CARLA yaw

CARLA executorはexternal-state action lookaheadがinvalid/missingならSUMO angleへfallbackせずfail-closed brakeとする。

## Test結果

### Service・integration

- 保存済み旧9-cycle不連続再現: PASS
- `vehicle1167`実Odaiba shape 10-cycle replay: PASS
- left/right、曲線、primary switch、intent flicker、blocker、低速deferred、settlement: PASS
- invalid target/route/non-finite fail closed: PASS
- legacy angle blend回帰: PASS
- service + integration: `183 passed`
- exact dedicated SUMO integration: `23 passed`
- `git diff --check`: PASS

### SUMO patch/build

- clean SUMO v1.23.1 commit `676720d13f6f42d8c79d156e9d67001f8c22f6f6`への`git apply --unidiff-zero --check`: PASS
- `apps/sumo_external_state/sumo-v1.23.1-set-external-state.patch`の今回差分: 0
- TraCI/libsumo `moveToXYImmediate` self-check: PASS
- final image:

```text
terasim-service:sumo-external-state-lateral-action-continuous-20260813
sha256:5b9f826e9f60a93451b479d0757a83a14159b8ef4ded88596ad8f951674b4794
```

## Odaiba guarded run

設定:

```text
CARLA_COSIM_ACKERMANN_FEEDBACK_ACTORS='AV,*'
assimilation=external_state
strictLaneHint=true
step=0.05 s
SUMO GUI noVNC=6093
CARLA/chase noVNC=6092
seed=42
```

起動ログは`actors=['*', 'AV']`を示した。修正後runではAV 886 records、BV 86 actors、`vehicle1167` 886 feedback recordsを確認した。

### 修正前field初回比較（522–528秒、121 cycles）

- raw steer最大cycle差: `0.328421 rad`
- commanded steer最大cycle差: `0.059784 rad`
- local lookahead X最小: `6.864 m`
- `target_behind=0`
- invalid action: `0`
- angle blend最大: `0.0`
- 正常終了: SUMO 543.90秒、TraCI requested termination、container exit 0

### 修正後（522–528秒、121 cycles）

- raw steer最大cycle差: `0.040824 rad`
- commanded steer最大cycle差: `0.030000 rad`
- CARLA yaw最大cycle差: `0.412495 deg`
- local lookahead X最小: `6.997 m`
- `target_behind=0`
- invalid action: `0`
- angle blend最大: `0.0`
- 8 m mapping failure: `0`
- frame mismatch: `0`

`vehicle1167` lane列:

```text
edge_417_0
-> :ia_300006_11_1
-> edge_420_1
-> edge_420_2
-> :ia_300005_10_2
-> edge_426_2
```

同じlaneへのrollbackやpredecessor/internalへの巻き戻りはなかった。

### 残課題

- 修正前にも`vehicle1167`と`vehicle1593`のjunction collisionが522.90秒に存在した。修正後は524.55秒に発生しており、新規衝突ではないが未解決である。
- 修正後runは544.30秒、`executeMove()`中にSUMOがログ上の明示エラーなしで終了した。TeraSimは`FatalTraCIError: Connection closed by SUMO`を受け、CARLA clientが待機したため、当該TeraSim containerだけを停止した。
- この終了はlookahead helperのPython例外、mapping threshold、invalid action、frame mismatchではない。SUMO process exit/クラッシュの独立再現が必要である。
- 既存`carla-novnc-test`、`autoware-cosim`、`spectator-cam`は停止・再起動しておらず、report作成時も稼働中である。

## 変更していないもの

- SUMO source/API patch
- `moveToXYImmediate`仕様
- strict mappingとprimary lane規則
- SUMO lane-change model/state/parameters/policy
- normal `moveToXY`とlegacy assimilation
- pure pursuit、操舵角上限、操舵rate上限
- speed補正、collision guard、追加speed limiter
