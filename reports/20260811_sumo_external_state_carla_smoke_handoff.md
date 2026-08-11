# SUMO external state assimilation / 実CARLA 1車両スモーク 引き継ぎ

## 1. 結論

Odaibaへ進む前のゲートとして、専用SUMO `setExternalState` API、TeraSim serviceの
`external_state` assimilation mode、1車両integration test、実CARLA Town01スモークを
実装・検証した。

実CARLA 60フレームでは、修正前に低速でも毎step約0.200 m進んでいたSUMO Phase B変位が
最大0.00496 mへ低下した。次Phase Aの低速補正は最大0.02006 mであり、以前の
0.065↔16.845 m/s、yaw 104↔161度の交互振動は発生しなかった。

小規模ゲートは通過したが、Odaiba統合走行はまだ実施していない。次は専用imageと
fail-closed validationを維持したまま、OdaibaでAV 1車両・30～60フレームのguarded smokeを
行う。

## 2. Git基準

- branch: `kawai/experiment/sumo-two-phase-movetoxy-20260810`
- 作業開始時および本checkpointの親HEAD:
  `c2aac41dca0673389cd2b01ff97cae6ce27c3e4f`
- `c2aac41` の直接緊急ブレーキ修正は保持されている。
- 失敗した0.025秒×2回のtwo-substep実験は保持していない。
- Phase Bは従来どおり0.05秒の`simulationStep()`を1回だけ実行する。
- 既存のユーザー変更と20260810の2本の引き継ぎレポートは保持した。

## 3. 実装した構成

### Phase A: 時間を進めないassimilation

CARLAのx/y/yaw/speed/accelerationを、専用TraCI API
`traci.vehicle.setExternalState(...)`でSUMOへ即時反映する。

SUMO source側の処理順は次のとおり。

1. 既存`moveToXY`のroad/lane mappingを利用する。
2. 対象車両のqueued remote stateを即時適用する。
3. global remote-control queueから対象車両を除去する。
4. same-timestamp remote-control latchを完了状態にする。
5. 古いTraCI `setSpeed` timelineを`setSpeed(vehID, -1)`で解除する。
6. 外部speed/accelerationを`setPreviousSpeed`で現在状態へ設定する。

Phase Aではplanning、movement、output、SUMO時刻更新を行わない。serviceは直後に時刻、位置、
yaw、speedを読み戻す。既定では位置1 mm、yaw/speed 1e-6の範囲外、または時刻変化があれば
fail closedで通常のSUMO stepを拒否する。

### Phase B: 通常の意思決定と1 step

TeraSim pipelineは次のpriority順を維持する。

- `-90`: serviceがCARLA feedbackをPhase Aとして適用
- `0`: TeraSim/NADEが反映済み状態を観測し行動を決定
- `10`: 通常の`simulationStep()`を0.05秒だけ1回実行

Phase Aで古いspeed timelineを解除した後、priority 0のTeraSim/NADEは必要なら新しい
TraCI speed actionを設定できる。CARLAは従来どおり0.05秒tickを1回行う。

## 4. 重要な原因と修正

最初の実CARLAスモークではPhase Aの即時一致自体は成功したが、CARLA速度がほぼ0 m/sでも
Phase B後のSUMO位置が毎回約0.200 m進み、次Phase Aで約0.200 m戻されていた。

原因は`NADEWithAV.add_av_unsafe()`がAV追加時に実行する`setSpeed(AV, 4.0)`である。
SUMOではこれは長期間有効なspeed timelineとなる。旧`setExternalState`はcurrent speedだけを
更新し、このtimelineを残していたため、speedMode 0のPhase Bで4.0 m/sが再適用されていた。

専用APIがPhase Aで古いtimelineを解除するよう変更した。退行テストではさらに厳しい
16.845 m/sのstale commandとspeedMode 0を設定し、修正前のPhase B変位0.84225 mを再現して
から修正後の通過を確認した。

## 5. lane declared length / shape length正規化

この修正はtwo-substep実験とは独立して保持した。

SUMOの`lanePosition`はlaneのdeclared length基準だが、修復・簡略化されたpolyline shapeの実長が
一致しない場合がある。lane-relative座標からx/yを復元するとき、
`lanePosition / declaredLength`を進捗率としてshape lengthへ写像するよう変更した。

`external_state` modeではSUMO側とCARLA側のlane-relative state利用を既定で有効にする。
Phase A対象車両ではstale subscriptionよりlive `getLaneID/getLanePosition/
getLateralLanePosition`を優先する。legacy modeの既定動作は変更しない。

## 6. 主なファイル

- `Dockerfile.sumo-external-state`
  - SUMO v1.23.1固定sourceへpatchを適用して専用imageをbuildする。
- `apps/sumo_external_state/sumo-v1.23.1-set-external-state.patch`
  - TraCI/libsumo API、remote queue即時適用、remote latch解除、speed timeline解除を実装する。
- `apps/sumo_external_state/README.md`
  - API契約、build、非login shellでのvalidation手順を記載する。
- `packages/terasim-service/terasim_service/plugins/cosim.py`
  - `legacy`/`external_state`選択、lane mapping、即時readback、fail-closed validationを実装する。
- `packages/terasim-service/terasim_service/utils/carla/cosim.py`
  - `external_state`時にlane-relative targetを既定で利用する。
- `packages/terasim-service/terasim_service/utils/sumo_lane_geometry.py`
  - declared lane lengthとshape lengthの正規化を実装する。
- `docker-compose.ackermann-odaiba-feedback-gui.yml`
  - assimilation mode、validation、位置toleranceの環境変数を追加する。
- `tests/test_integration/test_sumo_external_state.py`
  - raw API、pipeline priority、single-step、stale speed latchの1車両ゲートを実装する。
- `tests/test_service/test_carla_ackermann_feedback.py`
  - service mode、fail closed、readback rounding、lane-relative stateのunit testを追加する。

## 7. 専用SUMO build

- upstream tag: `v1_23_1`
- upstream commit: `676720d13f6f42d8c79d156e9d67001f8c22f6f6`
- experimental vehicle variable ID: `0xf8`
- image: `terasim-service:sumo-external-state-v1.23.1`
- image label: `SUMO-v1.23.1-external-state-2`
- image digest:
  `sha256:6b30cf23d26cb656ea4f22b168198e71eaf374b55d08841a33412c2d5fcc9932`

build時に固定commit確認、patchの`git apply --check`、SUMO/TraCI/libsumo API self-checkを行う。

注意: validation containerは`/bin/bash -c`を使うこと。`-lc`ではlogin shellがimageのPATHを
置き換え、専用`/opt/sumo-external-state/bin/sumo`ではなくstock
`/usr/local/bin/sumo`を選ぶ場合がある。

## 8. 自動テスト結果

専用image内で次を実行した。

```bash
docker run --rm \
  -v /home/h-kawai/TeraSim-ackermann-feedback-gui:/workspace:ro \
  -w /workspace \
  -e HOME=/tmp \
  --entrypoint /bin/bash \
  terasim-service:sumo-external-state-v1.23.1 \
  -c 'test "$(command -v sumo)" = /opt/sumo-external-state/bin/sumo && \
      pip install pytest==8.3.5 >/dev/null && \
      python -m pytest -o addopts= -p no:cacheprovider -q \
      tests/test_service/test_carla_ackermann_feedback.py \
      tests/test_integration/test_sumo_external_state.py'
```

結果:

- `82 passed, 5 warnings in 3.68s`
- bundled unified diffを除く通常変更の`git diff --check`: clean
- 変更Python 5ファイルの`py_compile`: pass

## 9. 実CARLA＋service 1車両スモーク

### 条件

- CARLA: 0.9.16、Town01、offscreen、隔離container
- TeraSim: direct gRPC service
- environment: `NADEWithAV`
- 車両: AV 1台、BVなし
- SUMO/CARLA step: 0.05秒
- requested frames: 60
- feedback mode: `apply`
- assimilation mode: `external_state`
- external-state validation: enabled、位置tolerance 1 mm
- lane-relative mode: 明示指定せず、`external_state`既定値を検証
- Odaiba: 未実施

run UUID:
`f10c8fcb-80c8-4e1b-9cb4-8846dfa78177`

一時ログ:

- `/tmp/terasim-external-state-smoke/smoke_run7.log`
- `/tmp/terasim-external-state-smoke/output/external_state_smoke/raw_data/one_vehicle/f10c8fcb-80c8-4e1b-9cb4-8846dfa78177/terasim_cosim_plugin.log`
- `/tmp/terasim-external-state-smoke/output/external_state_smoke/raw_data/one_vehicle/f10c8fcb-80c8-4e1b-9cb4-8846dfa78177/run.log`

### 結果

- service/SUMO step: 60回
- completed SUMO time: 0.10～3.05秒
- 全隣接SUMO time delta: 0.05秒
- feedback records: 60
- spawn transform pendingによる初期rejected: 4
- Phase A accepted: 56
- accepted CARLA frames: 154757～154812、全て連続
- control traces: 56
- Phase A validation failure: 0
- service ERROR/CRITICAL、Traceback: 0
- lane ID: 全て`-1_0`
- lane position: 10.8000～12.0476 m

不変条件の集計:

| 項目 | 結果 |
| --- | ---: |
| accepted CARLA speed | 0.0000～3.8602 m/s |
| speed隣接差最大 | 1.3199 m/s |
| 0.065未満/付近 ↔ 10 m/s超の交互振動 | 0件 |
| accepted yaw | 270.03175～270.03267度 |
| yaw隣接差最大 | 0.000137度 |
| yaw 104度帯または161度帯 | 0件 |
| Phase B変位（全55対応区間） | 0.00460～0.19751 m |
| CARLA speed 0.1 m/s未満のPhase B変位最大 | 0.00496 m |
| 次Phase A補正（全対応区間） | 0.00417～0.04686 m |
| CARLA speed 0.1 m/s未満の次Phase A補正最大 | 0.02006 m |

最大Phase B変位0.19751 mはCARLA speed 3.8602 m/sの初期走行区間であり、0.05秒stepと整合する。
低速時の固定0.200 m前進と次Phase Aでの同量巻き戻りは解消した。

終了時に隔離CARLAはTown01、非同期、fixed deltaなし、vehicle 0、sensor 0へ戻した後で
停止・削除した。既存`carla-novnc-test`、`autoware-cosim`、`spectator-cam`は停止していない。

## 10. legacy互換と運用上の注意

- repository既定は`legacy`であり、既存`moveTo`＋`setPreviousSpeed`経路を維持する。
- `external_state`を選んで専用APIが存在しない場合、SUMO step前にfail closedする。
- API variable ID `0xf8`は実験用である。今後upstream SUMOと衝突しないか確認が必要。
- 専用imageはlocal buildであり、registryへpushしていない。
- `/tmp`のrun7ログはcommit対象ではない。本レポートに主要数値を固定した。
- 現時点の検証はTown01、AV 1台、BVなし、3.0秒相当までである。

## 11. 次の作業

1. 本checkpointから専用imageを指定する。
2. `CARLA_COSIM_ACKERMANN_FEEDBACK_ASSIMILATION_MODE=external_state`を明示する。
3. validation、1 mm tolerance、feedback/control traceを有効にする。
4. OdaibaでAV 1車両、30～60フレームのみ実行する。
5. Phase A failure、Phase B変位、次Phase A補正、speed/yaw振動をrun7と同じ集計で確認する。
6. 合格後にのみBVを少数追加する。
7. 通常規模・長時間のOdaiba走行はその後に行う。

Odaiba guarded smokeで不変条件が崩れた場合は、simulationをfail closedで止め、BV追加や
長時間走行へ進まないこと。
