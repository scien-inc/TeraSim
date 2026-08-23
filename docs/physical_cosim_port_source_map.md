# Physical CoSim port source map

This branch ports the validated physical CoSim behavior without merging the
history of `kawai/feat/physic-cosim`. The canonical source tip is `b04321c`;
`b6897d5` and `f1d46ac` are the specific lateral synchronization sources.

| Ported area | Canonical source | Main implementation | Port rule |
| --- | --- | --- | --- |
| Patched SUMO `setExternalState` | `b04321c:apps/Docker/sumo-v1.23.1-set-external-state.patch` | Same path | Identical patch blob; retain the main in-process Docker base |
| Route/lateral helper | `f1d46ac:packages/terasim-service/terasim_service/utils/sumo_lane_geometry.py` | Same path | Identical file blob (`8bf78ce2c81ae9ce13defdab31ee173c6e371d7c`) |
| Phase-aligned state schema | `f1d46ac:packages/terasim-service/terasim_service/utils/messages/AgentStateSimplified.py` | Same path | Identical file blob (`bee890e256c2930bd636bb00e09fce031cec0b3b`) |
| Live local route | `b6897d5` and `f1d46ac` in `plugins/cosim.py` | `plugins/cosim_inprocess.py` | Adapter obtains live Phase B lane, position, route and first next link, then calls the canonical helper |
| Requested/observed/live lane split | `f1d46ac` in `plugins/cosim.py` | `plugins/cosim_inprocess.py` | Adapter stores Phase A requested/observed values separately from Phase B live values |
| Initial pose state | `b04321c:plugins/cosim.py` state export and `_populate_lane_relative_position` | `plugins/cosim_inprocess.py` | Adapter exports SUMO slope before the first feedback and passes declared lane length plus raw SUMO x/y to the canonical reconstruction helper |
| CARLA Ackermann math | `b04321c:packages/terasim-service/terasim_service/utils/carla/ackermann_control.py` | Same path | Main already has the identical feature-tip blob |
| CARLA front/rear reference conversion | `b04321c` functions `_carla_transform_to_sumo_feedback_state`, `_sumo_front_to_carla_transform`, and phase-aligned error helpers | `utils/carla/cosim.py` | Function bodies retained; the wrapper emits main's in-process `set_state` command |
| Physics initialization | `b04321c` functions `_prepare_ackermann_actor_physics` through `_ensure_ackermann_actor_physics` | `utils/carla/cosim.py` | Footprint overlap, one completed-frame wait, three stable ticks, initial velocity, wheel geometry, and retry limit retained |
| Ackermann production control | `b04321c` functions `_resolve_ackermann_longitudinal_target` through `_record_ackermann_control_trace` | `utils/carla/cosim.py` | Restart, per-type emergency decel, brake hysteresis, steering preservation, and control equations retained |
| Initialization/collision diagnostics | `0d3899c` as present at `b04321c` | `utils/carla/cosim.py` | JSONL and summary code retained and defaulted off |
| Master physical tick pipeline | `b04321c:utils/carla/cosim.py` `_tick_ackermann_feedback_apply_direct` | `utils/carla/cosim.py` `_tick_master` | Resolve the previous SUMO result, apply state, tick CARLA once, collect that frame's feedback, and defer the newly requested SUMO result until the next cycle |
| Phase A feedback order | `b04321c:utils/carla/cosim.py` `_collect_ackermann_feedback` | `utils/carla/cosim.py` `_build_physics_feedback_commands` | Selected background actors are emitted in the same lexicographic actor-ID order; this preserves deterministic eager `moveToXYImmediate` lane-change neighborhood updates |

The adapter preserves main's state filter, actor index, batched controls, and
master/follower/async tick structure. It does not restore Redis, HTTP, gRPC, or
direct-link transports.

CARLA adapter-only differences are limited to reading the current in-process
state, using main's persistent `role_name` actor index, appending control to
main's existing batch, and raising an authoritative-action error only after
the same-frame fail-closed brake has been applied. It never adds an extra
CARLA tick while main is the clock master. The master adapter holds at most one
in-process SUMO request: it resolves and applies the previous result before the
next CARLA frame, then submits the completed frame's feedback without waiting
for that new request in the same cycle. The follow and async pipelines retain
their existing ordering.

An AST audit against
`b04321c:packages/terasim-service/terasim_service/utils/carla/cosim.py`
found 102 function names shared by the two implementations. Of those, 77 have
identical function ASTs. The remaining 25 are the constructor, lifecycle/tick,
actor-index/batch, in-process feedback-ack, trace-state, blueprint, AV, TLS,
VRU, and first-Phase-A initialization gate adapter boundaries. The gate keeps
a newly stabilized actor at its canonical initial velocity for one frame in
the unchanged follow/async pipelines. Master does not use the gate: its
state-apply -> CARLA tick -> feedback order matches feature and issues the
first Ackermann control as soon as physics stabilization completes.
The 29 feature-only functions are the removed
direct/HTTP roundtrip, profiling, and their transport-specific feedback/index
helpers; they are not restored. The 16 main-only functions implement the
in-process master/follower/async tick, persistent indexes, stale-actor cleanup,
and feedback-command adapter.

The baseline deliberately excludes the first port attempt's route recovery,
position continuity correction, spawn-height adjustment, and ground-wait
logic. Those changes remain only on
`kawai/archive/port-physical-cosim-attempt1` for failure reproduction.
