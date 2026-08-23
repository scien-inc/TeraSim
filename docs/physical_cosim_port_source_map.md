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
| CARLA Ackermann math | `b04321c:packages/terasim-service/terasim_service/utils/carla/ackermann_control.py` | Same path | Main already has the identical feature-tip blob |

The adapter preserves main's state filter, actor index, batched controls, and
master/follower/async tick structure. It does not restore Redis, HTTP, gRPC, or
direct-link transports.

The baseline deliberately excludes the first port attempt's route recovery,
position continuity correction, spawn-height adjustment, and ground-wait
logic. Those changes remain only on
`kawai/archive/port-physical-cosim-attempt1` for failure reproduction.
