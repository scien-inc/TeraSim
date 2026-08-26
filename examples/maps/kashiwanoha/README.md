# Kashiwanoha campus map (SUMO net for co-simulation)

SUMO network of the Kashiwanoha campus area, for Autoware x CARLA x TeraSim
co-simulation. Converted from the public Kashiwanoha lanelet2 map:

- Source: https://github.com/tier4/scenario_simulator_v2
  `map/kashiwanoha_map/map/private_road_and_walkway_ele_fix/lanelet2_map.osm`
  (commit 6a805e47), License: **Apache-2.0** (TIER IV, Inc.)
- Conversion chain: lanelet2 -> OpenDRIVE (autoware_lanelet2_to_opendrive
  v2.62.0, origin = MGRS 54SVE in-square (3750, 73750)) -> this net
  (scripts/xodr_to_sumo_converter.py). This file is a derivative work of the
  Apache-2.0 licensed map above.
- Coordinate frame: `<location>` declares raw coords as UTM 54N with
  netOffset (-403686.78, -3973677.87), so the CARLA co-simulation layer can
  auto-calibrate against the matching OpenDRIVE geoReference
  (lat_0=35.9033135426554, lon_0=139.93338978245356).

Files:

| file | purpose |
|---|---|
| kashiwanoha.net.xml | SUMO network (no traffic lights; all priority junctions) |
| kashiwanoha.trips.xml | random background traffic (seed 42, period 12s) |
| kashiwanoha_empty.trips.xml | vType only, no background traffic |
| kashiwanoha_dt005.sumocfg / _notraffic.sumocfg | step 0.05s configs |
| vtypes.add.xml | NDE_URBAN vehicle type (referenced by trips generation) |

Known limitations: the small ring network with priority-only junctions can
gridlock under sustained random traffic (time-to-teleport is disabled);
tune the trips period or use the notraffic config for ego-only runs.
