# One geotagged detection, broadcast drone-to-drone on /swarm/hazards so every
# drone can de-duplicate against everyone else's finds without a ground station.
#
# hazard_id = "d<origin_drone>-<seq>" is unique swarm-wide. When two drones log
# the same object (inside min_sep_m) the canonical one is the EARLIER stamp,
# ties going to the lower origin_drone; every drone applies the same rule, so
# their lists converge.

string hazard_id
uint8 origin_drone
uint32 seq
builtin_interfaces/Time stamp
float32 north             # m, shared frame (drone 0's home)
float32 east
float32 alt               # altitude the detection was made from
string cls
float32 conf
