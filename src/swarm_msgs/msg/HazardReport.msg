# One geotagged detection, broadcast drone-to-drone on /swarm/hazards so every
# drone can de-duplicate against everyone else's finds without a ground station.
#
# hazard_id = "d<origin_drone>-<seq>" is unique swarm-wide. When two drones log
# the same object (inside min_sep_m) the canonical one is the EARLIER stamp,
# ties going to the lower origin_drone; every drone applies the same rule, so
# their lists converge.
#
# The origin drone keeps refining its hazard's position as it sees it again
# (confidence-weighted mean of its sightings) and re-broadcasts with a higher
# version; receivers keep the highest version. Only the origin refines, so all
# drones converge on the same position too.

string hazard_id
uint8 origin_drone
uint32 seq
uint32 version            # 0 = first report; +1 per re-broadcast refinement
builtin_interfaces/Time stamp   # first sighting - never changes, it orders canonicity
float32 north             # m, shared frame (drone 0's home), current best estimate
float32 east
float32 alt               # altitude of the first detection
string cls
float32 conf              # best confidence seen
uint32 n_sightings        # sightings merged into this estimate
float32 spread_m          # RMS distance of those sightings from the estimate
