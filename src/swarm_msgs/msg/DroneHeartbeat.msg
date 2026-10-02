# Peer-to-peer swarm heartbeat. Every drone publishes this ~2 Hz on
# /swarm/heartbeat and listens to everyone else's. Nothing here depends on a
# ground station.
#
# Positions are in the SHARED frame: drone 0's home, PX4 NED axes (north, east),
# altitude positive up. Each drone converts its own local NED position by adding
# its spawn offset, so peers can compare positions directly.

uint8 STATE_BOOT=0        # node up, not yet airborne under our control
uint8 STATE_ENGAGING=1    # arming / climbing at home
uint8 STATE_SURVEY=2      # flying its own band
uint8 STATE_TAKEOVER=3    # flying another drone's unfinished lanes
uint8 STATE_HOLD=4        # own band done, hovering while a silent peer's deadline runs out
uint8 STATE_YIELD=5       # separation hold: a peer is too close
uint8 STATE_RETURNING=6   # RTL under PX4 control (finished, or returning early)
uint8 STATE_LANDED=7      # back home, mission over for this drone
uint8 STATE_FAILED=8      # gave up (arm timeout, return timeout)

uint8 drone_id
uint8 num_drones
builtin_interfaces/Time stamp
uint8 state

float32 north             # m, shared frame
float32 east              # m, shared frame
float32 alt               # m above home

int8 band                 # band being flown right now, -1 if none
int8 lane                 # lane index within that band, -1 if none
uint8 own_lanes_done      # lanes of its OWN band completed, flown in order
uint8 own_lanes_total
int8 claimed_band         # band it is taking over, -1 if none
uint8 claimed_from_lane   # first lane of that band it is flying
uint32 bands_done_mask    # bit b set = band b fully covered by this drone
float32 eta_s             # estimated seconds to finish its remaining lanes, -1 unknown
float32 battery_pct       # 0-100, -1 unknown
