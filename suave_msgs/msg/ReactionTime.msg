# Reaction time of one adaptation type, measured by suave_metrics/mission_metrics.
# Times are in seconds. latest and mean are only meaningful when status is
# STATUS_VALID; they are 0.0 while no reaction has been recorded.

uint8 STATUS_INVALID=0
uint8 STATUS_VALID=1

string ADAPTATION_THRUSTER="thruster"
string ADAPTATION_WATER_VISIBILITY="water_visibility"
string ADAPTATION_BATTERY="battery"

builtin_interfaces/Time stamp

# One of the ADAPTATION_* constants.
string adaptation_type

# One of the STATUS_* constants.
uint8 status

# Most recent reaction time.
float64 latest

# Mean of all reaction times recorded so far.
float64 mean

# Number of reactions recorded so far.
uint32 count
