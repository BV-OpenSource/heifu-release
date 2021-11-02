# Desired frame
uint8 frame
uint8 FRAME_GLOBAL_INT = 5
uint8 FRAME_GLOBAL_REL_ALT = 6
uint8 FRAME_GLOBAL_TERRAIN_ALT = 11
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_LOCAL_OFFSET_NED = 7
uint8 FRAME_BODY_NED = 8
uint8 FRAME_BODY_OFFSET_NED = 9

# Desired waypoint controller type
uint8 type
uint8 TYPE_POSITION = 0                 # Position with yaw
uint8 TYPE_POSITION_ONLY = 1            # Only Position
uint8 TYPE_POSITION_YAW = 2             # Set yaw first, then goto Position
uint8 TYPE_POSITION_YAW_WP = 3          # Set heading to WP, goto Position
uint8 TYPE_POSITION_YAW_FIRST_WP = 4    # Set heading to WP, then goto Position
uint8 TYPE_VELOCITY = 10                # Velocity with yaw
uint8 TYPE_VELOCITY_ONLY = 11           # Only Velocity
uint8 TYPE_VELOCITY_YAW = 12            # Set yaw first, then goto Position
uint8 TYPE_VELOCITY_YAW_WP = 13         # Set heading to WP, goto Position
uint8 TYPE_VELOCITY_YAW_FIRST_WP = 14   # Set heading to WP, then goto Position
uint8 TYPE_VELOCITY_BYPASS = 19         # Bypass controller (xyz is velocity)
uint8 TYPE_YAW_ONLY = 20                # Only set yaw
uint8 TYPE_LOITER = 30                  # Set loiter setpoint ( Fixed-wing only )

# Local position
float32 x
float32 y
float32 z

float32 yaw

# Parameters
float32 radius                  # Waypoint acceptance radius / Loiter radius (if TYPE_LOITER)
float32 yawError                # Maximum allowed yaw error
float32 maxLinearVel            # Maximum linear velocity
float32 maxAngularVel           # Maximum angular velocity
