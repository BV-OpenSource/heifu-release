import geometry_msgs.msg
from geometry_msgs.msg import Twist
import mavros_msgs
from mavros_msgs.msg import WaypointList, Waypoint


def TwistCv(msg):
    conversion = Twist()
    #sent = json.loads(json.dumps(msg))
    conversion.linear.x = msg['x']
    conversion.linear.y = msg['Y']
    if msg['z'] < 0:
        conversion.linear.z = -1
        pass
    else:
        conversion.linear.z = msg['z']
        pass
    conversion.angular.x = msg['R']
    conversion.angular.y = msg['P']
    conversion.angular.z = msg['y'] / 6.0
    return conversion

def WaypointCv(msg):
    conversion = Waypoint()
    conversion.frame = 3
    conversion.command = int(msg['command'])
    conversion.is_current = False
    conversion.autocontinue = False
    conversion.param1 = float(msg['param1'])
    conversion.param2 = float(msg['param2'])
    conversion.param3 = float(msg['param3'])
    conversion.param4 = float(msg['param4'])
    conversion.x_lat = float(msg['xLat'])
    conversion.y_long = float(msg['yLong'])
    conversion.z_alt = float(msg['zAlt'])

    return conversion, conversion.command
