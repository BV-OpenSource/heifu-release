# Connect gimbal to PixHawk

First we need to connect **RX**, **TX** and **GND** serial channels to Telem2 of Pixhawk like the following image:

<img src="simplebgc-gimbal-pixhawk.png" width="300px" height="300px" align="center"/>

## Setup in QGround Control

Alter the following params:

- MNT_TYPE = AlexMos-Serial
- SERIAL2_PROTOCOL = AlexMos Gimbal Serial (Because we are using telem2 if it was telem1 would be SERIAL1)

## Setup minimum and maximum angles:

1. Roll
	1. MNT_ANGMIN_ROL = -35
	2. MNT_ANGMAX_ROL = 35
2. TILT (PITCH)
	1. MNT_ANGMIN_TILT = -90
	2. MNT_ANGMAX_TILT = 90
3. PAN (YAW)
	1. MNT_ANGMIN_PAN = -180
	2. MNT_ANGMAX_PAN = 179

## Services to control gimbal via MAVROS

To set value:

```rosservice call /mavros/param/set```

To check current param value:

```rosservice call /mavros/param/get```

Params id:
```
MNT_NEUTRAL_X #ROLL
MNT_NEUTRAL_Y #PITCH
MNT_NEUTRAL_Z #YAW
```

## Subscribers

* gimbal/getAxes [[gimbal::getGimbalAxes](msg/getGimbalAxes.msg)]
* gimbal/setAxes [[gimbal::setGimbalAxes](msg/setGimbalAxes.msg)]
