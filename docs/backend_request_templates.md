# Backend Request templates

## Post Request

### /settings/manage_webinterface

restart all subscribers and publischers
```json
{
    "action": "restart"
}
```

destroy all subscribers and publischers
```json
{
    "action": "stop"
}
```

### /settings/update_json_config

example for a request to update the json config file 

```json
{
    "data": 
    {
        "component": [
        {
            "id": "topic2",
            "state": false,
            "render_type": "String",
            "position": { "x": 0, "y": 0 },
            "size": { "width": 200, "height": 200 },
            "topic": [
            {
                "name": "topic1",
                "topic": "/topic1",
                "type": "String",
                "route": "/subscriber/topic1"
            }
            ]
        },
        {
            "id": "OpenStreetMap",
            "state": false,
            "render_type": "OpenStreetMapCenter",
            "position": { "x": 0, "y": 0 },
            "size": { "width": 650, "height": 650 },
            "topic": [
            {
                "name": "gps_cordinates",
                "topic": "/gps/fix",
                "type": "NavSatFix",
                "route": "/subscriber/gps_cordinates"
            },
            {
                "name": "waypoint_reached",
                "topic": "/waypoint_reached",
                "type": "String",
                "route": "/subscriber/waypoint_reached"
            },
            {
                "name": "waypoint_add",
                "topic": "/waypoint_add",
                "type": "String",
                "route": "/publisher/waypoint_add"
            },
            {
                "name": "waypoint_delete",
                "topic": "/waypoint_delete",
                "type": "String",
                "route": "/publisher/waypoint_delete"
            }
            ]
        }
        ],
        "preset_list": [
        {
            "preset_name": "preset1",
            "size_config": { "width": 1625, "height": 900 },
            "settings": [
            {
                "id": "OpenStreetMap",
                "size": { "width": 1200, "height": 900 },
                "position": { "x": 0, "y": 0 }
            },
            {
                "id": "topic2",
                "size": { "width": 350, "height": 900 },
                "position": { "x": 1200, "y": 0 }
            }
            ]
        }
        ]
    }
}
```

## /publisher/String 

the json can be configured however needed since the strigefy method is used to convert to Ros 2 mesage formate 

## Get Requests

will be fully filled out later 

### std_msgs

#### /subscriber/Bool

Response:
```json
{

}
```


#### /subscriber/Byte

Response:
```json
{

}
```

#### /subscriber/ByteMultiArray

Response:
```json
{

}
```


#### /subscriber/Char

Response:
```json
{

}
```


#### /subscriber/ColorRGBA

Response:
```json
{

}
```


#### /subscriber/Empty

Response:
```json
{

}
```


#### /subscriber/Float32

Response:
```json
{

}
```


#### /subscriber/Float32MultiArray

Response:
```json
{

}
```


#### /subscriber/Float64

Response:
```json
{

}
```


#### /subscriber/Float64MultiArray

Response:
```json
{

}
```


#### /subscriber/Header

Response:
```json
{

}
```


#### /subscriber/Int16

Response:
```json
{

}
```


#### /subscriber/Int16MultiArray

Response:
```json
{

}
```


#### /subscriber/Int32

Response:
```json
{

}
```


#### /subscriber/Int32MultiArray

Response:
```json
{

}
```


#### /subscriber/Int64

Response:
```json
{

}
```


#### /subscriber/Int64MultiArray

Response:
```json
{

}
```


#### /subscriber/Int8

Response:
```json
{

}
```


#### /subscriber/Int8MultiArray

Response:
```json
{

}
```


#### /subscriber/MultiArrayDimension

Response:
```json
{

}
```


#### /subscriber/MultiArrayLayout

Response:
```json
{

}
```


#### /subscriber/String

Response:
```json
{

}
```


#### /subscriber/UInt16

Response:
```json
{

}
```


#### /subscriber/UInt16MultiArray

Response:
```json
{

}
```


#### /subscriber/UInt32

Response:
```json
{

}
```


#### /subscriber/UInt32MultiArray

Response:
```json
{

}
```


#### /subscriber/UInt64

Response:
```json
{

}
```


#### /subscriber/UInt64MultiArray

Response:
```json
{

}
```


#### /subscriber/UInt8

Response:
```json
{

}
```


#### /subscriber/UInt8MultiArray

Response:
```json
{

}
```

### sensor_msgs

#### /subscriber/BatteryState
Response:
```json
{

}
```

#### /subscriber/CameraInfo
Response:
```json
{

}
```

#### /subscriber/ChannelFloat32
Response:
```json
{

}
```

#### /subscriber/CompressedImage
Response:
```json
{

}
```

#### /subscriber/FluidPressure
Response:
```json
{

}
```

#### /subscriber/Illuminance
Response:
```json
{

}
```

#### /subscriber/Image
Response:
```json
{

}
```

#### /subscriber/Imu
Response:
```json
{

}
```

#### /subscriber/JointState
Response:
```json
{

}
```

#### /subscriber/Joy
Response:
```json
{

}
```

#### /subscriber/JoyFeedback
Response:
```json
{

}
```

#### /subscriber/JoyFeedbackArray
Response:
```json
{

}
```

#### /subscriber/LaserEcho
Response:
```json
{

}
```

#### /subscriber/LaserScan
Response:
```json
{

}
```

#### /subscriber/MagneticField
Response:
```json
{

}
```

#### /subscriber/MultiDOFJointState
Response:
```json
{

}
```

#### /subscriber/MultiEchoLaserScan
Response:
```json
{

}
```

#### /subscriber/NavSatFix
Response:
```json
{

}
```

#### /subscriber/NavSatStatus
Response:
```json
{

}
```

#### /subscriber/PointCloud
Response:
```json
{

}
```

#### /subscriber/PointCloud2
Response:
```json
{

}
```

#### /subscriber/PointField
Response:
```json
{

}
```

#### /subscriber/Range
Response:
```json
{

}
```

#### /subscriber/RegionOfInterest
Response:
```json
{

}
```

#### /subscriber/RelativeHumidity
Response:
```json
{

}
```

#### /subscriber/Temperature
Response:
```json
{

}
```

#### /subscriber/TimeReference
Response:
```json
{

}
```
