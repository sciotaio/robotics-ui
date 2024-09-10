import std_msgs.msg
import sensor_msgs.msg

from enum import Enum


class MessageType(Enum):
    # std_msgs
    Bool = std_msgs.msg.Bool
    Byte = std_msgs.msg.Byte
    ByteMultiArray = std_msgs.msg.ByteMultiArray
    Char = std_msgs.msg.Char
    ColorRGBA = std_msgs.msg.ColorRGBA
    Empty = std_msgs.msg.Empty
    Float32 = std_msgs.msg.Float32
    Float32MultiArray = std_msgs.msg.Float32MultiArray
    Float64 = std_msgs.msg.Float64
    Float64MultiArray = std_msgs.msg.Float64MultiArray
    Header = std_msgs.msg.Header
    Int16 = std_msgs.msg.Int16
    Int16MultiArray = std_msgs.msg.Int16MultiArray
    Int32 = std_msgs.msg.Int32
    Int32MultiArray = std_msgs.msg.Int32MultiArray
    Int64 = std_msgs.msg.Int64
    Int64MultiArray = std_msgs.msg.Int64MultiArray
    Int8 = std_msgs.msg.Int8
    Int8MultiArray = std_msgs.msg.Int8MultiArray
    MultiArrayDimension = std_msgs.msg.MultiArrayDimension
    MultiArrayLayout = std_msgs.msg.MultiArrayLayout
    String = std_msgs.msg.String
    UInt16 = std_msgs.msg.UInt16
    UInt16MultiArray = std_msgs.msg.UInt16MultiArray
    UInt32 = std_msgs.msg.UInt32
    UInt32MultiArray = std_msgs.msg.UInt32MultiArray
    UInt64 = std_msgs.msg.UInt64
    UInt64MultiArray = std_msgs.msg.UInt64MultiArray
    UInt8 = std_msgs.msg.UInt8
    UInt8MultiArray = std_msgs.msg.UInt8MultiArray    
    
    # sensor_msgs
    BatteryState = sensor_msgs.msg.BatteryState
    CameraInfo = sensor_msgs.msg.CameraInfo
    ChannelFloat32 = sensor_msgs.msg.ChannelFloat32
    CompressedImage = sensor_msgs.msg.CompressedImage
    FluidPressure = sensor_msgs.msg.FluidPressure
    Illuminance = sensor_msgs.msg.Illuminance
    Image = sensor_msgs.msg.Image
    Imu = sensor_msgs.msg.Imu
    JointState = sensor_msgs.msg.JointState
    Joy = sensor_msgs.msg.Joy
    JoyFeedback = sensor_msgs.msg.JoyFeedback
    JoyFeedbackArray = sensor_msgs.msg.JoyFeedbackArray
    LaserEcho = sensor_msgs.msg.LaserEcho
    LaserScan = sensor_msgs.msg.LaserScan
    MagneticField = sensor_msgs.msg.MagneticField
    MultiDOFJointState = sensor_msgs.msg.MultiDOFJointState
    MultiEchoLaserScan = sensor_msgs.msg.MultiEchoLaserScan    
    NavSatFix = sensor_msgs.msg.NavSatFix
    NavSatStatus = sensor_msgs.msg.NavSatStatus
    PointCloud = sensor_msgs.msg.PointCloud
    PointCloud2 = sensor_msgs.msg.PointCloud2
    PointField = sensor_msgs.msg.PointField
    Range = sensor_msgs.msg.Range
    RegionOfInterest = sensor_msgs.msg.RegionOfInterest
    RelativeHumidity = sensor_msgs.msg.RelativeHumidity
    Temperature = sensor_msgs.msg.Temperature
    TimeReference = sensor_msgs.msg.TimeReference
    
    
    # Function to get the class from the enum value
def get_class_from_enum_value(enum_value):
    try:
        # Convert the string to the enum
        message_type = MessageType[enum_value]
        # Return the corresponding class
        return message_type.value
    except KeyError:
        # If the enum value is not found, return None
        return None