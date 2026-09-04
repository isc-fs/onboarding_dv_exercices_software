"""
IFSSIM Types — API-compatible with the original FSDS types.
Drop-in replacement: from ifssim import CarControls, CarState, etc.
"""
try:
    import numpy as np
except ImportError:
    np = None


class ImageType:
    Scene = 0
    DepthPlanner = 1
    DepthPerspective = 2
    DepthVis = 3
    DisparityNormalized = 4
    Segmentation = 5
    SurfaceNormals = 6
    Infrared = 7


class Vector3r:
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0

    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val

    def __repr__(self):
        return f"Vector3r({self.x_val:.4f}, {self.y_val:.4f}, {self.z_val:.4f})"

    def __add__(self, other):
        return Vector3r(self.x_val + other.x_val, self.y_val + other.y_val, self.z_val + other.z_val)

    def __sub__(self, other):
        return Vector3r(self.x_val - other.x_val, self.y_val - other.y_val, self.z_val - other.z_val)

    def get_length(self):
        return (self.x_val**2 + self.y_val**2 + self.z_val**2) ** 0.5

    def to_numpy_array(self):
        return np.array([self.x_val, self.y_val, self.z_val], dtype=np.float32)


class Quaternionr:
    w_val = 1.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0

    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0, w_val=1.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val
        self.w_val = w_val

    def __repr__(self):
        return f"Quaternionr(w={self.w_val:.4f}, x={self.x_val:.4f}, y={self.y_val:.4f}, z={self.z_val:.4f})"

    def to_numpy_array(self):
        return np.array([self.x_val, self.y_val, self.z_val, self.w_val], dtype=np.float32)


class Pose:
    position = Vector3r()
    orientation = Quaternionr()

    def __init__(self, position_val=None, orientation_val=None):
        self.position = position_val if position_val else Vector3r()
        self.orientation = orientation_val if orientation_val else Quaternionr()


class GeoPoint:
    latitude = 0.0
    longitude = 0.0
    altitude = 0.0


class CarControls:
    throttle = 0.0
    steering = 0.0
    brake = 0.0
    handbrake = False
    is_manual_gear = False
    manual_gear = 0
    gear_immediate = True

    def __init__(self, throttle=0, steering=0, brake=0,
                 handbrake=False, is_manual_gear=False, manual_gear=0, gear_immediate=True):
        self.throttle = throttle
        self.steering = steering
        self.brake = brake
        self.handbrake = handbrake
        self.is_manual_gear = is_manual_gear
        self.manual_gear = manual_gear
        self.gear_immediate = gear_immediate

    def set_throttle(self, throttle_val, forward):
        if forward:
            self.is_manual_gear = False
            self.manual_gear = 0
            self.throttle = abs(throttle_val)
        else:
            self.is_manual_gear = False
            self.manual_gear = -1
            self.throttle = -abs(throttle_val)


class KinematicsState:
    position = Vector3r()
    orientation = Quaternionr()
    linear_velocity = Vector3r()
    angular_velocity = Vector3r()
    linear_acceleration = Vector3r()
    angular_acceleration = Vector3r()


class CarState:
    speed = 0.0
    gear = 0
    rpm = 0.0
    maxrpm = 0.0
    handbrake = False
    kinematics_estimated = KinematicsState()
    timestamp = 0


class LidarData:
    point_cloud = []
    time_stamp = 0
    pose = Pose()


class ImuData:
    time_stamp = 0
    orientation = Quaternionr()
    angular_velocity = Vector3r()
    linear_acceleration = Vector3r()


class GnssReport:
    geo_point = GeoPoint()
    eph = 0.0
    epv = 0.0
    velocity = Vector3r()
    time_utc = 0


class GpsData:
    time_stamp = 0
    gnss = GnssReport()


class GroundSpeedSensorData:
    time_stamp = 0
    linear_velocity = Vector3r()


class Point2D:
    x = 0.0
    y = 0.0


class ConePosition:
    """A cone on the track with ENU position and color."""
    x = 0.0
    y = 0.0
    color = 0  # 0=Yellow, 1=Blue, 2=OrangeLarge, 3=OrangeSmall, 4=Unknown

    YELLOW = 0
    BLUE = 1
    ORANGE_BIG = 2
    ORANGE_SMALL = 3
    UNKNOWN = 4


class RefereeState:
    doo_counter = 0          # Cone hits (Deletion of Opportunity)
    oc_counter = 0           # Off-track / out-of-bounds count
    laps = 0
    lap_times = []           # List of lap times in seconds
    required_laps = 0        # Laps needed to finish the event
    finished = False         # True when event is complete
    event = "unknown"        # Event type: trackdrive, acceleration, skidpad, autocross
    initial_position = Point2D()
    cones = []               # List of ConePosition objects
    cone_count = 0           # Total cone count


class ImageRequest:
    camera_name = '0'
    image_type = ImageType.Scene
    pixels_as_float = False
    compress = True

    def __init__(self, camera_name, image_type, pixels_as_float=False, compress=True):
        self.camera_name = camera_name
        self.image_type = image_type
        self.pixels_as_float = pixels_as_float
        self.compress = compress
