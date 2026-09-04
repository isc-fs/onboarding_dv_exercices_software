"""
IFSSIM Python Client — API-compatible with the original FSDS FSDSClient.

Usage:
    from ifssim import IFSSIMClient, CarControls, ImageType
    client = IFSSIMClient()
    client.confirmConnection()
    client.enableApiControl(True)
    state = client.getCarState()
    print(state.speed)
"""

import socket
import struct
import time
import json
from .types import *


class IFSSIMClient:
    """
    Drop-in replacement for fsds.FSDSClient.
    Connects to the IFSSIM TCP RPC server on port 41451.
    """

    def __init__(self, ip="127.0.0.1", port=41451, timeout_value=5):
        self.ip = ip
        self.port = port
        self.timeout = timeout_value

    # --- Connection ---

    def _text_cmd(self, cmd):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect((self.ip, self.port))
        s.sendall((cmd + "\n").encode())
        time.sleep(0.05)
        data = s.recv(65536).decode().strip()
        s.close()
        return data

    def _binary_cmd(self, cmd):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect((self.ip, self.port))
        s.sendall((cmd + "\n").encode())

        # Read header line
        header = b""
        while True:
            byte = s.recv(1)
            if not byte or byte == b"\n":
                break
            header += byte
        header = header.decode()

        colon = header.find(":")
        if colon < 0:
            s.close()
            return header, b""

        prefix = header[:colon]
        count = int(header[colon + 1:])

        # PTS handler (`getLidarDataBinary` reply) was removed in #322
        # along with the rest of the TCP-LiDAR path on the plugin side.
        # `getLidarData()` below now raises NotImplementedError pointing
        # callers at the ROS topic /lidar/Lidar1, which is what every
        # real consumer uses.
        if prefix == "IMG":
            byte_size = count
        else:
            s.close()
            return header, b""

        data = b""
        while len(data) < byte_size:
            chunk = s.recv(byte_size - len(data))
            if not chunk:
                break
            data += chunk
        s.close()
        return header, data

    def _parse(self, json_str, key):
        try:
            search = f'"{key}":'
            pos = json_str.find(search)
            if pos < 0:
                return None
            pos += len(search)
            end = json_str.find(",", pos)
            if end < 0:
                end = json_str.find("}", pos)
            val = json_str[pos:end].strip().strip('"')
            try:
                return float(val)
            except:
                return val
        except:
            return None

    def ping(self):
        return self._text_cmd("ping") == "true"

    def reset(self):
        self._text_cmd("reset")

    def confirmConnection(self):
        if not self.ping():
            raise ConnectionError(f"Cannot connect to IFSSIM at {self.ip}:{self.port}")

    def enableApiControl(self, is_enabled, vehicle_name='FSCar'):
        self._text_cmd("enableApiControl")

    def isApiControlEnabled(self, vehicle_name='FSCar'):
        return self._text_cmd("isApiControlEnabled") == "true"

    def getSettingsString(self):
        return self._text_cmd("getSettingsString")

    # --- Vehicle Control ---

    def setCarControls(self, controls, vehicle_name='FSCar'):
        cmd = f"setCarControls {controls.throttle} {controls.steering} {controls.brake}"
        self._text_cmd(cmd)

    def getCarControls(self, vehicle_name='FSCar'):
        resp = self._text_cmd("getCarControls")
        controls = CarControls()
        controls.throttle = self._parse(resp, "throttle") or 0.0
        controls.steering = self._parse(resp, "steering") or 0.0
        controls.brake = self._parse(resp, "brake") or 0.0
        controls.handbrake = str(self._parse(resp, "handbrake")).lower() == "true"
        controls.is_manual_gear = str(self._parse(resp, "is_manual_gear")).lower() == "true"
        controls.manual_gear = int(self._parse(resp, "manual_gear") or 0)
        controls.gear_immediate = str(self._parse(resp, "gear_immediate")).lower() == "true"
        return controls

    def getCarState(self, vehicle_name='FSCar'):
        resp = self._text_cmd("getCarState")
        state = CarState()
        state.speed = self._parse(resp, "speed") or 0.0
        state.gear = int(self._parse(resp, "gear") or 0)
        state.rpm = self._parse(resp, "rpm") or 0.0
        state.maxrpm = self._parse(resp, "maxrpm") or 0.0

        kin = KinematicsState()
        kin.position = Vector3r(
            self._parse(resp, "x") or 0.0,
            self._parse(resp, "y") or 0.0,
            self._parse(resp, "z") or 0.0
        )
        kin.linear_velocity = Vector3r(
            self._parse(resp, "vx") or 0.0,
            self._parse(resp, "vy") or 0.0,
            self._parse(resp, "vz") or 0.0
        )
        kin.orientation = Quaternionr(
            self._parse(resp, "qx") or 0.0,
            self._parse(resp, "qy") or 0.0,
            self._parse(resp, "qz") or 0.0,
            self._parse(resp, "qw") or 1.0
        )
        state.kinematics_estimated = kin
        state.timestamp = int(time.time() * 1e9)
        return state

    # --- Sensors ---

    def getGpsData(self, gps_name='', vehicle_name='FSCar'):
        resp = self._text_cmd("getGpsData")
        data = GpsData()
        data.time_stamp = int(time.time() * 1e9)
        data.gnss = GnssReport()
        data.gnss.geo_point = GeoPoint()
        data.gnss.geo_point.latitude = self._parse(resp, "lat") or 0.0
        data.gnss.geo_point.longitude = self._parse(resp, "lon") or 0.0
        data.gnss.geo_point.altitude = self._parse(resp, "alt") or 0.0
        return data

    def getImuData(self, imu_name='', vehicle_name='FSCar'):
        resp = self._text_cmd("getImuData")
        data = ImuData()
        data.time_stamp = int(time.time() * 1e9)
        data.linear_acceleration = Vector3r(
            self._parse(resp, "ax") or 0.0,
            self._parse(resp, "ay") or 0.0,
            self._parse(resp, "az") or 0.0
        )
        data.angular_velocity = Vector3r(
            self._parse(resp, "gx") or 0.0,
            self._parse(resp, "gy") or 0.0,
            self._parse(resp, "gz") or 0.0
        )
        data.orientation = Quaternionr(
            self._parse(resp, "qx") or 0.0,
            self._parse(resp, "qy") or 0.0,
            self._parse(resp, "qz") or 0.0,
            self._parse(resp, "qw") or 1.0
        )
        return data

    def getGroundSpeedSensorData(self, vehicle_name='FSCar'):
        resp = self._text_cmd("getGroundSpeedSensorData")
        data = GroundSpeedSensorData()
        data.time_stamp = int(time.time() * 1e9)
        data.linear_velocity = Vector3r(
            self._parse(resp, "vx") or 0.0,
            self._parse(resp, "vy") or 0.0,
            self._parse(resp, "vz") or 0.0
        )
        return data

    def getLidarData(self, lidar_name='', vehicle_name='FSCar'):
        # Removed in #322. The `getLidarDataBinary` RPC reply was the
        # request/response counterpart to the also-deleted `streamLidar`
        # TCP push; both paths existed pre-#321 to bypass macOS Docker
        # Desktop's TCP loopback throughput cap, and were retired once
        # FSDSUdpBroadcaster proved reliable on every supported host.
        # Real consumers (the ROS bridge, autonomy stack, Lichtblick)
        # subscribe to /lidar/Lidar1 instead.
        raise NotImplementedError(
            "getLidarData was removed in #322 — LiDAR is now UDP-only via "
            "FSDSUdpBroadcaster. Subscribe to /lidar/Lidar1 from the ROS "
            "bridge, or replicate the UDP receiver in udp_receiver.h."
        )

    # --- Camera (removed in perf/strip-cameras) ---
    # Camera sensors were stripped from the simulator — the real IFS-08
    # has no cameras and the autonomy pipeline never consumed any
    # /camera/* topics. simGetImage / simGetImages now raise so legacy
    # client scripts surface the change instead of silently getting
    # zero-byte payloads.

    def simGetImage(self, camera_name, image_type, vehicle_name='FSCar'):
        raise NotImplementedError(
            "Cameras were removed from IFSSIM in perf/strip-cameras. "
            "If you need RGB capture, pin to a pre-strip commit."
        )

    def simGetImages(self, requests, vehicle_name='FSCar'):
        raise NotImplementedError(
            "Cameras were removed from IFSSIM in perf/strip-cameras. "
            "If you need RGB capture, pin to a pre-strip commit."
        )

    # --- Ground Truth ---

    def simGetGroundTruthKinematics(self, vehicle_name='FSCar'):
        resp = self._text_cmd("simGetGroundTruthKinematics")
        kin = KinematicsState()
        kin.position = Vector3r(
            self._parse(resp, "px") or 0.0,
            self._parse(resp, "py") or 0.0,
            self._parse(resp, "pz") or 0.0
        )
        kin.linear_velocity = Vector3r(
            self._parse(resp, "vx") or 0.0,
            self._parse(resp, "vy") or 0.0,
            self._parse(resp, "vz") or 0.0
        )
        kin.linear_acceleration = Vector3r(
            self._parse(resp, "ax") or 0.0,
            self._parse(resp, "ay") or 0.0,
            self._parse(resp, "az") or 0.0
        )
        kin.angular_velocity = Vector3r(
            self._parse(resp, "wx") or 0.0,
            self._parse(resp, "wy") or 0.0,
            self._parse(resp, "wz") or 0.0
        )
        kin.orientation = Quaternionr(
            self._parse(resp, "qx") or 0.0,
            self._parse(resp, "qy") or 0.0,
            self._parse(resp, "qz") or 0.0,
            self._parse(resp, "qw") or 1.0
        )
        return kin

    def simGetVehiclePose(self, vehicle_name='FSCar'):
        resp = self._text_cmd("simGetVehiclePose")
        pose = Pose()
        pose.position = Vector3r(
            self._parse(resp, "x") or 0.0,
            self._parse(resp, "y") or 0.0,
            self._parse(resp, "z") or 0.0
        )
        pose.orientation = Quaternionr(
            self._parse(resp, "qx") or 0.0,
            self._parse(resp, "qy") or 0.0,
            self._parse(resp, "qz") or 0.0,
            self._parse(resp, "qw") or 1.0
        )
        return pose

    def simSetVehiclePose(self, pose, ignore_collision=True, vehicle_name='FSCar'):
        cmd = f"simSetVehiclePose {pose.position.x_val} {pose.position.y_val} {pose.position.z_val}"
        self._text_cmd(cmd)

    # --- Competition ---

    def getRefereeState(self):
        resp = self._text_cmd("getRefereeState")
        state = RefereeState()
        state.doo_counter = int(self._parse(resp, "doo_counter") or 0)
        state.oc_counter = int(self._parse(resp, "oc_counter") or 0)
        state.laps = int(self._parse(resp, "laps") or 0)
        state.cone_count = int(self._parse(resp, "cones") or 0)

        # Parse full JSON response
        state.lap_times = []
        import json
        try:
            data = json.loads(resp)
            state.lap_times = data.get("lap_times", [])
            state.required_laps = data.get("required_laps", 0)
            state.finished = data.get("finished", False)
            state.event = data.get("event", "unknown")
            # Parse cone positions
            state.cones = []
            for c in data.get("cone_positions", []):
                cp = ConePosition()
                cp.x = c.get("x", 0.0)
                cp.y = c.get("y", 0.0)
                cp.color = c.get("color", 4)
                state.cones.append(cp)
        except (json.JSONDecodeError, ValueError):
            pass

        return state

    # --- Simulation Control ---

    def simPause(self, pause=True):
        self._text_cmd("simPause" if pause else "simResume")

    def simResume(self):
        self._text_cmd("simResume")

    def simIsPaused(self):
        return self._text_cmd("simIsPaused") == "true"

    def simContinueForTime(self, seconds):
        self._text_cmd(f"simContinueForTime {seconds}")


# Alias for drop-in compatibility with FSDS
FSDSClient = IFSSIMClient
