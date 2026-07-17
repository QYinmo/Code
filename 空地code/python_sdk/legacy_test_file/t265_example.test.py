#####################################################
##           librealsense T265 example             ##
#####################################################

import math
import time


# First import the library
import pyrealsense2.pyrealsense2 as rs

rs.log_to_file(rs.log_severity.debug, "rs.log")

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
# cfg.enable_stream(rs.stream.fisheye, 1)  # left
# cfg.enable_stream(rs.stream.fisheye, 2)  # right
# cfg.enable_stream(rs.stream.accel)
# cfg.enable_stream(rs.stream.gyro)
device = cfg.resolve(pipe).get_device()
print(f"Connected to {device}")
print(f"Device sensors: {device.query_sensors()}")
pose_sensor = device.first_pose_sensor()
print(f"Pose sensor: {pose_sensor}")
pose_sensor.set_option(rs.option.enable_mapping, 1)
pose_sensor.set_option(rs.option.enable_map_preservation, 1)
pose_sensor.set_option(rs.option.enable_relocalization, 1)
pose_sensor.set_option(rs.option.enable_pose_jumping, 1)
pose_sensor.set_option(rs.option.enable_dynamic_calibration, 1)
print(f"Pose sensor options:")
for opt in pose_sensor.get_supported_options():
    print(f"  {opt}: {pose_sensor.get_option(opt)}")


def quaternions_to_euler(w, x, y, z):
    # mathod 1
    # r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # p = math.asin(2 * (w * y - z * x))
    # y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    # mathod 2
    # r = math.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
    # p = -math.asin(2.0 * (x * z - w * y))
    # y = math.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)

    # mathod 3
    r = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    # Resolve the gimbal lock problem
    p = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    y = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # convert axis and radians to degrees
    r, p, y = -y / math.pi * 180, r / math.pi * 180, p / math.pi * 180
    return r, p, y


BACK = "\033[F"
start_time = time.perf_counter()


def print_pose(pose):
    if pose:
        data = pose.get_pose_data()
        f_num = pose.frame_number
        print(f"Frame #{f_num} fps: {f_num/(time.perf_counter()-start_time):.2f}    ")
        px, py, pz = data.translation.x, data.translation.y, data.translation.z  # meters
        print(f"Position      : {px:10.6f}, {py:10.6f}, {pz:10.6f}")
        vx, vy, vz = data.velocity.x, data.velocity.y, data.velocity.z  # meters/sec
        print(f"Velocity      : {vx:10.6f}, {vy:10.6f}, {vz:10.6f}")
        ax, ay, az = data.acceleration.x, data.acceleration.y, data.acceleration.z  # meters/sec^2
        print(f"Acceleration  : {ax:10.6f}, {ay:10.6f}, {az:10.6f}")
        avx, avy, avz = data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z  # radians/sec
        print(f"Angular vel   : {avx:10.6f}, {avy:10.6f}, {avz:10.6f}")
        aa_x, aa_y, aa_z = (
            data.angular_acceleration.x,
            data.angular_acceleration.y,
            data.angular_acceleration.z,
        )  # radians/sec^2
        print(f"Angular accel : {aa_x:10.6f}, {aa_y:10.6f}, {aa_z:10.6f}")
        rw, rx, ry, rz = data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z  # quaternions
        print(f"Rotation      : {rw:10.6f}, {rx:10.6f}, {ry:10.6f}, {rz:10.6f}")
        r, p, y = quaternions_to_euler(rw, rx, ry, rz)  # degrees
        print(f"Roll/Pitch/Yaw: {r:10.5f}, {p:10.5f}, {y:10.5f}")
        track_conf = data.tracker_confidence
        mapper_conf = data.mapper_confidence
        print(f"Tracker confidence: {track_conf}, Mapper confidence: {mapper_conf}")
        print(BACK * 9, end="")


if 0:  # query
    pipe.start(cfg)
    try:
        while True:
            frames = pipe.wait_for_frames()
            pose = frames.get_pose_frame()
            print_pose(pose)
    finally:
        pipe.stop()
else:  # async
    def callback(frame):
        pose = frame.as_pose_frame()
        print_pose(pose)

    pipe.start(cfg, callback)
    try:
        pass
        while True:
            time.sleep(1)
    finally:
        pipe.stop()
