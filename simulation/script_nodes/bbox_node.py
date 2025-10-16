"""this file defines a script node to publish bounding boxes via ROS2"""

# ==================== Imports ====================
import math
import rclpy
import asyncio
import omni.usd
from pxr import UsdGeom
from std_msgs.msg import Header
from typing import Tuple, List
from omni import syntheticdata as sd
from omni.isaac.core.prims import XFormPrim
from rclpy.qos import qos_profile_sensor_data
from isaac_ros2_messages.msg import FrameBboxes, Bbox
from omni.kit.viewport.utility import get_active_viewport


# ==================== Helper - Quaternion to Euler Conversion ====================
def euler_from_quaternion(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert quaternion (w, x, y, z) to Euler angles in degrees using XYZ rotation order, and z up/down correction"""
    
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z

    m11 = 1 - 2 * (yy + zz)
    m12 = 2 * (xy - wz)
    m13 = 2 * (xz + wy)
    m21 = 2 * (xy + wz)
    m22 = 1 - 2 * (xx + zz)
    m23 = 2 * (yz - wx)
    m31 = 2 * (xz - wy)
    m32 = 2 * (yz + wx)
    m33 = 1 - 2 * (xx + yy)

    m13_clamped = max(min(m13, 1.0), -1.0)
    y = math.asin(m13_clamped)

    if abs(m13_clamped) < 0.9999999:
        x = math.atan2(-m23, m33)
        z = math.atan2(-m12, m11)
    else:
        x = math.atan2(m32, m22)
        z = 0

    return math.degrees(x), math.degrees(y), math.degrees(z)


# ==================== Helper - Extract Euler from Prim ====================
def get_euler_from_prim(prim) -> Tuple[float, float, float]:
    """Extract roll, pitch, yaw from a prim's orientation quaternion, in degrees."""
    xformable = UsdGeom.Xformable(prim)
    ops = xformable.GetOrderedXformOps()

    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            quat = op.Get()
            w = quat.GetReal()
            x, y, z = quat.GetImaginary()
            return tuple(round(angle, 1) for angle in euler_from_quaternion(w, x, y, z))

    return (0.0, 0.0, 0.0)


# ==================== Helper - Async Frame Bboxes Publisher ====================
async def publish_frame_bboxes(db) -> None:
    """publish all bounding boxes in the current frame"""

    now = db.internal_state.node.get_clock().now()
    if db.internal_state.last_publish_time is None:
        db.internal_state.last_publish_time = now

    elapsed = (now - db.internal_state.last_publish_time).nanoseconds / 1e9
    if elapsed < db.internal_state.publish_period:
        return

    msg = FrameBboxes()
    msg.header = Header()
    msg.header.stamp = now.to_msg()
    msg.header.frame_id = str(db.internal_state.frame_id)

    msg.bboxes = build_bboxes(db)

    db.internal_state.publisher.publish(msg)
    db.internal_state.last_publish_time = now
    db.internal_state.frame_id += 1

    rclpy.spin_once(db.internal_state.node, timeout_sec=0.01)


# ==================== Helper - Bounding Box Builder ====================
def build_bboxes(db) -> List[Bbox]:
    """Build list of all bounding boxes data in the current frame"""

    tight_data = sd.sensors.get_bounding_box_2d_tight(db.internal_state.vp_api)
    loose_data = sd.sensors.get_bounding_box_2d_loose(db.internal_state.vp_api)

    prims_in_tight = {obj[1]: obj for obj in tight_data}
    prims_in_loose = {obj[1]: obj for obj in loose_data}
    all_paths = db.internal_state.bboxes_paths

    stage = omni.usd.get_context().get_stage()
    bboxes = []

    for path in all_paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue

        bbox_msg = Bbox()
        bbox_msg.target_name = prim.GetName()

        # Visibility flags
        in_tight = path in prims_in_tight
        in_loose = path in prims_in_loose
        bbox_msg.in_frame = in_tight or in_loose
        bbox_msg.is_visible = in_tight

        # Bounding box coordinates
        bbox_data = prims_in_loose.get(path)
        if bbox_data:
            bbox_msg.x1 = int(bbox_data[6])
            bbox_msg.y1 = int(bbox_data[7])
            bbox_msg.x2 = int(bbox_data[8])
            bbox_msg.y2 = int(bbox_data[9])
        else:
            bbox_msg.x1 = bbox_msg.y1 = bbox_msg.x2 = bbox_msg.y2 = -1

        # Geolocation
        bbox_msg.lat = float(prim.GetAttribute("cesium:anchor:latitude").Get())
        bbox_msg.lon = float(prim.GetAttribute("cesium:anchor:longitude").Get())
        bbox_msg.alt = float(prim.GetAttribute("cesium:anchor:height").Get())

        # World pose
        target = XFormPrim(path)
        camera = XFormPrim(db.internal_state.camera_prim.GetPath().pathString)

        target_pos, target_rot = target.get_world_pose()
        camera_pos, _ = camera.get_world_pose()

        # Orientation
        # the local orientation of the objects in isaac sim, the orientation the user defines in the gui
        bbox_msg.roll, bbox_msg.pitch, bbox_msg.yaw = get_euler_from_prim(prim)

        # Relative position
        # the distance difference between the camera and the object in the world in local xyz coordinates, in meters
        bbox_msg.distance_x = float(target_pos[0] - camera_pos[0])
        bbox_msg.distance_y = float(target_pos[1] - camera_pos[1])
        bbox_msg.distance_z = float(target_pos[2] - camera_pos[2])

        bboxes.append(bbox_msg)

    return bboxes


# ==================== isaac sim funcs ====================
# ==================== Setup
def setup(db) -> None:
    """setup the script node internal state"""

    db.internal_state.frame_id = 0
    db.internal_state.last_publish_time = None
    db.internal_state.publish_period = 1.0 / db.inputs.hz
    db.internal_state.topic_name = db.inputs.topic_name

    try:
        rclpy.init()
    except Exception:
        pass

    db.internal_state.node = rclpy.create_node('bbox_publisher')
    try:
        db.internal_state.node.declare_parameter('use_sim_time', True)
    except rclpy.exceptions.ParameterAlreadyDeclaredException:
        pass

    db.internal_state.publisher = db.internal_state.node.create_publisher(
        FrameBboxes,
        db.internal_state.topic_name,
        qos_profile_sensor_data
    )

    db.internal_state.vp_api = get_active_viewport()
    camera_path = db.internal_state.vp_api.camera_path.pathString
    stage = omni.usd.get_context().get_stage()
    db.internal_state.camera_prim = stage.GetPrimAtPath(camera_path)
    db.internal_state.bbox_sensor_tight = sd.sensors.create_or_retrieve_sensor(
        db.internal_state.vp_api,
        sd._syntheticdata.SensorType.BoundingBox2DTight
    )
    db.internal_state.bbox_sensor_loose = sd.sensors.create_or_retrieve_sensor(
        db.internal_state.vp_api,
        sd._syntheticdata.SensorType.BoundingBox2DLoose
    )

    bbox_root = stage.GetPrimAtPath("/bboxes")
    db.internal_state.bboxes_paths = []

    if bbox_root.IsValid():
        for prim in bbox_root.GetChildren():
            if prim.IsValid():
                db.internal_state.bboxes_paths.append(prim.GetPath().pathString)
    else:
        print("[bbox_script_node] Warning: /bboxes prim not found.")


# ==================== Compute
def compute(db) -> bool:
    """compute the script node logic each frame"""

    asyncio.ensure_future(publish_frame_bboxes(db))
    return True


# ==================== Cleanup
def cleanup(db) -> None:
    """cleanup the script node internal state"""
    
    if db.internal_state.node is not None:
        db.internal_state.node.destroy_node()
        rclpy.shutdown()
