import asyncio
import omni.kit.raycast.query
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.rotations import quat_to_rot_matrix

_raycast_interface = omni.kit.raycast.query.acquire_raycast_query_interface()


async def get_range_from_ray(db, ray) -> None:
    result_future = asyncio.Future()

    def raycast_callback(ray, result):
        result_future.set_result(result)

    _raycast_interface.submit_raycast_query(ray, raycast_callback)
    raycast_result = await result_future

    if not raycast_result.valid:
        db.internal_state.range = db.internal_state.max_range
        return

    db.internal_state.range = raycast_result.hit_t

    if db.internal_state.range < db.internal_state.min_range:
        db.internal_state.range = db.internal_state.min_range
    elif db.internal_state.range > db.internal_state.max_range:
        db.internal_state.range = db.internal_state.max_range


def setup(db: og.Database) -> None:
    db.internal_state.__dict__["range"] = 1
    db.internal_state.__dict__["min_range"] = db.inputs.min_range
    db.internal_state.__dict__["max_range"] = db.inputs.max_range
    db.internal_state.__dict__["camera_xform"] = XFormPrim(db.inputs.camera_xform_path)


def cleanup(db: og.Database) -> None:
    db.internal_state.range = 0
    db.internal_state.min_range = 0
    db.internal_state.max_range = 0
    db.internal_state.camera_xform = None


def compute(db: og.Database) -> bool:
    camera_position, camera_rotation = db.internal_state.camera_xform.get_world_pose()
    rotation_matrix = quat_to_rot_matrix(camera_rotation)
    ray_direction = rotation_matrix @ (0, 0, -1)

    ray_query = omni.kit.raycast.query.Ray(camera_position.tolist(), ray_direction.tolist())
    asyncio.ensure_future(get_range_from_ray(db, ray_query))
    db.outputs.range = db.internal_state.range

    return True
