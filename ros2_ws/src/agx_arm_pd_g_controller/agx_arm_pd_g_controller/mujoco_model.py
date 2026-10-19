"""Load a MuJoCo model from a MuJoCo XML, URDF, or xacro file.

URDF/xacro files (e.g. from agx_arm_description) are preprocessed so MuJoCo
can import them directly:
- <visual> elements are stripped (DAE meshes are not loadable by MuJoCo and
  visuals are irrelevant for gravity computation),
- package://<pkg>/... mesh URIs in <collision> elements are rewritten to
  absolute paths (collision geoms are kept so configurations can be
  collision-checked),
- a <mujoco><compiler .../></mujoco> extension element is injected.
"""

import os
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter
from typing import Callable, Dict, Optional, Sequence

import mujoco
import numpy as np

_HALTON_PRIMES = (2, 3, 5, 7, 11, 13, 17, 19)

# Wall keys accepted by the `walls` dict argument of load_mujoco_model /
# _compile_spec, and the axis/side each corresponds to.
_WALL_SIDES = {
    "x_pos": ("x", 1.0),
    "x_neg": ("x", -1.0),
    "y_pos": ("y", 1.0),
    "y_neg": ("y", -1.0),
}


def _default_package_resolver(package_name: str) -> str:
    from ament_index_python.packages import get_package_share_directory

    return get_package_share_directory(package_name)


def _resolve_package_uris(root: ET.Element, package_resolver: Callable[[str], str]) -> None:
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith("package://"):
            continue
        package_name, _, relative = filename[len("package://"):].partition("/")
        mesh.set("filename", os.path.join(package_resolver(package_name), relative))


def preprocess_urdf(
    urdf_string: str,
    package_resolver: Optional[Callable[[str], str]] = None,
) -> str:
    """Return a MuJoCo-importable URDF string."""
    package_resolver = package_resolver or _default_package_resolver
    root = ET.fromstring(urdf_string)

    for link in root.iter("link"):
        for visual in link.findall("visual"):
            link.remove(visual)

    _resolve_package_uris(root, package_resolver)

    if root.find("mujoco") is None:
        mujoco_tag = ET.SubElement(root, "mujoco")
        # boundmass/boundinertia give massless virtual links (e.g. the piper
        # gripper's prismatic 'gripper_link' frame) a negligible mass instead
        # of failing MuJoCo's moving-body mass check. fusestatic must stay off
        # or the base link is merged into the world body, which disables
        # parent-child collision filtering between the base and link1.
        ET.SubElement(
            mujoco_tag,
            "compiler",
            {
                "balanceinertia": "true",
                "strippath": "false",
                "discardvisual": "true",
                "fusestatic": "false",
                "boundmass": "0.001",
                "boundinertia": "1e-08",
            },
        )

    return ET.tostring(root, encoding="unicode")


def load_mujoco_model(
    model_path: str,
    package_resolver: Optional[Callable[[str], str]] = None,
    add_ground_plane: bool = False,
    ground_height: float = 0.0,
    walls: Optional[Dict[str, Optional[float]]] = None,
    weld_joints_except: Optional[Sequence[str]] = None,
) -> mujoco.MjModel:
    """Load a MuJoCo model from a .xml (MJCF), .urdf, or .xacro file.

    With add_ground_plane, an infinite plane is added to the world at
    z=ground_height (robot base frame) so collision checking also rejects
    configurations that dip below base level (e.g. table-mounted arms). The
    base link is welded to the world, so it is automatically excluded from
    plane contacts.

    walls optionally adds invisible collision walls in the base frame: a dict
    with any of the keys "x_pos", "x_neg", "y_pos", "y_neg" mapped to the wall's
    coordinate along that axis (None or omitted disables that side). Sides are
    independent, so the box need not be symmetric. Like the ground plane, the
    base link is welded to the world and excluded from wall contacts.

    weld_joints_except, if given, rigidly welds every joint whose name is not
    in the sequence (e.g. gripper fingers when only the arm joints are being
    controlled/observed) to its parent body, at that joint's reference (zero)
    position. See _weld_joints_except.
    """
    extension = os.path.splitext(model_path)[1].lower()

    if extension == ".xml":
        return _compile_spec(model_path, add_ground_plane, ground_height, walls, weld_joints_except)

    if extension == ".xacro":
        import xacro

        urdf_string = xacro.process_file(model_path).toxml()
    elif extension == ".urdf":
        with open(model_path, "r") as f:
            urdf_string = f.read()
    else:
        raise ValueError(
            f"Unsupported model file extension '{extension}' for '{model_path}'. "
            "Expected .xml (MJCF), .urdf, or .xacro."
        )

    processed = preprocess_urdf(urdf_string, package_resolver)

    # MuJoCo dispatches URDF vs MJCF parsing on the file extension.
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        f.write(processed)
        tmp_path = f.name
    try:
        return _compile_spec(tmp_path, add_ground_plane, ground_height, walls, weld_joints_except)
    finally:
        os.unlink(tmp_path)


def _weld_joints_except(spec, keep_joint_names: Sequence[str]) -> None:
    """Rigidly weld every joint not in keep_joint_names to its parent body.

    Removes each such joint from the spec entirely (which fixes it at its
    reference/zero position - e.g. a gripper's fingers get welded closed
    rather than at whatever position they last held), along with anything
    that would otherwise dangle-reference it: equality constraints,
    actuators, and sensors.
    """
    keep = set(keep_joint_names)
    to_weld = [joint for joint in spec.joints if joint.name not in keep]
    if not to_weld:
        return
    weld_names = {joint.name for joint in to_weld}

    for equality in list(spec.equalities):
        if equality.name1 in weld_names or equality.name2 in weld_names:
            spec.delete(equality)
    for actuator in list(spec.actuators):
        if actuator.target in weld_names:
            spec.delete(actuator)
    for sensor in list(spec.sensors):
        if getattr(sensor, "objname", None) in weld_names:
            spec.delete(sensor)
    # Keyframes record qpos/qvel for every DOF in the model; welding changes
    # that count, so any saved keyframe would otherwise fail to compile.
    for key in list(spec.keys):
        spec.delete(key)
    for joint in to_weld:
        spec.delete(joint)


def _add_wall(
    spec,
    key: str,
    distance: float,
    thickness: float = 0.02,
    half_extent: float = 5.0,
    z_center: float = 1.0,
    z_half_extent: float = 2.0,
) -> None:
    """Add a thin box geom acting as an invisible wall perpendicular to key's axis.

    The box's inner face sits exactly at `distance` (in the base frame) along
    the axis, and extends further away from the origin from there, so any
    configuration reaching past `distance` registers a contact.
    """
    axis, sign = _WALL_SIDES[key]
    center = distance + sign * thickness / 2.0
    geom = spec.worldbody.add_geom()
    geom.name = f"wall_{key}"
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    if axis == "x":
        geom.size = [thickness / 2.0, half_extent, z_half_extent]
        geom.pos = [center, 0.0, z_center]
    else:
        geom.size = [half_extent, thickness / 2.0, z_half_extent]
        geom.pos = [0.0, center, z_center]


def _compile_spec(
    path: str,
    add_ground_plane: bool,
    ground_height: float,
    walls: Optional[Dict[str, Optional[float]]] = None,
    weld_joints_except: Optional[Sequence[str]] = None,
) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(path)
    if add_ground_plane:
        geom = spec.worldbody.add_geom()
        geom.name = "ground"
        geom.type = mujoco.mjtGeom.mjGEOM_PLANE
        geom.size = [0.0, 0.0, 0.1]
        geom.pos = [0.0, 0.0, ground_height]
    for key, distance in (walls or {}).items():
        if distance is None:
            continue
        if key not in _WALL_SIDES:
            raise ValueError(f"Unknown wall key '{key}'. Expected one of {list(_WALL_SIDES)}.")
        _add_wall(spec, key, distance)
    if weld_joints_except is not None:
        _weld_joints_except(spec, weld_joints_except)
    return spec.compile()


class HaltonSampler:
    """Halton sequence sampler for joint configurations."""

    def __init__(self, limits_min, limits_max):
        self.center = 0.5 * (np.array(limits_max) + np.array(limits_min))
        self.radius = 0.5 * (np.array(limits_max) - np.array(limits_min))
        self.primes = _HALTON_PRIMES[: len(self.center)]
        self.index = 0

    def sample(self):
        result = np.array(
            [
                self.center[i] + self.radius[i] * (2 * mujoco.mju_Halton(self.index, p) - 1)
                for i, p in enumerate(self.primes)
            ]
        )
        self.index += 1
        return result


def get_colliding_body_pairs(model, data, exclude_adjacent: bool = True) -> Counter:
    """Contact counts between body pairs at the configuration set in data.

    With exclude_adjacent (default), contacts between a body and its kinematic
    parent are ignored: adjacent links overlap at the joints in mesh-based
    models, and MuJoCo's own parent filtering does not cover bodies welded to
    the world (e.g. the arm base).
    """
    mujoco.mj_forward(model, data)
    contacts = Counter()
    for contact in data.contact:
        body1_id = model.geom_bodyid[contact.geom1]
        body2_id = model.geom_bodyid[contact.geom2]
        if exclude_adjacent and (
            model.body_parentid[body1_id] == body2_id
            or model.body_parentid[body2_id] == body1_id
        ):
            continue
        body1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body1_id)
        body2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body2_id)
        contacts[(body1, body2)] += 1
    return contacts
