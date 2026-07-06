"""Load a MuJoCo model from a MuJoCo XML, URDF, or xacro file.

URDF/xacro files (e.g. from agx_arm_description) are preprocessed so MuJoCo
can import them directly:
- <visual> elements are stripped (DAE meshes are not loadable by MuJoCo and
  visuals are irrelevant for gravity computation),
- package://<pkg>/... mesh URIs in <collision> elements are rewritten to
  absolute paths (collision geoms are kept so configurations can be
  collision-checked during gravity calibration),
- a <mujoco><compiler .../></mujoco> extension element is injected.
"""

import os
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter
from typing import Callable, Optional

import mujoco
import numpy as np

_HALTON_PRIMES = (2, 3, 5, 7, 11, 13, 17, 19)


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
) -> mujoco.MjModel:
    """Load a MuJoCo model from a .xml (MJCF), .urdf, or .xacro file.

    With add_ground_plane, an infinite plane is added to the world at
    z=ground_height (robot base frame) so collision checking also rejects
    configurations that dip below base level (e.g. table-mounted arms). The
    base link is welded to the world, so it is automatically excluded from
    plane contacts.
    """
    extension = os.path.splitext(model_path)[1].lower()

    if extension == ".xml":
        return _compile_spec(model_path, add_ground_plane, ground_height)

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
        return _compile_spec(tmp_path, add_ground_plane, ground_height)
    finally:
        os.unlink(tmp_path)


def _compile_spec(
    path: str, add_ground_plane: bool, ground_height: float
) -> mujoco.MjModel:
    spec = mujoco.MjSpec.from_file(path)
    if add_ground_plane:
        geom = spec.worldbody.add_geom()
        geom.name = "ground"
        geom.type = mujoco.mjtGeom.mjGEOM_PLANE
        geom.size = [0.0, 0.0, 0.1]
        geom.pos = [0.0, 0.0, ground_height]
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
