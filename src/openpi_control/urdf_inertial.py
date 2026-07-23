"""Effector-mass-merged URDF generation (ported from robot-test).

The gravity model must see the attached effector's inertia exactly once. Instead of
relying on every arm URDF shipping a massless end link plus native-side mass addition
(an implicit convention that silently double-counts when a URDF bakes the gripper in),
the end link's <inertial> block is *replaced* with the effector's mass model — or with
zero mass when no effector is attached — producing a merged URDF that is handed to the
native node. The base URDF files stay untouched.

Only the <inertial> section of the target link changes; the rest of the file is kept
byte-for-byte identical via regex search/replace.
"""

from __future__ import annotations

import json
import os
import re
import tempfile
from pathlib import Path

from .config import ResolvedArmAssets
from .exceptions import ConfigurationError

END_LINK_NAME = "end_link"

_ZERO_MASS_DATA: dict[str, object] = {
    "mass": 0.0,
    "center_of_mass": [0.0, 0.0, 0.0],
    "inertia": {"ixx": 0.0, "iyy": 0.0, "izz": 0.0, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0},
}


def prepare_merged_urdf(
    assets: ResolvedArmAssets, *, model: str, effector_model: str | None
) -> Path:
    """Create (or reuse) the effector-mass-merged URDF for a resolved arm.

    Reads the base URDF from ``assets.urdf`` and the effector mass model from
    ``<effector_model_config stem>_mass.json``; with no effector the end link
    gets zero added inertia. The merged file lands in a per-user temp directory
    under a deterministic name so both arms of a bimanual run share one file.

    Args:
        assets: Resolved packaged model files for the arm.
        model: Arm model name (used for the merged file name).
        effector_model: Effector model name, or None for a bare arm.

    Returns:
        Path to the merged URDF.

    Raises:
        ConfigurationError: If the effector mass model is missing or malformed,
            or the base URDF has no end link.
    """
    base_text = assets.urdf.read_text(encoding="utf-8")
    if effector_model:
        if assets.effector_model_config is None:
            raise ConfigurationError(
                f"effector {effector_model!r} requested but its model config was not resolved"
            )
        mass_path = assets.effector_model_config.with_name(f"{effector_model}_mass.json")
        if not mass_path.is_file():
            raise ConfigurationError(
                f"effector mass model not found: {mass_path} (required for the gravity model)"
            )
        try:
            mass_data = json.loads(mass_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as err:
            raise ConfigurationError(f"effector mass model {mass_path} is not valid JSON") from err
        for key in ("mass", "center_of_mass", "inertia"):
            if key not in mass_data:
                raise ConfigurationError(f"effector mass model {mass_path} is missing {key!r}")
        merged_name = f"{model}__{effector_model}.urdf"
    else:
        mass_data = _ZERO_MASS_DATA
        merged_name = f"{model}__no_effector.urdf"

    merged_text = update_link_inertial(base_text, END_LINK_NAME, mass_data)

    merged_dir = Path(tempfile.gettempdir()) / f"openpi-control-urdf-{os.getuid()}"
    merged_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    merged_path = merged_dir / merged_name

    # The merged URDF is deterministic for a given (arm, effector) pair. When the
    # file already holds the exact content (e.g. the second arm of a bimanual run
    # resolving right after the first) reuse it instead of rewriting: a rewrite
    # races a sibling pi_control_node parsing the same path.
    try:
        if merged_path.read_text(encoding="utf-8") == merged_text:
            return merged_path
    except FileNotFoundError:
        pass

    # Atomic replace: readers see either the previous complete file or the new
    # complete file, never a truncated one.
    temp_write_path = merged_path.with_suffix(".urdf.tmp")
    temp_write_path.write_text(merged_text, encoding="utf-8")
    os.replace(temp_write_path, merged_path)
    return merged_path


def update_link_inertial(urdf_text: str, link_name: str, mass_data: dict) -> str:
    """Replace the <inertial> block of one link, preserving the rest byte-for-byte.

    Args:
        urdf_text: Original URDF file content.
        link_name: Name of the link to update (e.g. "end_link").
        mass_data: Dict with "mass" (kg), "center_of_mass" ([x, y, z] m), and
            "inertia" (ixx/ixy/ixz/iyy/iyz/izz).

    Returns:
        Modified URDF text.

    Raises:
        ConfigurationError: If the link is not found in the URDF.
    """
    mass = float(mass_data["mass"])
    com = mass_data["center_of_mass"]
    inertia = mass_data["inertia"]

    link_pat = re.compile(
        rf"(<link\s+[^>]*\bname\s*=\s*\"{re.escape(link_name)}\"[^>]*>\s*)(.*?)(\s*</link>)",
        re.DOTALL,
    )
    match = link_pat.search(urdf_text)
    if match is None:
        raise ConfigurationError(f"link {link_name!r} not found in URDF")
    link_start, link_body, link_end = match.groups()

    inert_pat = re.compile(r"(^[ \t]*)<inertial>.*?</inertial>", re.DOTALL | re.MULTILINE)
    inert_match = inert_pat.search(link_body)
    if inert_match:
        indent = inert_match.group(1)
    else:
        first_line_match = re.match(r"^([ \t]*)", link_body)
        indent = (first_line_match.group(1) if first_line_match else "") + "  "

    new_inertial = _make_inertial_xml(indent, mass, com, inertia)
    if inert_match:
        new_body = inert_pat.sub(lambda _: new_inertial, link_body, count=1)
    else:
        if not link_body.endswith("\n"):
            link_body += "\n"
        new_body = link_body + new_inertial + "\n"

    return urdf_text[: match.start()] + link_start + new_body + link_end + urdf_text[match.end():]


def _fmt(value: float, precision: int = 12) -> str:
    """Format a float, avoiding negative-zero output."""
    text = f"{float(value):.{precision}g}"
    return "0" if text in ("-0", "-0.0") else text


def _make_inertial_xml(
    indent: str, mass: float, com: list[float], inertia: dict[str, float]
) -> str:
    """Construct the <inertial> XML block with the given indentation."""
    pad = indent
    pad2 = indent + "  "
    return (
        f"{pad}<inertial>\n"
        f'{pad2}<origin xyz="{_fmt(com[0], 8)} {_fmt(com[1], 8)} {_fmt(com[2], 8)}" rpy="0 0 0"/>\n'
        f'{pad2}<mass value="{_fmt(mass, 8)}"/>\n'
        f'{pad2}<inertia ixx="{_fmt(inertia.get("ixx", 0))}" ixy="{_fmt(inertia.get("ixy", 0))}" '
        f'ixz="{_fmt(inertia.get("ixz", 0))}" iyy="{_fmt(inertia.get("iyy", 0))}" '
        f'iyz="{_fmt(inertia.get("iyz", 0))}" izz="{_fmt(inertia.get("izz", 0))}"/>\n'
        f"{pad}</inertial>"
    )
