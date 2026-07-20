"""Effector-mass-merged URDF generation."""

from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

from openpi_control.config import resolve_model_assets
from openpi_control.exceptions import ConfigurationError
from openpi_control.urdf_inertial import (
    END_LINK_NAME,
    prepare_merged_urdf,
    update_link_inertial,
)

_SAMPLE_URDF = """<robot name="sample">
    <link name="base_link">
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <mass value="1.0"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
        </inertial>
    </link>
    <link name="end_link">
        <inertial>
            <origin xyz="0.01 0.02 0.03" rpy="0 0 0"/>
            <mass value="0.581000"/>
            <inertia ixx="0.00029" ixy="0" ixz="0" iyy="0.00028" iyz="-0.00001" izz="0.00045"/>
        </inertial>
        <visual>
            <geometry><box size="0.1 0.1 0.1"/></geometry>
        </visual>
    </link>
</robot>
"""

_MASS_DATA = {
    "mass": 0.123,
    "center_of_mass": [0.4, 0.5, 0.6],
    "inertia": {"ixx": 0.001, "iyy": 0.002, "izz": 0.003, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0},
}


def test_update_replaces_only_the_target_link_inertial() -> None:
    merged = update_link_inertial(_SAMPLE_URDF, END_LINK_NAME, _MASS_DATA)

    assert '<mass value="0.123"/>' in merged
    assert '<origin xyz="0.4 0.5 0.6" rpy="0 0 0"/>' in merged
    assert '<mass value="0.581000"/>' not in merged
    # base_link and everything outside end_link's inertial stay byte-identical.
    assert '<mass value="1.0"/>' in merged
    assert "<visual>" in merged
    without_inertials = re.sub(r"<inertial>.*?</inertial>", "", _SAMPLE_URDF, flags=re.DOTALL)
    merged_without_inertials = re.sub(r"<inertial>.*?</inertial>", "", merged, flags=re.DOTALL)
    assert merged_without_inertials == without_inertials


def test_update_rejects_missing_link() -> None:
    with pytest.raises(ConfigurationError):
        update_link_inertial(_SAMPLE_URDF, "nonexistent_link", _MASS_DATA)


def test_arx_x5_merge_replaces_baked_in_gripper_mass_exactly_once() -> None:
    # ARX_X5.urdf bakes the 0.581 kg gripper into end_link; the merge must
    # replace (not add to) it with E_ARX_mass.json so gravity compensation
    # counts the gripper exactly once.
    assets = resolve_model_assets("ARX_X5", effector_model="E_ARX")
    merged_path = prepare_merged_urdf(assets, model="ARX_X5", effector_model="E_ARX")

    merged = merged_path.read_text()
    assert assets.effector_model_config is not None
    mass_json = json.loads(
        assets.effector_model_config.with_name("E_ARX_mass.json").read_text()
    )
    assert merged.count(f'<mass value="{mass_json["mass"]:g}"/>') == 1
    # The base URDF must stay untouched.
    assert '<mass value="0.581000"/>' in assets.urdf.read_text()


def test_merge_without_effector_zeroes_the_end_link_inertia() -> None:
    assets = resolve_model_assets("ARX_X5")
    merged_path = prepare_merged_urdf(assets, model="ARX_X5", effector_model=None)

    merged = merged_path.read_text()
    assert merged_path.name == "ARX_X5__no_effector.urdf"
    assert '<mass value="0"/>' in merged
    assert '<mass value="0.581000"/>' not in merged


def test_merge_is_deterministic_and_reuses_the_existing_file() -> None:
    assets = resolve_model_assets("ARX_X5", effector_model="E_ARX")
    first = prepare_merged_urdf(assets, model="ARX_X5", effector_model="E_ARX")
    first_mtime_ns = first.stat().st_mtime_ns
    second = prepare_merged_urdf(assets, model="ARX_X5", effector_model="E_ARX")

    assert first == second
    # Unchanged content must not be rewritten (a rewrite races a sibling node
    # parsing the same path).
    assert second.stat().st_mtime_ns == first_mtime_ns


def test_yam_merge_replaces_the_massless_flange() -> None:
    assets = resolve_model_assets("Yam", effector_model="E_Yam")
    merged_path = prepare_merged_urdf(assets, model="Yam", effector_model="E_Yam")

    merged = merged_path.read_text()
    assert '<mass value="1e-6"/>' not in merged
    mass = json.loads(
        Path(str(assets.effector_model_config)).with_name("E_Yam_mass.json").read_text()
    )["mass"]
    assert f'<mass value="{mass:g}"/>' in merged
