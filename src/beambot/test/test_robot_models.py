"""Rendered URDF/SRDF for every configured gripper match the beamline YAML.

The same inputs always produce the same model, so these checks run here instead
of on every MoveIt launch. Rerun after editing URDF/SRDF xacros, a gripper's
beamline entry, or its MoveIt config, then rebuild before launching the robot.
Rendering uses the installed cms_robot_description / cms_moveit_config packages.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
BEAMLINE_CONFIGS = ("cms_beamline.yaml", "cms_drylab.yaml")
# Match robot_bringup.launch.py's defaults; only names and structure are checked.
UR_TYPE = "ur5e"
ROBOT_IP = "192.168.1.101"


def model_content_error(grippers: dict, gripper: str, urdf: str, srdf: str) -> str:
    """Return a model/configuration mismatch, or an empty string on success."""
    try:
        urdf_root = ET.fromstring(urdf)
        srdf_root = ET.fromstring(srdf)
    except ET.ParseError as error:
        return f"invalid robot description XML: {error}"

    config = grippers[gripper]
    links = {link.get("name"): link for link in urdf_root.findall("link")}
    expected_tip = config.get("tip_frame", "")
    if expected_tip and expected_tip not in links:
        return f"expected link '{expected_tip}' is missing"

    known_tips = {
        item.get("tip_frame")
        for item in grippers.values()
        if item.get("tip_frame") and item.get("tip_frame") != "flange"
    }
    expected_tips = {expected_tip} if expected_tip != "flange" else set()
    if known_tips & links.keys() != expected_tips:
        return (
            f"tool links do not match {gripper}: expected "
            f"{sorted(expected_tips)}, got {sorted(known_tips & links.keys())}"
        )

    groups = {group.get("name"): group for group in srdf_root.findall("group")}
    known_groups = {
        item.get("gripper_group")
        for item in grippers.values()
        if item.get("gripper_group")
    }
    expected_group = config.get("gripper_group", "")
    expected_groups = {expected_group} if expected_group else set()
    if known_groups & groups.keys() != expected_groups:
        return (
            f"tool groups do not match {gripper}: expected "
            f"{sorted(expected_groups)}, got {sorted(known_groups & groups.keys())}"
        )

    expected_states = set((config.get("states") or {}).values())
    actual_states = {
        state.get("name")
        for state in srdf_root.findall("group_state")
        if state.get("group") == expected_group
    }
    if not expected_states <= actual_states:
        return f"expected named states are missing: {sorted(expected_states - actual_states)}"

    if expected_group:
        tool_links = {
            link.get("name") for link in groups[expected_group].findall("link")
        }
        missing_links = tool_links - links.keys()
        if missing_links:
            return f"SRDF tool links are missing from URDF: {sorted(missing_links)}"
    elif expected_tip and expected_tip != "flange":
        parents = {
            joint.find("child").get("link"): joint.find("parent").get("link")
            for joint in urdf_root.findall("joint")
            if joint.find("child") is not None and joint.find("parent") is not None
        }
        tool_links = set()
        link = expected_tip
        while link and link != "flange" and link not in tool_links:
            tool_links.add(link)
            link = parents.get(link)
    else:
        tool_links = set()

    if tool_links and not any(
        links[name].find("collision/geometry") is not None
        for name in tool_links
    ):
        return f"tool collision geometry is missing for {gripper}"
    return ""


def test_checker_accepts_matching_models_and_rejects_mismatches():
    grippers = {
        "hande": {
            "gripper_group": "hande_gripper",
            "tip_frame": "robotiq_hande_end",
            "states": {"grasp": "hande_closed", "release": "hande_open"},
        },
        "none": {"gripper_group": "", "tip_frame": "flange", "states": {}},
        "epick": {
            "gripper_group": "epick_gripper",
            "tip_frame": "epick_tip",
            "states": {"grasp": "vacuum_on", "release": "vacuum_off"},
        },
        "pipettor": {
            "gripper_group": "", "tip_frame": "pipette_tip_link", "states": {}
        },
    }
    descriptions = {
        "hande": (
            """<robot><link name="flange"/><link name="hande_body"><collision><geometry><box size="1 1 1"/></geometry></collision></link><link name="robotiq_hande_end"/></robot>""",
            """<robot><group name="ur_arm"/><group name="hande_gripper"><link name="hande_body"/><link name="robotiq_hande_end"/></group><group_state name="hande_open" group="hande_gripper"/><group_state name="hande_closed" group="hande_gripper"/></robot>""",
        ),
        "none": (
            "<robot><link name=\"flange\"/></robot>",
            "<robot><group name=\"ur_arm\"/></robot>",
        ),
        "epick": (
            """<robot><link name="flange"/><link name="epick_suction_cup"><collision><geometry><cylinder radius="0.0015" length="0.003"/></geometry></collision></link><link name="epick_tip"/></robot>""",
            """<robot><group name="ur_arm"/><group name="epick_gripper"><link name="epick_suction_cup"/><link name="epick_tip"/></group><group_state name="vacuum_on" group="epick_gripper"/><group_state name="vacuum_off" group="epick_gripper"/></robot>""",
        ),
        "pipettor": (
            """<robot><link name="flange"/><link name="pipette_body_link"><collision><geometry><box size="1 1 1"/></geometry></collision></link><link name="pipette_tip_link"/><joint name="body" type="fixed"><parent link="flange"/><child link="pipette_body_link"/></joint><joint name="tip" type="fixed"><parent link="pipette_body_link"/><child link="pipette_tip_link"/></joint></robot>""",
            "<robot><group name=\"ur_arm\"/></robot>",
        ),
    }

    for gripper, (urdf, srdf) in descriptions.items():
        assert model_content_error(grippers, gripper, urdf, srdf) == ""

    assert model_content_error(grippers, "epick", *descriptions["hande"])
    assert "collision geometry" in model_content_error(grippers, 
        "epick",
        descriptions["epick"][0].replace("<collision>", "<!--").replace(
            "</collision>", "-->"
        ),
        descriptions["epick"][1],
    )


def _configured_grippers():
    for config_name in BEAMLINE_CONFIGS:
        grippers = yaml.safe_load((CONFIG_DIR / config_name).read_text())["grippers"]
        for gripper, entry in grippers.items():
            if "launch_urdf" in entry:
                yield pytest.param(grippers, gripper, id=f"{config_name}:{gripper}")


def _render(grippers: dict, gripper: str) -> tuple[str, str]:
    """Render the URDF/SRDF with the xacro arguments robot_bringup.launch.py uses."""
    xacro = pytest.importorskip("xacro")
    packages = pytest.importorskip("ament_index_python.packages")
    try:
        desc_share = packages.get_package_share_path("cms_robot_description")
        moveit_share = packages.get_package_share_path("cms_moveit_config")
    except packages.PackageNotFoundError as error:
        pytest.skip(f"robot description packages are not built: {error}")

    entry = grippers[gripper]
    launch_gripper = entry.get("gripper_arg", gripper)
    mappings = {
        "name": UR_TYPE,
        "ur_type": UR_TYPE,
        "tf_prefix": "",
        "robot_ip": ROBOT_IP,
        "kinematics_params": str(desc_share / "config" / "ur5e_calibration.yaml"),
    }
    if launch_gripper == "epick":
        mappings["cup_profile"] = entry["cup_profile"]
    elif launch_gripper == "hande":
        mappings["socat_ip_address"] = ROBOT_IP

    urdf = xacro.process_file(
        str(desc_share / "urdf" / entry["launch_urdf"]), mappings=mappings
    ).toxml()
    srdf = xacro.process_file(
        str(moveit_share / "srdf" / "ur.srdf.xacro"), mappings={"gripper": launch_gripper}
    ).toxml()
    return urdf, srdf


@pytest.mark.parametrize(("grippers", "gripper"), _configured_grippers())
def test_rendered_model_matches_gripper_config(grippers, gripper):
    assert model_content_error(grippers, gripper, *_render(grippers, gripper)) == ""
