"""Beamline config validation that runs once when the config loads."""

from pathlib import Path

import pytest
import yaml

import beambot.config_loader as config_loader
from beambot.config_loader import BeamlineConfigError, validate_beamline_config

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

VALID_GRIPPER = {
    "tool_voltage": 24,
    "payload_mass": 1.5,
    "payload_cog": {"x": 0.01, "y": -0.02, "z": 0.03},
}


def _errors(**gripper_overrides):
    gripper = {**VALID_GRIPPER, **gripper_overrides}
    return validate_beamline_config({"grippers": {"hande": gripper}})


@pytest.fixture(autouse=True)
def _reset_config_cache():
    config_loader.reset_beamline_config_cache()
    yield
    config_loader.reset_beamline_config_cache()


@pytest.mark.parametrize(
    "config_name",
    [
        "cms_beamline.yaml",
        "cms_drylab.yaml",
        pytest.param(
            "lix_beamline.yaml",
            marks=pytest.mark.xfail(
                strict=True, reason="LIX hande payload_mass/payload_cog not yet measured"
            ),
        ),
    ],
)
def test_beamline_configs_are_valid(config_name):
    config = yaml.safe_load((CONFIG_DIR / config_name).read_text())
    assert validate_beamline_config(config) == []


def test_valid_gripper_and_absent_sections_pass():
    assert _errors() == []
    assert validate_beamline_config({}) == []
    # Omitted tool_voltage means 0 V.
    assert validate_beamline_config({"grippers": {"none": {
        "payload_mass": 1.0, "payload_cog": {"x": 0, "y": 0, "z": 0},
    }}}) == []


@pytest.mark.parametrize(
    ("overrides", "field"),
    [
        ({"payload_mass": None}, "payload_mass"),
        ({"payload_mass": "heavy"}, "payload_mass"),
        ({"payload_mass": True}, "payload_mass"),
        ({"payload_mass": float("nan")}, "payload_mass"),
        ({"payload_mass": float("inf")}, "payload_mass"),
        ({"payload_mass": 0}, "payload_mass"),
        ({"payload_mass": -1}, "payload_mass"),
        ({"payload_cog": None}, "payload_cog"),
        ({"payload_cog": {"x": 0.0, "y": 0.0}}, "payload_cog"),
        ({"payload_cog": {"x": "left", "y": 0.0, "z": 0.0}}, "payload_cog"),
        ({"payload_cog": {"x": 0.0, "y": float("nan"), "z": 0.0}}, "payload_cog"),
        ({"tool_voltage": 48}, "tool_voltage"),
        ({"tool_voltage": "24"}, "tool_voltage"),
        ({"tool_voltage": False}, "tool_voltage"),
    ],
)
def test_invalid_gripper_values_are_reported(overrides, field):
    errors = _errors(**overrides)
    assert len(errors) == 1
    assert errors[0].startswith(f"grippers.hande.{field}:")


def test_non_mapping_gripper_is_reported():
    assert validate_beamline_config({"grippers": {"hande": 24}}) == [
        "grippers.hande: expected a mapping"
    ]


def test_load_reports_every_error_and_is_not_cached(tmp_path, monkeypatch):
    config = tmp_path / "beamline.yaml"
    config.write_text(yaml.safe_dump({"grippers": {
        "hande": {**VALID_GRIPPER, "payload_mass": 0},
        "epick": {**VALID_GRIPPER, "tool_voltage": 48},
    }}))
    monkeypatch.setenv("BEAMBOT_BEAMLINE_CONFIG", str(config))

    with pytest.raises(BeamlineConfigError) as raised:
        config_loader.load_beamline_config()
    assert "grippers.hande.payload_mass" in str(raised.value)
    assert "grippers.epick.tool_voltage" in str(raised.value)

    # Fixing the file lets the next load succeed without a restart.
    config.write_text(yaml.safe_dump({"grippers": {"hande": VALID_GRIPPER}}))
    assert config_loader.load_beamline_config()[0]["grippers"]["hande"] == VALID_GRIPPER
