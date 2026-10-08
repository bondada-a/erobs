"""Filesystem-only checks for beamline path handling."""

from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from beambot.config_loader import get_beamline_config_path, resolve_beamline_path


def test_beamline_paths():
    with TemporaryDirectory() as directory:
        root = Path(directory)
        workspace = root / "workspace"
        config = workspace / "src" / "beamline" / "config.yaml"
        config.parent.mkdir(parents=True)
        config.write_text("beamline: test\n")
        poses = workspace / "poses.yaml"

        assert resolve_beamline_path("", str(config)) == ""
        assert resolve_beamline_path("poses.yaml", str(config)) == str(poses)
        assert resolve_beamline_path(str(poses), str(config)) == str(poses)
        assert resolve_beamline_path("poses.yaml", str(root / "config.yaml")) == str(
            root / "poses.yaml"
        )

        # Relative resources follow the config's target when it is a symlink.
        alias = root / "config.yaml"
        alias.symlink_to(config)
        assert resolve_beamline_path("poses.yaml", str(alias)) == str(poses)

        # Declared absolute paths normalize '..' without following symlinks.
        link = root / "linked_directory"
        link.symlink_to(config.parent, target_is_directory=True)
        lexical = str(link / ".." / "poses.yaml")
        assert resolve_beamline_path(lexical, str(config)) == str(root / "poses.yaml")

        with patch.dict("os.environ", {
            "HOME": str(root),
            "BEAMBOT_BEAMLINE_CONFIG": "~/linked_directory/../config.yaml",
        }):
            assert get_beamline_config_path() == str(alias)
            assert resolve_beamline_path("~/poses.yaml", str(config)) == str(root / "poses.yaml")


if __name__ == "__main__":
    test_beamline_paths()
    print("Beamline path checks passed.")
