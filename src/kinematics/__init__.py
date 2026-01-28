"""Kinematics data for ViperX-300s arm."""

import os
import sys


def _get_kinematics_path() -> str:
    """Get absolute path to kinematics file, works for dev and PyInstaller bundle."""
    if hasattr(sys, "_MEIPASS"):
        # PyInstaller extracts data files to _MEIPASS/kinematics/
        return os.path.join(sys._MEIPASS, "kinematics", "vx300s.json")
    else:
        # Development mode: file is in same directory as this module
        return os.path.join(os.path.dirname(__file__), "vx300s.json")


KINEMATICS_FILE = _get_kinematics_path()
