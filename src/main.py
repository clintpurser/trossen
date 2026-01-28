"""Viam module entry point for ViperX-300s arm and gripper."""

import asyncio
import sys
import os

# Add src to path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from viam.module.module import Module
from models.viperx_300s import Viperx300s
from models.viperx_300s_gripper import Viperx300sGripper


if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
