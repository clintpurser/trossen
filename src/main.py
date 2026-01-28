"""Viam module entry point for ViperX-300s arm and gripper."""

import asyncio
import logging
import sys
import os

# Configure logging early
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# Add src to path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

logger.info("Starting ViperX-300s module")

try:
    from viam.module.module import Module

    logger.debug("Importing Viperx300s...")
    from models.viperx_300s import Viperx300s

    logger.debug(f"Viperx300s imported, MODEL={Viperx300s.MODEL}")

    logger.debug("Importing Viperx300sGripper...")
    from models.viperx_300s_gripper import Viperx300sGripper

    logger.debug(f"Viperx300sGripper imported, MODEL={Viperx300sGripper.MODEL}")

except Exception as e:
    logger.error(f"Failed to import modules: {e}", exc_info=True)
    raise


if __name__ == "__main__":
    logger.info("Running module from registry")
    asyncio.run(Module.run_from_registry())
