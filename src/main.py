import asyncio
from viam.module.module import Module
from models.viperx_300s import Viperx300S


if __name__ == '__main__':
    asyncio.run(Module.run_from_registry())
