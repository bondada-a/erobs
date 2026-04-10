"""Allow running as: python3 -m beambot.agent 'pick up the sample'"""
from .cli import main
import asyncio

asyncio.run(main())
