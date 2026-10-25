"""Keep test-only ROS/SDK modules from leaking into the next test suite."""
import sys
from unittest.mock import patch


def isolate_imports():
    """Return unittest module fixtures; snapshot only when the suite starts."""
    guard = patch.dict(sys.modules)
    return guard.start, guard.stop
