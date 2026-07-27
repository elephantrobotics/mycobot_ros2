__version__ = "1.0.0"

# Prefer colored logs for ros2 run (WARN=yellow, ERROR=red). Launch files also set this.
import os
os.environ.setdefault("RCUTILS_COLORIZED_OUTPUT", "1")


def get_version():
    """Return current version of ultraArm P1 Python module."""
    return __version__
