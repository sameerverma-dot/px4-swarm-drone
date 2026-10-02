import sys
from pathlib import Path

# Lets the tests import the package from a source checkout, from any directory.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
