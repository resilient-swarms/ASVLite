import pathlib
import sys
unit_test_dir  = pathlib.Path(__file__).parent.resolve()
cython_dir    = unit_test_dir.parent
sys.path.insert(0, str(cython_dir))

import unittest
from test_geometry import *

unittest.main()