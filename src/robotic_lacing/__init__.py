"""
********************************************************************************
Robotic lacing
********************************************************************************

.. currentmodule:: robotic_lacing


.. toctree::
    :maxdepth: 1


"""

from __future__ import print_function
from .vision import find_corners

import os
import sys


__author__ = ["Lauren Dreier"]
__copyright__ = "2021"
__license__ = "MIT License"
__email__ = "ldreier@princeton.edu"
__version__ = "0.1.0"


HERE = os.path.dirname(__file__)

HOME = os.path.abspath(os.path.join(HERE, "../../"))
DATA = os.path.abspath(os.path.join(HOME, "data"))
DOCS = os.path.abspath(os.path.join(HOME, "docs"))
TEMP = os.path.abspath(os.path.join(HOME, "temp"))


__all__ = ["HOME", "DATA", "DOCS", "TEMP"]
