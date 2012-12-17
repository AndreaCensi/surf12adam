import logging
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

import numpy as np 
from contracts import contract

from .diffeo_learner import * 
from .programs import *
from .oneclick import *
from .pixelized import *
