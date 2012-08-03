__version__ = '0.1'

import numpy as np
from contracts import contract

__docformat__ = 'restructuredtext'

### Setup logging
from logging import getLogger
from conf_tools.utils import col_logging # colored logging

import logging
logging.basicConfig()

everybody_uses_same_logger = True
if everybody_uses_same_logger:
    logger = logging.getLogger('diffeoplan')
    getLogger = lambda _: logger
else:
    logger = logging.getLogger(__name__)

logger.setLevel(logging.DEBUG)

from .configuration import *
