from .. import logger, np, contract

class Storage:
    config = None
    
def set_current_config(config):
    Storage.config = config
    
def get_current_config():
    if Storage.config is None:
        msg = 'No current config was given'
        raise ValueError(msg)
    return Storage.config
    

from .sym_diffeo import *
from .eval_diffeo import *
from .twirl import *
from .square_domain import *
from .coordinate_change import *
from .viewport import *
from .ddsfromsymbolic import *
