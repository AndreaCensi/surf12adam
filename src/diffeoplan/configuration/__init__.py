from .. import logger
from .checks import *
from .master import *
# 
# class Storage:
#     config = None
#     
def set_current_config(config):
    set_dp_config(config)
#     Storage.config = config
    
def get_current_config():
#     if Storage.config is None:
#         msg = 'No current config was given'
#         raise ValueError(msg)
#     return Storage.config

    return get_dp_config()
