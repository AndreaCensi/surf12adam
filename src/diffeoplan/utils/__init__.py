from .script_utils import *
from .lenient_option_parser import *
from .matrices import *
from .memoization import *
from bootstrapping_olympics.utils.change_module import assign_all_to_module

assign_all_to_module(sys.modules[__name__])
