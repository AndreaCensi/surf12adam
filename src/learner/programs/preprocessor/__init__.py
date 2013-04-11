
import logging
logger = logging.getLogger('preprocess')
logger.setLevel(logging.DEBUG)

topic_image_raw = '/usb_cam/image_raw'
topic_camera_executed = '/logitech_cam/camera_executed'
#topic_camera_executed = '/youbot_arm/incremental_instruction'


from .pre_processor import *

from .main import *
from youbot_preprocessor import *
