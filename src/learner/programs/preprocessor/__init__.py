
import logging
logger = logging.getLogger('preprocess')
logger.setLevel(logging.DEBUG)

topic_image_raw = '/usb_cam/image_raw'
topic_camera_executed = '/logitech_cam/camera_executed'


from .pre_processor import *

from .main import *
