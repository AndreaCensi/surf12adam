#!/bin/bash
# Produce a second bag
rosrun logitech_cam pre_processor.py -i /media/data/raw-capture/commandsptz1800.raw.bag -o /media/data/processed-data/commandsptz1800w320.processed.bag -w 320 -h 240
# Learn the first bag
rosrun logitech_cam learner.py -i /media/data/processed-data/commandsptz1800w160.processed.bag -o /media/data/learned_diffeo/diffeoptz1800w160.pickle
# Learn the second bag
rosrun logitech_cam learner.py -i /media/data/processed-data/commandsptz1800w320.processed.bag -o /media/data/learned_diffeo/diffeoptz1800w320.pickle

# Produce and learn a third bag
rosrun logitech_cam pre_processor.py -i /media/data/raw-capture/commandsptz1800.raw.bag -o /media/data/processed-data/commandsptz1800w640.processed.bag -w 640 -h 480
rosrun logitech_cam learner.py -i /media/data/processed-data/commandsptz1800w640.processed.bag -o /media/data/learned_diffeo/diffeoptz1800w640.pickle


