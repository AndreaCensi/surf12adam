import os
from setuptools import setup, find_packages

version = "0.1"

description = """"""

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

long_description = read('README.md')


setup(name='surf12adam',
      url='http://github.com/AndreaCensi/surf12adam',

      description=description,
      long_description=long_description,
      keywords="",
      license="",

      classifiers=[
        'Development Status :: 4 - Beta',
      ],

	    version=version,
      package_dir={'':'src'},
      packages=find_packages('src'),
      install_requires=[
        #'scikits.image', # TODO: put conditional 
        'BootOlympics>=1.0,<2'],
      setup_requires=['nose>=1.0'],
      tests_require=['nose>=1.0'],

       entry_points={
         'console_scripts': [
           'dp = diffeoplan.programs:dpmain',
           'diffeo-learner = learner:diffeo_learner_main',
#           'diffeo-tests = learner:diffeo_learner_main',
           'preprocess = learner:preprocessor_main',
           'youbot-preprocess = learner:youbot_preprocessor',
           'oneclick = learner:one_click_main',
           'dp-online = diffeoplan.programs.online:main',
           'dp-ros = diffeoplan.programs.online:ros_main',
           'dp-compose = diffeoplan.library.discdds.compose_graph:main',
           'timeuse = diffeoplan.programs.timeuse_learners:main',
           'dds-explorer = diffeoplan.programs.dds_explorer.dds_explorer:main'
        ]
    }
)

