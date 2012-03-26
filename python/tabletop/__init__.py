"""
Next lines inspired from object_recognition_core/db/__init__.py
"""

#from ecto.load_pybindings import load_pybindings
#load_pybindings(__name__)

#import os

# go over the PYTHONPATH, and add any folder ending in object_recognition_core/db
#INSTALL_FOLDER = os.path.join('tabletop')

#for path in os.environ['PYTHONPATH'].split(':'):
#    potential_additional_path = os.path.join(path, INSTALL_FOLDER)
#    if os.path.isdir(potential_additional_path) and path not in __path__:
#        __path__.append(potential_additional_path)




from tabletop.detector import TabletopObjectDetectionPipeline, TabletopTableDetectionPipeline
from tabletop.table_publisher import TablePublisherSink
from tabletop.db import SqlHouseHoldDb
