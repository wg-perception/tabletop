#!/usr/bin/env python
"""
Module defining the transparent objects detector to find objects in a scene
"""

from ecto_object_recognition.object_recognition_db import DbModels, ObjectDbParameters
from object_recognition.common.utils import json_helper
from object_recognition.pipelines.detection import DetectionPipeline
import tabletop_cells

########################################################################################################################

class TabletopTableDetectionPipeline(DetectionPipeline):
    @classmethod
    def type_name(cls):
        return 'tabletop_table'

    #def detector(self, submethod, parameters, db_params, model_documents, args):
    def detector(self, *args, **kwargs):
        visualize = kwargs.pop('visualize', False)
        submethod = kwargs.pop('submethod')
        parameters = kwargs.pop('parameters')

        return tabletop_cells.TableDetector(visualize=visualize)

########################################################################################################################

#class TabletopObjectDetectionPipeline(DetectionPipeline):
#    @classmethod
#    def type_name(cls):
#        return 'tabletop_object'
#
#    #def detector(self, submethod, parameters, db_params, model_documents, args):
#    def detector(self, *args, **kwargs):
#        visualize = kwargs.pop('visualize', False)
#        submethod = kwargs.pop('submethod')
#        parameters = kwargs.pop('parameters')
#
#        return tabletop_cells.ObjectDetector(visualize=visualize)
