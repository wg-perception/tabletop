#!/usr/bin/env python
"""
Module defining the transparent objects detector to find objects in a scene
"""

from ecto_object_recognition.object_recognition_db import DbModels, ObjectDbParameters
from object_recognition.common.utils import json_helper
from object_recognition.pipelines.detection import DetectionPipeline
from image_pipeline_conversion import MatToPointCloudXYZOrganized
import tabletop_table
import ecto

try:
    import ecto_ros
    ECTO_ROS_FOUND = True
except ImportError:
    ECTO_ROS_FOUND = False

class TabletopTableDetector(ecto.BlackBox):
    table_detector = tabletop_table.TableDetector
    table_pose = tabletop_table.TablePose
    to_cloud_conversion = MatToPointCloudXYZOrganized
    passthrough = ecto.PassthroughN
    if ECTO_ROS_FOUND:
        message_cvt = ecto_ros.Mat2Image

    def __init__(self, submethod, parameters, **kwargs):
        self._submethod = submethod
        self._parameters = parameters

        ecto.BlackBox.__init__(self, **kwargs)

    def declare_params(self, p):
        if ECTO_ROS_FOUND:
            p.forward('rgb_frame_id', cell_name='message_cvt', cell_key='frame_id')

    def declare_io(self, _p, i, o):
        self.passthrough = ecto.PassthroughN(items=dict(image='An image',
                                                   K='The camera matrix'
                                                   ))
        i.forward(['image', 'K'], cell_name='passthrough', cell_key=['image', 'K'])
        #i.forward('mask', cell_name='to_cloud_conversion', cell_key='mask')
        i.forward('points3d', cell_name='to_cloud_conversion', cell_key='points')

        o.forward('pose_results', cell_name='table_pose', cell_key='pose_results')

    def configure(self, p, _i, _o):
        vertical_direction = self._parameters.pop('vertical_direction', None)
        if vertical_direction is not None:
            self.table_pose = tabletop_table.TablePose(vertical_direction=self._parameters['vertical_direction'])
        if self._parameters:
            self.table_detector = tabletop_table.TableDetector(**self._parameters)

    def connections(self):
        # make sure the inputs reach the right cells
        connections = [self.to_cloud_conversion['point_cloud'] >> self.table_detector['image'],
                       self.passthrough['image'] >> self.guess_generator['image'], ]

        connections += [ self.descriptor_matcher['spans'] >> self.guess_generator['spans'],
                       self.descriptor_matcher['object_ids'] >> self.guess_generator['object_ids'] ]

        return connections

########################################################################################################################

class TabletopTableDetectionPipeline(DetectionPipeline):
    @classmethod
    def type_name(cls):
        return 'tabletop_table'

    #def detector(self, submethod, parameters, db_params, model_documents, args):
    def detector(self, *args, **kwargs):
        submethod = kwargs.pop('submethod')
        parameters = kwargs.pop('parameters')

        return TabletopTableDetector(submethod, parameters, **kwargs)

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
