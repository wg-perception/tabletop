#!/usr/bin/env python
"""
Module defining the transparent objects detector to find objects in a scene
"""

import ecto
from object_recognition_tabletop.ecto_cells.tabletop_table import TablePose, Clusterer, TableDetector
from object_recognition_tabletop.ecto_cells import tabletop_object
from ecto_image_pipeline.base import RescaledRegisteredDepth
from ecto_image_pipeline.conversion import MatToPointCloudXYZOrganized
from ecto_opencv.calib import DepthTo3d
from ecto_pcl import Cropper
from object_recognition_core.db import ObjectDb, ObjectDbParameters
from object_recognition_core.pipelines.detection import DetectionPipeline
from object_recognition_core.utils import json_helper

try:
    import ecto_ros
    ECTO_ROS_FOUND = True
except ImportError:
    ECTO_ROS_FOUND = False

class TabletopTableDetector(ecto.BlackBox):
    table_detector = TableDetector
    table_pose = TablePose
    to_cloud_conversion = MatToPointCloudXYZOrganized
    passthrough = ecto.PassthroughN
    clusterer = Clusterer
    cropper = Cropper
    if ECTO_ROS_FOUND:
        message_cvt = ecto_ros.Mat2Image

    def __init__(self, submethod, parameters, **kwargs):
        self._submethod = submethod
        self._parameters = parameters

        ecto.BlackBox.__init__(self, **kwargs)

    def declare_params(self, p):
        p.forward_all('clusterer')
        p.forward_all('cropper')
        p.forward_all('table_detector')

        if ECTO_ROS_FOUND:
            p.forward('rgb_frame_id', cell_name='message_cvt', cell_key='frame_id')

    def declare_io(self, _p, i, o):
        i.forward('point_cloud', cell_name='cropper', cell_key='input')

        o.forward('clouds', cell_name='table_detector', cell_key='clouds')
        o.forward('clouds_hull', cell_name='table_detector', cell_key='clouds_hull')
        o.forward('clusters', cell_name='clusterer', cell_key='clusters')
        o.forward('rotations', cell_name='table_detector', cell_key='rotations')
        o.forward('translations', cell_name='table_detector', cell_key='translations')
        o.forward('pose_results', cell_name='table_pose', cell_key='pose_results')

    def configure(self, p, _i, _o):
        vertical_direction = self._parameters.pop('vertical_direction', None)
        if vertical_direction is not None:
            self.table_pose = tabletop_table.TablePose(vertical_direction=vertical_direction)
        else:
            self.table_pose = TablePose()
        ## TODO not correctly passed, look into this
#        if self._parameters:
#            if self._parameters.has_key('cropper'):
#                print self._parameters['cropper']
#                cropper = Cropper(**self._parameters['cropper'])
#            if self._parameters.has_key('table_detector'):
#                print 'td'
#                table_detector = TableDetector(**self._parameters['table_detector'])                                       
#            if self._parameters.has_key('clusterer'):
#                print 'cr'
#                table_detector = Clusterer(**self._parameters['clusterer'])

    def connections(self):
        # First find the table, then the pose
        connections = [ self.cropper['output'] >> self.table_detector['cloud'],
                       self.table_detector['rotations'] >> self.table_pose['rotations'],
                       self.table_detector['translations'] >> self.table_pose['translations'] ]
        # also find the clusters of points
        connections += [ self.cropper['output'] >> self.clusterer['cloud'],
                       self.table_detector['clouds_hull'] >> self.clusterer['clouds_hull'] ]

        return connections

########################################################################################################################

class TabletopTableDetectionPipeline(DetectionPipeline):

    @classmethod
    def config_doc(cls):
        return  """
        # No parameters are required
        parameters: ''
        """

    @classmethod
    def type_name(cls):
        return 'tabletop_table'

    @classmethod
    def detector(self, *args, **kwargs):
        submethod = kwargs.pop('subtype')
        parameters = kwargs.pop('parameters')

        return TabletopTableDetector(submethod, parameters, **kwargs)

########################################################################################################################

class TabletopObjectDetectionPipeline(DetectionPipeline):

    @classmethod
    def config_doc(cls):
        return  """
        """

    @classmethod
    def type_name(cls):
        return 'tabletop_object'

    @classmethod
    def detector(self, *args, **kwargs):
        visualize = kwargs.pop('visualize', False)
        submethod = kwargs.pop('subtype')
        parameters = kwargs.pop('parameters')
        object_db = ObjectDb(parameters['db'])

        return tabletop_object.ObjectRecognizer(object_ids=parameters['tabletop_object_ids'],
                                                db=object_db)
