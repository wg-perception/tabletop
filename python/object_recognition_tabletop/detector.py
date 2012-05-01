#!/usr/bin/env python
"""
Module defining the transparent objects detector to find objects in a scene
"""

import ecto
from object_recognition_tabletop.ecto_cells.tabletop_table import TablePose, Clusterer, TableDetector
from ecto_image_pipeline.conversion import MatToPointCloudXYZOrganized
from object_recognition_core.boost.interface import DbModels, ObjectDbParameters
from object_recognition_core.db.object_db import ObjectDb
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
    _clusterer = Clusterer
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

        o.forward('clouds', cell_name='table_detector', cell_key='clouds')
        o.forward('clouds_hull', cell_name='table_detector', cell_key='clouds_hull')
        o.forward('clusters', cell_name='_clusterer', cell_key='clusters')
        o.forward('rotations', cell_name='table_detector', cell_key='rotations')
        o.forward('translations', cell_name='table_detector', cell_key='translations')
        o.forward('pose_results', cell_name='table_pose', cell_key='pose_results')

    def configure(self, p, _i, _o):
        vertical_direction = self._parameters.pop('vertical_direction', None)
        if vertical_direction is not None:
            self.table_pose = tabletop_table.TablePose(vertical_direction=vertical_direction)
        else:
            self.table_pose = TablePose()
        if self._parameters:
            self.table_detector = TableDetector(**self._parameters)
        else:
            self.table_detector = TableDetector()

    def connections(self):
        # First find the table, then the pose
        connections = [self.to_cloud_conversion['point_cloud'] >> self.table_detector['cloud'],
                       self.table_detector['rotations'] >> self.table_pose['rotations'],
                       self.table_detector['translations'] >> self.table_pose['translations'] ]
        # also find the clusters of points
        connections += [self.to_cloud_conversion['point_cloud'] >> self._clusterer['cloud'],
                       self.table_detector['clouds_hull'] >> self._clusterer['clouds_hull'] ]

        return connections

########################################################################################################################

class TabletopTableDetectionPipeline(DetectionPipeline):
    @classmethod
    def type_name(cls):
        return 'tabletop_table'

    @classmethod
    def detector(self, *args, **kwargs):
        submethod = kwargs.pop('submethod')
        parameters = kwargs.pop('parameters')

        return TabletopTableDetector(submethod, parameters, **kwargs)

########################################################################################################################

class TabletopObjectDetectionPipeline(DetectionPipeline):
    @classmethod
    def type_name(cls):
        return 'tabletop_object'

    @classmethod
    def detector(self, *args, **kwargs):
        visualize = kwargs.pop('visualize', False)
        submethod = kwargs.pop('submethod')
        parameters = kwargs.pop('parameters')
        object_db = ObjectDb(parameters['db'])

        return tabletop_object.ObjectRecognizer(object_ids=parameters['object_ids'],
                                                db=object_db)
