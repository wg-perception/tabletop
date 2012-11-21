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
    _clusterer = Clusterer
    _depth_map = RescaledRegisteredDepth
    _points3d = DepthTo3d
    if ECTO_ROS_FOUND:
        message_cvt = ecto_ros.Mat2Image

    def __init__(self, submethod, parameters, **kwargs):
        self._submethod = submethod
        self._parameters = parameters

        ecto.BlackBox.__init__(self, **kwargs)

    def declare_params(self, p):
        p.forward_all('_clusterer')
        if ECTO_ROS_FOUND:
            p.forward('rgb_frame_id', cell_name='message_cvt', cell_key='frame_id')

    def declare_io(self, _p, i, o):
        self.passthrough = ecto.PassthroughN(items=dict(image='An image',
                                                   K='The camera matrix'
                                                   ))
        i.forward(['image', 'K'], cell_name='passthrough', cell_key=['image', 'K'])
        #i.forward('mask', cell_name='to_cloud_conversion', cell_key='mask')
        i.forward('depth', cell_name='_depth_map', cell_key='depth')

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
            param = self._parameters
            if 'object_ids' in param:
                param.pop('object_ids')
            if 'clustering_voxel_size' in param:
                param.pop('clustering_voxel_size')
            if 'cluster_distance' in param:
                param.pop('cluster_distance')
            if 'min_cluster_size' in param:
                param.pop('min_cluster_size')
            if 'table_z_filter_min' in param:
                param.pop('table_z_filter_min')
            if 'table_z_filter_max' in param:
                param.pop('table_z_filter_max')                                                
            
            self.table_detector = TableDetector(**self._parameters)
        else:
            self.table_detector = TableDetector()
        self._depth_map = RescaledRegisteredDepth()
        self._points3d = DepthTo3d()
        self.to_cloud_conversion = MatToPointCloudXYZOrganized()

    def connections(self):
        # Rescale the depth image and convert to 3d
        connections = [ self.passthrough['image'] >> self._depth_map['image'],
                       self._depth_map['depth'] >>  self._points3d['depth'],
                       self.passthrough['K'] >> self._points3d['K'],
                       self._points3d['points3d'] >> self.to_cloud_conversion['points'] ]
        # First find the table, then the pose
        connections += [self.to_cloud_conversion['point_cloud'] >> self.table_detector['cloud'],
                       self.table_detector['rotations'] >> self.table_pose['rotations'],
                       self.table_detector['translations'] >> self.table_pose['translations'] ]
        # also find the clusters of points
        connections += [self.to_cloud_conversion['point_cloud'] >> self._clusterer['cloud'],
                       self.table_detector['clouds_hull'] >> self._clusterer['clouds_hull'] ]

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
