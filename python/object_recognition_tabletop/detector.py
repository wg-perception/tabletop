#!/usr/bin/env python
"""
Module defining the transparent objects detector to find objects in a scene
"""

import ecto
from object_recognition_tabletop.ecto_cells.tabletop_table import TablePose, Clusterer, TableDetector
from object_recognition_tabletop.ecto_cells import tabletop_object
from object_recognition_core.db import ObjectDb
from object_recognition_core.pipelines.detection import DetectionPipeline
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward

class TabletopTableDetector(ecto.BlackBox):
    def __init__(self, **kwargs):
        ecto.BlackBox.__init__(self, **kwargs)
    
    @classmethod
    def declare_cells(cls, _p):
        return {'passthrough': ecto.Passthrough(),
                'table_detector': TableDetector(),
                'table_pose': CellInfo(TablePose),
                'clusterer': Clusterer()
                }

    def declare_forwards(self, p):
        p = {'clusterer': 'all', 'table_detector': 'all'}

        i = {'passthrough': [Forward('in', 'point_cloud')]}

        o = {'table_detector': [Forward('clouds'),Forward('clouds_hull'), Forward('rotations'),
                                Forward('translations')],
             'clusterer': [Forward('clusters')],
             'table_pose': [Forward('pose_results')]
             }

        return (p,i,o)

    def connections(self, _p):
        # First find the table, then the pose
        connections = [ self.passthrough['out'] >> self.table_detector['cloud'],
                        self.table_detector['rotations'] >> self.table_pose['rotations'],
                        self.table_detector['translations'] >> self.table_pose['translations'] ]
        # also find the clusters of points
        connections += [ self.passthrough['out'] >> self.clusterer['cloud'],
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
        parameters = kwargs.pop('parameters')
        kwargs.update(parameters)

        return TabletopTableDetector(**kwargs)

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
        parameters = kwargs['parameters']
        object_db = ObjectDb(parameters['db'])

        return tabletop_object.ObjectRecognizer(object_ids=parameters['tabletop_object_ids'],
                                                db=object_db)
