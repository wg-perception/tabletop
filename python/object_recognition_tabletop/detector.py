#!/usr/bin/env python
"""
Module defining the transparent objects detector to find objects in a scene
"""

from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward
from object_recognition_core.db import ObjectDb, ObjectDbParameters
from object_recognition_core.pipelines.detection import DetectorBase
from object_recognition_tabletop.ecto_cells import tabletop_object
from object_recognition_tabletop.ecto_cells.tabletop_table import TablePose, Clusterer, TableDetector
import ecto

class TabletopTableDetector(ecto.BlackBox, DetectorBase):
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)
        DetectorBase.__init__(self)

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

class TabletopObjectDetector(ecto.BlackBox, DetectorBase):

    def __init__(self, *args, **kwargs):
        self._params = kwargs
        ecto.BlackBox.__init__(self, *args, **kwargs)
        DetectorBase.__init__(self)

    def declare_cells(self, _p):
        return {'main': tabletop_object.ObjectRecognizer(object_ids=self._params['object_ids'],
                                                         db=ObjectDb(ObjectDbParameters(self._params['db'])))}

    def declare_forwards(self, p):
        return ({},{'main':'all'},{'main':'all'})

    def connections(self, _p):
        return [self.main]
