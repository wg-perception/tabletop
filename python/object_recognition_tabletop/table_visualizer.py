"""
Module defining a cell to visualize the output of the plane/cluster finder
"""

import ecto
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward
from ecto_opencv.highgui import imshow
from ecto_opencv.rgbd import PlaneDrawer
from object_recognition_tabletop.ecto_cells.tabletop_table import ClusterDrawer

########################################################################################################################

class TableVisualizer(ecto.BlackBox):
    """
    Class displaying the output of the table finder: the different planes and the clusters
    """
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)

    @classmethod
    def declare_cells(cls, p):
        return {'cluster_drawer': CellInfo(ClusterDrawer)}

    def declare_forwards(self, _p):
        i = {'cluster_drawer': [Forward('clusters2d')]}

        return ({},i,{})

    def configure(self, _p, _i, _o):
        self.plane_drawer = PlaneDrawer()
        self.imshow = imshow('Tabletop table results')

    def connections(self, _p):
        return [ self.plane_drawer['image'] >> self.cluster_drawer['image'],
                 self.cluster_drawer['image'] >> self.imshow['image'] ]
