"""
Module defining a cell to visualize the output of the plane/cluster finder
"""

import ecto
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward
from ecto_opencv.highgui import imshow
from ecto_opencv.rgbd import ClusterDrawer, PlaneDrawer

########################################################################################################################

class TableVisualizer(ecto.BlackBox):
    """
    Class displaying the output of the table finder: the different planes and the clusters
    """
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)

    @staticmethod
    def declare_cells(p):
        return {'cluster_drawer': CellInfo(ClusterDrawer),
                'plane_drawer': CellInfo(PlaneDrawer)}

    @staticmethod
    def declare_forwards(_p):
        i = {'cluster_drawer': [Forward('clusters2d')],
             'plane_drawer': [Forward('image'), Forward('masks', 'plane_mask')]}

        return ({},i,{})

    def configure(self, _p, _i, _o):
        self.imshow = imshow('Tabletop table results')

    def connections(self, _p):
        return [ self.plane_drawer['image'] >> self.cluster_drawer['image'],
                 self.cluster_drawer['image'] >> self.imshow['image'] ]
