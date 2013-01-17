"""
Module defining the Table Publisher
"""

from object_recognition_ros.ecto_cells.io_ros import Publisher_Marker, Publisher_MarkerArray
from object_recognition_core.io.sink import SinkBase
from object_recognition_tabletop.ecto_cells.tabletop_table import TableMsgAssembler, TableVisualizationMsgAssembler
from object_recognition_msgs.ecto_cells.ecto_object_recognition_msgs import Publisher_TableArray
import ecto
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward

MarkerPub = Publisher_Marker
MarkerArrayPub = Publisher_MarkerArray

########################################################################################################################

class TablePublisher(ecto.BlackBox, SinkBase):
    """
    Class publishing the different results of tabletop
    """
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)
        SinkBase.__init__(self)

    @classmethod
    def declare_cells(cls, p):
        return {'table_msg_assembler': CellInfo(TableMsgAssembler),
                'table_visualization_msg_assembler': CellInfo(TableVisualizationMsgAssembler),
                'marker_array_hull': CellInfo(MarkerArrayPub, params={'latched': p.latched}),
                'marker_array_origin': CellInfo(MarkerArrayPub, params={'latched': p.latched}),
                'marker_array_table': CellInfo(MarkerArrayPub, params={'latched': p.latched}),
                'marker_array_delete': CellInfo(MarkerArrayPub),
                'marker_array_clusters': CellInfo(MarkerArrayPub),
                'table_array': CellInfo(Publisher_TableArray),
                'passthrough': ecto.PassthroughN(items=dict(image_message='The original imagemessage',
                                                        pose_results='The final results'))
                }

    @staticmethod
    def declare_direct_params(p):
        p.declare('latched', 'Determines if the topics will be latched.', True)

    def declare_forwards(self, _p):
        p = {'marker_array_hull': [Forward('topic_name', 'marker_hull_topic',
                                           'The ROS topic to use for the table message.', 'marker_table')],
             'marker_array_origin': [Forward('topic_name', 'marker_origin_topic',
                                             'The ROS topic to use for the table message.', 'marker_table')],
             'marker_array_table': [Forward('topic_name', 'marker_table_topic',
                                            'The ROS topic to use for the table message.', 'marker_table')],
             'marker_array_delete': [Forward('topic_name', 'marker_array_delete',
                                             'The ROS topic to use for the markers to remove.', 'marker_table')],
             'marker_array_clusters': [Forward('topic_name', 'marker_array_clusters',
                                               'The ROS topic to use for the markers of the clusters.',
                                               'marker_array_clusters')],
             'table_array': [Forward('topic_name', 'table_array', 'The array of found tables.', 'table_array')]
             }

        i = {'table_msg_assembler': [Forward('clouds'), Forward('clouds_hull')],
             'table_visualization_msg_assembler': [Forward('clusters'), Forward('table_array_msg')],
             'passthrough': [Forward('image_message'), Forward('pose_results')]}

        return (p,i,{})

    def connections(self, _p):
        connections = [self.passthrough['image_message'] >> self.table_msg_assembler['image_message'],
                       self.passthrough['image_message'] >> self.table_visualization_msg_assembler['image_message'],
                       self.passthrough['pose_results'] >> self.table_msg_assembler['pose_results'],
                       self.passthrough['pose_results'] >> self.table_visualization_msg_assembler['pose_results'] ]

        connections += [ self.table_msg_assembler['table_array_msg'] >> self.table_array[:],
                        self.table_msg_assembler['table_array_msg'] >> self.table_visualization_msg_assembler['table_array_msg'] ]
        connections += [self.table_visualization_msg_assembler['marker_array_hull'] >> self.marker_array_hull[:],
                self.table_visualization_msg_assembler['marker_array_origin'] >> self.marker_array_origin[:],
                self.table_visualization_msg_assembler['marker_array_table'] >> self.marker_array_table[:],
                self.table_visualization_msg_assembler['marker_array_delete'] >> self.marker_array_delete[:],
                self.table_visualization_msg_assembler['marker_array_clusters'] >> self.marker_array_clusters[:] ]
        return connections
