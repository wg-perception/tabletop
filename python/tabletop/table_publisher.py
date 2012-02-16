"""
Module defining the Table Publisher
"""

from object_recognition_core.io.sink import Sink

import ecto
import ecto_tabletop
from ecto_object_recognition.io_ros import Publisher_Marker, Publisher_MarkerArray
from tabletop_table import TableMsgAssembler

TablePub = ecto_tabletop.Publisher_Table
MarkerPub = Publisher_Marker
MarkerArrayPub = Publisher_MarkerArray

########################################################################################################################

class TablePublisher(ecto.BlackBox):
    """
    Class publishing the different results of tabletop
    """
    _table_msg_assembler = TableMsgAssembler
    _table_pub = TablePub
    _marker_hull_pub = MarkerPub
    _marker_origin_pub = MarkerPub
    _marker_table_pub = MarkerPub
    _marker_delete = MarkerArrayPub

    def declare_params(self, p):
        p.declare('marker_hull_topic', 'The ROS topic to use for the table message.', 'marker_hull')
        p.declare('marker_origin_topic', 'The ROS topic to use for the table message.', 'marker_origin')
        p.declare('marker_table_topic', 'The ROS topic to use for the table message.', 'marker_table')
        p.declare('marker_array_delete', 'The ROS topic to use for the markers to remove.', 'marker_array_delete')
        p.declare('latched', 'Determines if the topics will be latched.', True)

    def declare_io(self, _p, i, _o):
        i.forward_all('_table_msg_assembler')

    def configure(self, p, _i, _o):
        self._table_msg_assembler = TablePublisher._table_msg_assembler()
        self._marker_hull_pub = TablePublisher._marker_hull_pub(topic_name=p.marker_hull_topic, latched=p.latched)
        self._marker_origin_pub = TablePublisher._marker_origin_pub(topic_name=p.marker_origin_topic, latched=p.latched)
        self._marker_table_pub = TablePublisher._marker_table_pub(topic_name=p.marker_table_topic, latched=p.latched)
        self._marker_delete = TablePublisher._marker_delete(topic_name=p.marker_array_delete)

    def connections(self):
        return [self._table_msg_assembler['marker_hull'] >> self._marker_hull_pub[:],
                self._table_msg_assembler['marker_origin'] >> self._marker_origin_pub[:],
                self._table_msg_assembler['marker_table'] >> self._marker_table_pub[:],
                self._table_msg_assembler['marker_array_delete'] >> self._marker_delete[:] ]

########################################################################################################################

class TablePublisherSink(Sink):

    @classmethod
    def type_name(cls):
        return 'table_publisher'

    @classmethod
    def sink(self, *args, **kwargs):
        return TablePublisher()
