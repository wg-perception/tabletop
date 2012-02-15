"""
Module defining the Table Publisher
"""

from object_recognition_core.io.sink import Sink
from tabletop_table import TablePublisher

########################################################################################################################

class TablePublisherPython(Sink):

    @classmethod
    def type_name(cls):
        return 'table_publisher'

    @classmethod
    def source(self, *args, **kwargs):
        return TablePublisher(**kwargs)
