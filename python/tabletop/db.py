"""
Define some interface to the SQL household database
"""

from object_recognition_core.db.object_db import ObjectDbBase
from object_recognition_core.db.interface import ObjectDbParameters
from tabletop.db_interface import ObjectDb as ObjectDbCpp

########################################################################################################################

class SqlHouseHoldDb(ObjectDbBase):
    @classmethod
    def type_name(cls):
        return 'SqlHousehold'

    @classmethod
    def object_db(cls, db_params):
        return ObjectDbCpp(ObjectDbParameters(db_params))
