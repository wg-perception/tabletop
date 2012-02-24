"""
Define some interface to the SQL household database
"""

from object_recognition_core.db.object_db import ObjectDbBase
from object_recognition_core.db.interface import ObjectDbParameters
from tabletop.db_interface import SqlHouseholdDb

########################################################################################################################

class SqlHouseHoldDb(ObjectDbBase):
    @classmethod
    def type_name(cls):
        return 'SqlHousehold'

    @classmethod
    def object_db(self, db_params):
        return SqlHouseholdDb(ObjectDbParameters(db_params))
