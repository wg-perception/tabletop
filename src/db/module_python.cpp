#include <boost/python.hpp>

namespace object_recognition_core
{
  namespace db
  {
    void
    wrap_object_db_local();
  }
}

BOOST_PYTHON_MODULE(db_interface)
{
  using namespace object_recognition_core::db;
  wrap_object_db_local();
}
