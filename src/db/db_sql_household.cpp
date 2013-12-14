/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <sstream>
#include <object_recognition_tabletop/household.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ObjectDbSqlHousehold::ObjectDbSqlHousehold()
{
}

void
ObjectDbSqlHousehold::set_parameters(ObjectDbParameters & in_parameters)
{
  ObjectDbParametersRaw parameters = in_parameters.raw();
  // Read the parameters
  for (ObjectDbParametersRaw::const_iterator iter = parameters.begin(), end = parameters.end(); iter != end; ++iter)
  {
    if (iter->first == "type")
      continue;

    ObjectDbParametersRaw::const_iterator val = parameters.find(iter->first);
    if (val == parameters.end())
      std::cerr << "The db parameters do not contain the field \""
          << iter->first << "\". Using the default: \""
          << iter->second.get_str() << "\"" << std::endl;
    else {
      if ((iter->first == "port") && (val->second.type() == or_json::int_type)) {
        std::stringstream ss;
        ss << val->second.get_int();
        parameters[iter->first] = ss.str();
      } else {
        if (val->second.type() != or_json::str_type)
          throw std::runtime_error(
              std::string("Key \"") + val->first
                  + std::string("\" needs to be a string"));
        parameters[iter->first] = val->second.get_str();
      }
    }
  }
  in_parameters = object_recognition_core::db::ObjectDbParameters(parameters);
  parameters_ = in_parameters;

  // Create the DB object
  db_.reset(new household_objects_database::ObjectsDatabase(parameters.at("host").get_str(), parameters.at("port").get_str(),
                                                      parameters.at("user").get_str(),
                                                      parameters.at("password").get_str(),
                                                      parameters.at("name").get_str()));
}

ObjectDbParametersRaw
ObjectDbSqlHousehold::default_raw_parameters() const
{
  ObjectDbParametersRaw res;
  res["host"] = "wgs36";
  res["port"] = "5432";
  res["user"] = "willow";
  res["password"] = "willow";
  res["name"] = "household_objects";
  res["type"] = type();

  return res;
}

void
ObjectDbSqlHousehold::insert_object(const or_json::mObject &fields, DocumentId & document_id, RevisionId & revision_id)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::persist_fields(const DocumentId & document_id, const or_json::mObject &fields,
                                     RevisionId & revision_id)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::load_fields(const DocumentId & document_id, or_json::mObject &fields)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::set_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                                            const MimeType& mime_type, const std::istream& stream,
                                            RevisionId & revision_id)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::get_attachment_stream(const DocumentId & document_id, const RevisionId & revision_id, const std::string& attachment_name,
                                            const std::string& content_type, std::ostream& stream)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::Delete(const ObjectId & id)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::QueryView(const object_recognition_core::db::View & view, int limit_rows, int start_offset,
                            int& total_rows, int& offset, std::vector<Document> & view_elements)
{
  switch (view.type())
  {
    case object_recognition_core::db::View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE:
    {
      object_recognition_core::db::View::Key key;
      std::string options;
      if (view.key(key))
      {
        // Load the mesh from the DB
        household_objects_database::DatabaseMesh mesh;
        db_->getScaledModelMesh(atoi(key.get_str().c_str()), mesh);

	std::stringstream stream(std::ios::in | std::ios::out | std::ios::binary);
        // Write the mesh to a stream according to the specs at http://en.wikipedia.org/wiki/STL_%28file_format%29
        int sizeof_uc = 1;
        int sizeof_us = 2;
        int sizeof_ui = 4;
        int sizeof_f = 4;
        // Write the random 80 character header
        for (unsigned char i = 0; i < 80; ++i)
          stream.write(reinterpret_cast<char*>(&i), sizeof_uc);
        unsigned int n_triangles = mesh.triangles_.data().size() / 3;
        stream.write(reinterpret_cast<char*>(&n_triangles), sizeof_ui);

        unsigned short zero_us = 0;
        for (size_t i = 0; i < n_triangles; ++i) {
          float x1[3];
          float x2[3];
          float x3[3];
          float X1[3], X2[3];
          // For each coordinate of the vertex
          for (char k = 0; k < 3; ++k) {
            x1[k] = mesh.vertices_.data().at(
                3 * mesh.triangles_.data().at(3 * i + 0) + k);
            x2[k] = mesh.vertices_.data().at(
                3 * mesh.triangles_.data().at(3 * i + 1) + k);
            x3[k] = mesh.vertices_.data().at(
                3 * mesh.triangles_.data().at(3 * i + 2) + k);
            X1[k] = x2[k]-x1[k];
            X2[k] = x3[k]-x1[k];
          }
          float normal[3];
          normal[0] = X1[1] * X2[2] - X1[2] * X2[1];
          normal[1] = X2[0] * X1[2] - X2[2] * X1[0];
          normal[2] = X1[0] * X2[1] - X1[1] * X2[0];
          float norm = std::sqrt(
              normal[0] * normal[0] + normal[1] * normal[1]
                  + normal[2] * normal[2]);
          for (char k = 0; k < 3; ++k)
            normal[k] = 0*normal[k] / norm;
          // Write the normal
          stream.write(reinterpret_cast<char*>(&normal[0]), sizeof_f).write(
              reinterpret_cast<char*>(&normal[1]), sizeof_f).write(
              reinterpret_cast<char*>(&normal[2]), sizeof_f);
          // Write the 3 vertices
          stream.write(reinterpret_cast<char*>(&x1[0]), sizeof_f).write(
              reinterpret_cast<char*>(&x1[1]), sizeof_f).write(
              reinterpret_cast<char*>(&x1[2]), sizeof_f);
          stream.write(reinterpret_cast<char*>(&x2[0]), sizeof_f).write(
              reinterpret_cast<char*>(&x2[1]), sizeof_f).write(
              reinterpret_cast<char*>(&x2[2]), sizeof_f);
          stream.write(reinterpret_cast<char*>(&x3[0]), sizeof_f).write(
              reinterpret_cast<char*>(&x3[1]), sizeof_f).write(
              reinterpret_cast<char*>(&x3[2]), sizeof_f);
          // Write the attribute
          stream.write(reinterpret_cast<char*>(&zero_us), sizeof_us);
        }

	stream.seekg(0);
        // Create the view element
        object_recognition_core::db::Document view_element;
        view_element.SetIdRev("", key.get_str());
        view_element.set_field("name", key.get_str());
        view_element.set_attachment_stream("mesh", stream);
        view_elements.push_back(view_element);
      }
      else
        throw std::runtime_error("Function not implemented in the SQL household DB.");
      break;
    }
  }
}

void
ObjectDbSqlHousehold::QueryGeneric(const std::vector<std::string> & queries, int limit_rows, int start_offset, int& total_rows,
                            int& offset, std::vector<Document> & view_elements)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

/** Once json_reader_stream_ has been filled, call that function to get the results of the view
 *
 */
void
ObjectDbSqlHousehold::QueryView(const std::string & in_url, int limit_rows, int start_offset,
                                const std::string &options, int& total_rows, int& offset,
                                std::vector<Document> & view_elements, bool do_throw)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::CreateCollection(const CollectionName &collection)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

std::string
ObjectDbSqlHousehold::Status() const
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

std::string
ObjectDbSqlHousehold::Status(const CollectionName& collection) const
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::DeleteCollection(const CollectionName &collection)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ObjectDbSqlHousehold, object_recognition_core::db::ObjectDb)
