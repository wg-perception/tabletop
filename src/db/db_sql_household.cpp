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
#include <object_recognition_core/db/db.h>
#include <object_recognition_tabletop/household.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ObjectDbSqlHousehold::ObjectDbSqlHousehold()
{
  ObjectDbParametersRaw parameters = default_raw_parameters();
  this->set_parameters(object_recognition_core::db::ObjectDbParameters(parameters));

  // Create the DB object
  db_.reset(new household_objects_database::ObjectsDatabase(parameters["host"].get_str(), parameters["port"].get_str(),
                                                      parameters["user"].get_str(), parameters["password"].get_str(),
                                                      parameters["name"].get_str()));
}

ObjectDbSqlHousehold::ObjectDbSqlHousehold(ObjectDbParametersRaw & in_parameters)
{
  ObjectDbParametersRaw parameters = default_raw_parameters();
  // Read the parameters
  for (ObjectDbParametersRaw::const_iterator iter = parameters.begin(), end = parameters.end(); iter != end; ++iter)
  {
    if (iter->first == "type")
      continue;

    ObjectDbParametersRaw::const_iterator val = in_parameters.find(iter->first);
    if (val == in_parameters.end())
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
  in_parameters = parameters;
  this->set_parameters(object_recognition_core::db::ObjectDbParameters(parameters));

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
ObjectDbSqlHousehold::GetObjectRevisionId(DocumentId& document_id, RevisionId & revision_id)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::GetRevisionId(RevisionId & revision_id)
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
                            int& total_rows, int& offset, std::vector<ViewElement> & view_elements)
{
  switch (view.type())
  {
    case object_recognition_core::db::View::VIEW_MODEL_WHERE_OBJECT_ID_AND_MODEL_TYPE:
    {
      throw std::runtime_error("Function not implemented in the SQL household DB.");
      break;
    }
    case object_recognition_core::db::View::VIEW_OBJECT_INFO_WHERE_OBJECT_ID:
    {
      object_recognition_core::db::View::Key key;
      std::string options;
      if (view.key(key))
      {
        // Get the information from the different tables
        BOOST_FOREACH(ViewElement & view_element, view_elements)
        {

        }
      }
      else
        throw std::runtime_error("Function not implemented in the SQL household DB.");
      break;
    }
  }
}

void
ObjectDbSqlHousehold::QueryGeneric(const std::vector<std::string> & queries, int limit_rows, int start_offset, int& total_rows,
                            int& offset, std::vector<ViewElement> & view_elements)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

/** Once json_reader_stream_ has been filled, call that function to get the results of the view
 *
 */
void
ObjectDbSqlHousehold::QueryView(const std::string & in_url, int limit_rows, int start_offset,
                                const std::string &options, int& total_rows, int& offset,
                                std::vector<ViewElement> & view_elements, bool do_throw)
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

void
ObjectDbSqlHousehold::upload_json(const or_json::mObject &params, const std::string& url, const std::string& request)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}
