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
#include "db_sql_household.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ObjectDbSqlHousehold::ObjectDbSqlHousehold()
    :
      ObjectDbBase("http://localhost:5984", "object_recognition")
{
}

ObjectDbSqlHousehold::ObjectDbSqlHousehold(const std::string &root, const std::string &collection)
    :
      ObjectDbBase(root, collection)
{
}

void
ObjectDbSqlHousehold::insert_object(const or_json::mObject &fields, DocumentId & document_id, RevisionId & revision_id)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::persist_fields(const DocumentId & document_id, const or_json::mObject &fields, RevisionId & revision_id)
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
                                     const MimeType& mime_type, const std::istream& stream, RevisionId & revision_id)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::get_attachment_stream(const DocumentId & document_id, const std::string& attachment_name,
                                     const std::string& content_type, std::ostream& stream, RevisionId & revision_id)
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
ObjectDbSqlHousehold::Query(const object_recognition::db::View & view, int limit_rows, int start_offset, int& total_rows,
                     int& offset, std::vector<ViewElement> & view_elements)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::Query(const std::vector<std::string> & queries, int limit_rows, int start_offset, int& total_rows,
                     int& offset, std::vector<ViewElement> & view_elements)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

/** Once json_reader_stream_ has been filled, call that function to get the results of the view
 *
 */
void
ObjectDbSqlHousehold::QueryView(const std::string & in_url, int limit_rows, int start_offset, const std::string &options,
                         int& total_rows, int& offset, std::vector<ViewElement> & view_elements, bool do_throw)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

void
ObjectDbSqlHousehold::CreateCollection(const CollectionName &collection)
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

std::string
ObjectDbSqlHousehold::Status()
{
  throw std::runtime_error("Function not implemented in the SQL household DB.");
}

std::string
ObjectDbSqlHousehold::Status(const CollectionName& collection)
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
