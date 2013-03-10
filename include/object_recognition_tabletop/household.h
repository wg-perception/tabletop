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

#ifndef DB_SQL_HOUSEHOLD_H_
#define DB_SQL_HOUSEHOLD_H_

#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/db_base.h>
#include <object_recognition_core/common/types.h>
#include <household_objects_database/objects_database.h>

using object_recognition_core::db::AttachmentName;
using object_recognition_core::db::CollectionName;
using object_recognition_core::db::DbType;
using object_recognition_core::db::DocumentId;
using object_recognition_core::db::ObjectId;
using object_recognition_core::db::MimeType;
using object_recognition_core::db::ObjectDbParameters;
using object_recognition_core::db::ObjectDbParametersRaw;
using object_recognition_core::db::RevisionId;
using object_recognition_core::db::View;
using object_recognition_core::db::ViewElement;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ObjectDbSqlHousehold: public object_recognition_core::db::ObjectDb
{
public:
  ObjectDbSqlHousehold();

  virtual
  ~ObjectDbSqlHousehold() {
  }

  virtual ObjectDbParametersRaw
  default_raw_parameters() const;

  virtual void
  set_parameters(ObjectDbParameters & in_parameters);

  virtual void
  insert_object(const or_json::mObject &fields, DocumentId & document_id, RevisionId & revision_id);

  virtual void
  persist_fields(const DocumentId & document_id, const or_json::mObject &fields, RevisionId & revision_id);

  virtual void
  load_fields(const DocumentId & document_id, or_json::mObject &fields);

  virtual void
  get_attachment_stream(const DocumentId & document_id, const RevisionId & revision_id, const std::string& attachment_name,
                        const std::string& content_type, std::ostream& stream);

  virtual void
  set_attachment_stream(const DocumentId & document_id, const AttachmentName& attachment_name,
                        const MimeType& mime_type, const std::istream& stream, RevisionId & revision_id);

  virtual
  void
  Delete(const ObjectId & id);

  virtual
  void
  QueryView(const View & view, int limit_rows, int start_offset, int& total_rows, int& offset,
        std::vector<ViewElement> & view_elements);

  virtual void
  QueryGeneric(const std::vector<std::string> & queries, int limit_rows, int start_offset, int& total_rows, int& offset,
               std::vector<ViewElement> & view_elements);

  virtual std::string
  Status() const;

  virtual std::string
  Status(const CollectionName& collection) const;

  virtual void
  CreateCollection(const CollectionName &collection);

  virtual void
  DeleteCollection(const CollectionName &collection);

  virtual DbType
  type() const
  {
    return "ObjectDbSqlHousehold";
  }

  boost::shared_ptr<household_objects_database::ObjectsDatabase> db() {
    return db_;
  }
private:

  /** Once json_reader_stream_ has been filled, call that function to get the results of the view
   *
   */
  void
  QueryView(const CollectionName & collection_name, int limit_rows, int start_offset, const std::string &options,
            int& total_rows, int& offset, std::vector<ViewElement> & view_elements, bool do_throw);

  boost::shared_ptr<household_objects_database::ObjectsDatabase> db_;
};

#endif /* DB_SQL_HOUSEHOLD_H_ */
