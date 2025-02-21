/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkQtSQLDatabase.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/*-------------------------------------------------------------------------
  Copyright 2008 Sandia Corporation.
  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
  the U.S. Government retains certain rights in this software.
-------------------------------------------------------------------------*/

#include "vtkQtSQLDatabase.h"

#include "vtkObjectFactory.h"
#include "vtkQtSQLQuery.h"

#include "vtkStringArray.h"
#include "vtkVariant.h"

#include <QtSql/QSqlError>
#include <QtSql/QtSql>

#include <sstream>
#include <vtksys/SystemTools.hxx>

vtkStandardNewMacro(vtkQtSQLDatabase);

int vtkQtSQLDatabase::id = 0;

vtkSQLDatabase* vtkQtSQLDatabaseCreateFromURLCallback(const char* URL)
{
  return vtkQtSQLDatabase::CreateFromURL(URL);
}

class vtkQtSQLDatabaseInitializer
{
public:
  inline void Use() {}

  vtkQtSQLDatabaseInitializer()
  {
    vtkSQLDatabase::RegisterCreateFromURLCallback(vtkQtSQLDatabaseCreateFromURLCallback);
  }
};

static vtkQtSQLDatabaseInitializer vtkQtSQLDatabaseInitializerGlobal;

vtkQtSQLDatabase::vtkQtSQLDatabase()
{
  vtkQtSQLDatabaseInitializerGlobal.Use();
  this->DatabaseType = nullptr;
  this->HostName = nullptr;
  this->UserName = nullptr;
  this->DatabaseName = nullptr;
  this->DbPort = -1;
  this->ConnectOptions = nullptr;
  this->myTables = vtkStringArray::New();
  this->currentRecord = vtkStringArray::New();
}

vtkQtSQLDatabase::~vtkQtSQLDatabase()
{
  this->SetDatabaseType(nullptr);
  this->SetHostName(nullptr);
  this->SetUserName(nullptr);
  this->SetDatabaseName(nullptr);
  this->SetConnectOptions(nullptr);
  this->myTables->Delete();
  this->currentRecord->Delete();
}

bool vtkQtSQLDatabase::Open(const char* password)
{
  if (!QCoreApplication::instance())
  {
    vtkErrorMacro("Qt isn't initialized, you must create an instance of QCoreApplication before "
                  "using this class.");
    return false;
  }

  if (this->DatabaseType == nullptr)
  {
    vtkErrorMacro("Qt database type must be non-null.");
    return false;
  }

  // We have to assign a unique ID to each database connection, so
  // Qt doesn't blow-away existing connections
  const QString connection_name = QString::number(vtkQtSQLDatabase::id++);
  this->QtDatabase = QSqlDatabase::addDatabase(this->DatabaseType, connection_name);

  if (this->HostName != nullptr)
  {
    this->QtDatabase.setHostName(this->HostName);
  }
  if (this->DatabaseName != nullptr)
  {
    this->QtDatabase.setDatabaseName(this->DatabaseName);
  }
  if (this->ConnectOptions != nullptr)
  {
    this->QtDatabase.setConnectOptions(this->ConnectOptions);
  }
  if (this->DbPort >= 0)
  {
    this->QtDatabase.setPort(this->DbPort);
  }
  if (this->QtDatabase.open(this->UserName, password))
  {
    return true;
  }

  return false;
}

void vtkQtSQLDatabase::Close()
{
  this->QtDatabase.close();
}

bool vtkQtSQLDatabase::IsOpen()
{
  return this->QtDatabase.isOpen();
}

vtkSQLQuery* vtkQtSQLDatabase::GetQueryInstance()
{
  vtkQtSQLQuery* query = vtkQtSQLQuery::New();
  query->SetDatabase(this);
  return query;
}

bool vtkQtSQLDatabase::HasError()
{
  return this->QtDatabase.lastError().isValid();
}

const char* vtkQtSQLDatabase::GetLastErrorText()
{
  return this->QtDatabase.lastError().text().toUtf8().data();
}

vtkStringArray* vtkQtSQLDatabase::GetTables()
{
  // Clear out any exiting stuff
  this->myTables->Initialize();

  // Yea... do different things depending on database type
  // Get tables on oracle is different
  if (this->QtDatabase.driverName() == "QOCI")
  {
    vtkSQLQuery* query = this->GetQueryInstance();
    query->SetQuery("select table_name from user_tables");
    query->Execute();
    while (query->NextRow())
      this->myTables->InsertNextValue(query->DataValue(0).ToString());

    // Okay done with query so delete
    query->Delete();
  }
  else
  {
    // Copy the table list from Qt database
    QStringList tables = this->QtDatabase.tables(QSql::Tables);
    for (int i = 0; i < tables.size(); ++i)
    {
      this->myTables->InsertNextValue(tables.at(i).toUtf8().data());
    }
  }

  return this->myTables;
}

vtkStringArray* vtkQtSQLDatabase::GetRecord(const char* table)
{
  // Clear any existing records
  currentRecord->Resize(0);

  QSqlRecord columns = this->QtDatabase.record(table);
  for (int i = 0; i < columns.count(); i++)
  {
    this->currentRecord->InsertNextValue(columns.fieldName(i).toUtf8().data());
  }

  return currentRecord;
}

vtkStringArray* vtkQtSQLDatabase::GetColumns()
{
  return this->currentRecord;
}

void vtkQtSQLDatabase::SetColumnsTable(const char* table)
{
  this->GetRecord(table);
}

bool vtkQtSQLDatabase::IsSupported(int feature)
{
  switch (feature)
  {
    case VTK_SQL_FEATURE_TRANSACTIONS:
      return this->QtDatabase.driver()->hasFeature(QSqlDriver::Transactions);

    case VTK_SQL_FEATURE_QUERY_SIZE:
      return this->QtDatabase.driver()->hasFeature(QSqlDriver::QuerySize);

    case VTK_SQL_FEATURE_BLOB:
      return this->QtDatabase.driver()->hasFeature(QSqlDriver::BLOB);

    case VTK_SQL_FEATURE_UNICODE:
      return this->QtDatabase.driver()->hasFeature(QSqlDriver::Unicode);

    case VTK_SQL_FEATURE_PREPARED_QUERIES:
      return this->QtDatabase.driver()->hasFeature(QSqlDriver::PreparedQueries);

    case VTK_SQL_FEATURE_NAMED_PLACEHOLDERS:
      return this->QtDatabase.driver()->hasFeature(QSqlDriver::NamedPlaceholders);

    case VTK_SQL_FEATURE_POSITIONAL_PLACEHOLDERS:
      return this->QtDatabase.driver()->hasFeature(QSqlDriver::PositionalPlaceholders);

    case VTK_SQL_FEATURE_LAST_INSERT_ID:
      return this->QtDatabase.driver()->hasFeature(QSqlDriver::LastInsertId);

    case VTK_SQL_FEATURE_BATCH_OPERATIONS:
      return this->QtDatabase.driver()->hasFeature(QSqlDriver::BatchOperations);

    default:
    {
      vtkErrorMacro(<< "Unknown SQL feature code " << feature << "!  See "
                    << "vtkSQLDatabase.h for a list of possible features.");
      return false;
    }
  }
}

void vtkQtSQLDatabase::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "DatabaseType: " << (this->DatabaseType ? this->DatabaseType : "nullptr") << endl;
  os << indent << "HostName: " << (this->HostName ? this->HostName : "nullptr") << endl;
  os << indent << "UserName: " << (this->UserName ? this->UserName : "nullptr") << endl;
  os << indent << "DatabaseName: " << (this->DatabaseName ? this->DatabaseName : "nullptr") << endl;
  os << indent << "DbPort: " << this->DbPort << endl;
  os << indent << "ConnectOptions: " << (this->ConnectOptions ? this->ConnectOptions : "nullptr")
     << endl;
}

//------------------------------------------------------------------------------
bool vtkQtSQLDatabase::ParseURL(const char* URL)
{
  std::string protocol;
  std::string username;
  std::string unused;
  std::string hostname;
  std::string dataport;
  std::string database;
  std::string dataglom;

  // SQLite is a bit special so lets get that out of the way :)
  if (!vtksys::SystemTools::ParseURLProtocol(URL, protocol, dataglom))
  {
    vtkGenericWarningMacro("Invalid URL: " << URL);
    return false;
  }

  if (protocol == "sqlite")
  {
    this->SetDatabaseType("QSQLITE");
    this->SetDatabaseName(dataglom.c_str());
    return true;
  }

  // Okay now for all the other database types get more detailed info
  if (!vtksys::SystemTools::ParseURL(URL, protocol, username, unused, hostname, dataport, database))
  {
    vtkGenericWarningMacro("Invalid URL: " << URL);
    return false;
  }

  // Create Qt 'version' of database prototcol type
  QString qtType = "Q" + QString::fromUtf8(protocol.c_str()).toUpper();

  this->SetDatabaseType(qtType.toUtf8().data());
  this->SetUserName(username.c_str());
  this->SetHostName(hostname.c_str());
  this->SetDbPort(atoi(dataport.c_str()));
  this->SetDatabaseName(database.c_str());
  return true;
}

//------------------------------------------------------------------------------
vtkSQLDatabase* vtkQtSQLDatabase::CreateFromURL(const char* URL)
{
  vtkQtSQLDatabase* qt_db = vtkQtSQLDatabase::New();
  if (qt_db->ParseURL(URL))
  {
    return qt_db;
  }
  qt_db->Delete();
  return nullptr;
}

//------------------------------------------------------------------------------
vtkStdString vtkQtSQLDatabase::GetURL()
{
  vtkStdString url;
  url = this->GetDatabaseType();
  url += "://";
  url += this->GetUserName();
  url += "@";
  url += this->GetHostName();
  url += ":";
  url += std::to_string(this->GetDbPort());
  url += "/";
  url += this->GetDatabaseName();
  return url;
}
