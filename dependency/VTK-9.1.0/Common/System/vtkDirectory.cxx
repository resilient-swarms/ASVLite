/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkDirectory.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkDirectory.h"
#include "vtkStringArray.h"

#include "vtkDebugLeaks.h"
#include "vtkObjectFactory.h"

#include <vtksys/SystemTools.hxx>

vtkStandardNewMacro(vtkDirectory);

vtkDirectory::vtkDirectory()
  : Path(nullptr)
{
  this->Files = vtkStringArray::New();
}

void vtkDirectory::CleanUpFilesAndPath()
{
  this->Files->Reset();
  delete[] this->Path;
  this->Path = nullptr;
}

vtkDirectory::~vtkDirectory()
{
  this->CleanUpFilesAndPath();
  this->Files->Delete();
  this->Files = nullptr;
}

void vtkDirectory::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Files:  (" << this->Files << ")\n";
  if (!this->Path)
  {
    os << indent << "Directory not open\n";
    return;
  }

  os << indent << "Directory for: " << this->Path << "\n";
  os << indent << "Contains the following files:\n";
  indent = indent.GetNextIndent();
  for (int i = 0; i < this->Files->GetNumberOfValues(); i++)
  {
    os << indent << this->Files->GetValue(i) << "\n";
  }
}

// First microsoft and borland compilers

#if defined(_MSC_VER) || defined(__BORLANDC__) || defined(__MINGW32__)
#include "vtkWindows.h"
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <direct.h>
#include <fcntl.h>
#include <io.h>
#include <sys/types.h>

int vtkDirectory::Open(const char* name)
{
  // clean up from any previous open
  this->CleanUpFilesAndPath();

  char* buf = 0;
  int n = static_cast<int>(strlen(name));
  if (name[n - 1] == '/')
  {
    buf = new char[n + 1 + 1];
    snprintf(buf, n + 1 + 1, "%s*", name);
  }
  else
  {
    buf = new char[n + 2 + 1];
    snprintf(buf, n + 2 + 1, "%s/*", name);
  }
  struct _finddata_t data; // data of current file

  // First count the number of files in the directory
  intptr_t srchHandle;

  srchHandle = _findfirst(buf, &data);

  if (srchHandle == -1)
  {
    _findclose(srchHandle);
    delete[] buf;
    return 0;
  }

  delete[] buf;

  // Loop through names
  do
  {
    this->Files->InsertNextValue(data.name);
  } while (_findnext(srchHandle, &data) != -1);

  this->Path = strcpy(new char[strlen(name) + 1], name);

  return _findclose(srchHandle) != -1;
}

const char* vtkDirectory::GetCurrentWorkingDirectory(char* buf, unsigned int len)
{
  return _getcwd(buf, len);
}

#else

// Now the POSIX style directory access

#include <dirent.h>
#include <sys/types.h>
#include <unistd.h>

// PGI with glibc has trouble with dirent and large file support:
//  http://www.pgroup.com/userforum/viewtopic.php?
//  p=1992&sid=f16167f51964f1a68fe5041b8eb213b6
// Work around the problem by mapping dirent the same way as readdir.
#if defined(__PGI) && defined(__GLIBC__)
#define vtkdirectory_dirent_readdir dirent
#define vtkdirectory_dirent_readdir64 dirent64
#define vtkdirectory_dirent vtkdirectory_dirent_lookup(readdir)
#define vtkdirectory_dirent_lookup(x) vtkdirectory_dirent_lookup_delay(x)
#define vtkdirectory_dirent_lookup_delay(x) vtkdirectory_dirent_##x
#else
#define vtkdirectory_dirent dirent
#endif

int vtkDirectory::Open(const char* name)
{
  // clean up from any previous open
  this->CleanUpFilesAndPath();

  DIR* dir = opendir(name);

  if (!dir)
  {
    return 0;
  }

  vtkdirectory_dirent* d = nullptr;

  for (d = readdir(dir); d; d = readdir(dir))
  {
    this->Files->InsertNextValue(d->d_name);
  }
  this->Path = strcpy(new char[strlen(name) + 1], name);

  closedir(dir);

  return 1;
}

const char* vtkDirectory::GetCurrentWorkingDirectory(char* buf, unsigned int len)
{
  return getcwd(buf, len);
}

#endif

//------------------------------------------------------------------------------
int vtkDirectory::MakeDirectory(const char* dir)
{
  return vtksys::SystemTools::MakeDirectory(dir).IsSuccess();
}

const char* vtkDirectory::GetFile(vtkIdType index)
{
  if (index >= this->Files->GetNumberOfValues() || index < 0)
  {
    vtkErrorMacro(<< "Bad index for GetFile on vtkDirectory\n");
    return nullptr;
  }

  return this->Files->GetValue(index).c_str();
}

vtkIdType vtkDirectory::GetNumberOfFiles()
{
  return this->Files->GetNumberOfValues();
}

//------------------------------------------------------------------------------
int vtkDirectory::FileIsDirectory(const char* name)
{
  // The vtksys::SystemTools::FileIsDirectory()
  // does not equal the following code (it probably should),
  // and it will broke KWWidgets. Reverse back to 1.30
  // return vtksys::SystemTools::FileIsDirectory(name);

  if (name == nullptr)
  {
    return 0;
  }

  int absolutePath = 0;
#if defined(_WIN32)
  if (name[0] == '/' || name[0] == '\\')
  {
    absolutePath = 1;
  }
  else
  {
    for (int i = 0; name[i] != '\0'; i++)
    {
      if (name[i] == ':')
      {
        absolutePath = 1;
        break;
      }
      else if (name[i] == '/' || name[i] == '\\')
      {
        break;
      }
    }
  }
#else
  if (name[0] == '/')
  {
    absolutePath = 1;
  }
#endif

  char* fullPath;

  int n = 0;
  if (!absolutePath && this->Path)
  {
    n = static_cast<int>(strlen(this->Path));
  }

  int m = static_cast<int>(strlen(name));

  fullPath = new char[n + m + 2];

  if (!absolutePath && this->Path)
  {
    strcpy(fullPath, this->Path);
#if defined(_WIN32)
    if (fullPath[n - 1] != '/' && fullPath[n - 1] != '\\')
    {
#if !defined(__CYGWIN__)
      fullPath[n++] = '\\';
#else
      fullPath[n++] = '/';
#endif
    }
#else
    if (fullPath[n - 1] != '/')
    {
      fullPath[n++] = '/';
    }
#endif
  }

  strcpy(&fullPath[n], name);

  int result = 0;
  vtksys::SystemTools::Stat_t fs;
  if (vtksys::SystemTools::Stat(fullPath, &fs) == 0)
  {
#if defined(_WIN32)
    result = ((fs.st_mode & _S_IFDIR) != 0);
#else
    result = S_ISDIR(fs.st_mode);
#endif
  }

  delete[] fullPath;

  return result;
}

int vtkDirectory::DeleteDirectory(const char* dir)
{
  return vtksys::SystemTools::RemoveADirectory(dir).IsSuccess();
}

int vtkDirectory::Rename(const char* oldname, const char* newname)
{
  return 0 == rename(oldname, newname);
}
