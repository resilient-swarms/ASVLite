/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkParseSystem.c

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkParseSystem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <dirent.h>
#endif
#include <sys/stat.h>

/* for PGI compiler, use dirent64 if readdir is readdir64 */
/* (KWSys has similar code in Directory.cxx) */
#if defined(__PGI) && defined(__GLIBC__)
#define system_dirent_readdir struct dirent
#define system_dirent_readdir64 struct dirent64
#define system_dirent system_dirent_lookup(readdir)
#define system_dirent_lookup(x) system_dirent_lookup2(x)
#define system_dirent_lookup2(x) system_dirent_##x
#else
#define system_dirent struct dirent
#endif

/* struct dirent only has d_type on certain systems */
/* (Linux defines _DIRENT_HAVE_D_TYPE to help us out) */
#ifndef HAVE_DIRENT_D_TYPE
#if defined(_DIRENT_HAVE_D_TYPE) || defined(__FreeBSD__) || defined(__APPLE__)
#define HAVE_DIRENT_D_TYPE
#endif
#endif

/* Ugly macro to check if a character is a path separator */
#ifdef _WIN32
#define system_path_separator(c) ((c) == '/' || (c) == '\\')
#else
#define system_path_separator(c) ((c) == '/')
#endif

/* Use hash table size that is a power of two */
#define FILE_HASH_TABLE_SIZE 1024u

/**
 * Add to the list of file system paths that are known to exist.
 * The file type must be specified: VTK_PARSE_ISFILE or VTK_PARSE_ISDIR.
 */
static void system_file_add(SystemInfo* info, const char* name, system_filetype_t type)
{
  const char**** htable_ref;
  const char*** htable;
  const char** hptr;
  size_t l = strlen(name);
  size_t n;
  /* FILE_HASH_TABLE_SIZE will always be power of two */
  unsigned int j = FILE_HASH_TABLE_SIZE - 1; /* bit mask */
  unsigned int i = (vtkParse_HashString(name, l) & j);

  /* each file type has its own hash table */
  if (type == VTK_PARSE_ISFILE)
  {
    htable_ref = &info->FileHashTable;
  }
  else if (type == VTK_PARSE_ISDIR)
  {
    htable_ref = &info->DirHashTable;
  }
  else
  {
    return;
  }

  /* check if the hash table has been allocated yet */
  htable = *htable_ref;
  if (htable == NULL)
  {
    /* create an empty hash table */
    htable = (const char***)malloc(FILE_HASH_TABLE_SIZE * sizeof(char**));
    for (j = 0; j < FILE_HASH_TABLE_SIZE; j++)
    {
      htable[j] = NULL;
    }
    *htable_ref = htable;
  }

  /* use the filename hash to find the row in the hash table */
  hptr = htable[i];
  if (hptr == NULL)
  {
    /* row needs to be allocated */
    hptr = (const char**)malloc(2 * sizeof(char*));
    /* add "name" to the hash table, followed by NULL */
    hptr[0] = name;
    hptr[1] = NULL;
    htable[i] = hptr;
  }
  else if (*hptr)
  {
    /* the rows exists, so let's see if "name" is already there */
    n = 0;
    do
    {
      if (strcmp(*hptr, name) == 0)
      {
        break;
      }
      n++;
      hptr++;
    } while (*hptr);

    /* if we did not find "name" in the hash table, we must add it */
    if (*hptr == NULL)
    {
      /* check if the row size (which is n+1) has reached a power of 2 */
      /* (the "+1" is needed for the NULL at the end of the row) */
      if ((n & (n + 1)) == 0)
      {
        /* double the allocated capacity each time the capacity is filled */
        htable[i] = (const char**)realloc((char**)(htable[i]), (2 * (n + 1)) * sizeof(char*));
        if (!htable[i])
        {
          fprintf(stderr, "memory allocation error vtkParseSystem.c:%d\n", __LINE__);
          exit(1);
        }
        hptr = &htable[i][n];
      }
      /* add "name" to the hash table, with NULL to mark end of row */
      *hptr++ = name;
      *hptr = NULL;
    }
  }
}

/**
 * Check if a file with the the given path is known and return its type:
 * VTK_PARSE_ISDIR, VTK_PARSE_ISFILE, or VTK_PARSE_NOFILE if not found.
 * The filename length must be provided (the name need not be terminated).
 */
static system_filetype_t system_file_cached(SystemInfo* info, const char* name, size_t l)
{
  const char*** htable = info->DirHashTable;
  const char** hptr;
  /* FILE_HASH_TABLE_SIZE will always be a power of two */
  unsigned int m = FILE_HASH_TABLE_SIZE - 1; /* bit mask */
  unsigned int i = (vtkParse_HashString(name, l) & m);
  system_filetype_t type = VTK_PARSE_ISDIR;
  int j;

  /* loop twice: do DirHashTable, then do FileHashTable */
  for (j = 0; j < 2; j++)
  {
    if (htable && ((hptr = htable[i]) != NULL) && *hptr)
    {
      do
      {
        if (strncmp(*hptr, name, l) == 0 && (*hptr)[l] == '\0')
        {
          return type;
        }
        hptr++;
      } while (*hptr);
    }

    /* for second loop, use FileHashTable */
    htable = info->FileHashTable;
    type = VTK_PARSE_ISFILE;
  }

  /* failed to find the file */
  return VTK_PARSE_NOFILE;
}

/**
 * Check if a file with the given name exists and return its type:
 * VTK_PARSE_ISDIR, VTK_PARSE_ISFILE, or VTK_PARSE_NOFILE if not found.
 * This will cache results for the entire directory in order to
 * accelerate later searches within the same directory.
 */
system_filetype_t vtkParse_FileExists(SystemInfo* info, const char* name)
{
  size_t l = strlen(name);
  size_t n;
  const char* dirname;
  char* fullname;
  system_filetype_t type;
  system_filetype_t result;
  struct stat fs;
#if defined(_WIN32)
  HANDLE dirhandle;
  WIN32_FIND_DATA entry;
#else
  DIR* dirhandle;
  system_dirent* entry;
#endif

  /* if there is no cache, then go directly to the filesystem */
  if (!info)
  {
    if (stat(name, &fs) == 0)
    {
#if defined(_WIN32) && !defined(__CYGWIN__)
      if ((fs.st_mode & _S_IFMT) == _S_IFDIR)
#else
      if (S_ISDIR(fs.st_mode))
#endif
      {
        return VTK_PARSE_ISDIR;
      }
      return VTK_PARSE_ISFILE;
    }
    return VTK_PARSE_NOFILE;
  }

  /* check if the file is already cached */
  result = system_file_cached(info, name, l);
  if (result != VTK_PARSE_NOFILE)
  {
    return result;
  }

  /* get the dirname by locating the last slash */
  for (n = 0; name[n] != '\0'; n++)
  {
    if (system_path_separator(name[n]))
    {
      l = n;
    }
  }

  /* check if the directory is already cached */
  if (system_file_cached(info, name, l) == VTK_PARSE_ISDIR)
  {
    /* we've already cached this dir, and it didn't contain the file */
    return VTK_PARSE_NOFILE;
  }

  /* add dirname to the cache */
  dirname = vtkParse_CacheString(info->Strings, name, l);
  system_file_add(info, dirname, VTK_PARSE_ISDIR);

  /* advance to just after the slash */
  if (system_path_separator(name[l]))
  {
    l++;
  }

#if defined(_WIN32)
  /* replace the filename with "*" to search the whole directory */
  fullname = vtkParse_NewString(info->Strings, l + 1);
  memcpy(fullname, name, l);
  strcpy(&fullname[l], "*");
  /* if backslash was used, replace with forward slash */
  if (fullname[l] == '\\')
  {
    fullname[l] = '/';
  }

  /* begin the search, using local text encoding */
  dirhandle = FindFirstFile(fullname, &entry);
  if (dirhandle != INVALID_HANDLE_VALUE)
  {
    do
    {
      if (entry.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
      {
        type = VTK_PARSE_ISDIR;
      }
      else
      {
        /* construct full path for this entry, and add it to cache */
        n = l + strlen(entry.cFileName);
        fullname = vtkParse_NewString(info->Strings, n);
        memcpy(fullname, name, l);
        strcpy(&fullname[l], entry.cFileName);
        type = VTK_PARSE_ISFILE;
        system_file_add(info, fullname, type);
      }

      /* check if this directory entry is the file we are looking for */
      if (result == VTK_PARSE_NOFILE && strcmp(entry.cFileName, &name[l]) == 0)
      {
        result = type;
      }

    } while (FindNextFile(dirhandle, &entry));
    FindClose(dirhandle);
  }
#elif defined(HAVE_DIRENT_D_TYPE)
  dirhandle = opendir(dirname);
  if (dirhandle)
  {
    for (entry = readdir(dirhandle); entry; entry = readdir(dirhandle))
    {
      if (entry->d_type == DT_DIR)
      {
        type = VTK_PARSE_ISDIR;
      }
      else
      {
        /* construct full path for this entry, and add it to cache */
        n = l + strlen(entry->d_name);
        fullname = vtkParse_NewString(info->Strings, n);
        memcpy(fullname, name, l);
        strcpy(&fullname[l], entry->d_name);
        type = VTK_PARSE_ISFILE;
        system_file_add(info, fullname, type);
      }

      /* check if this directory entry is the file we are looking for */
      if (result == VTK_PARSE_NOFILE && strcmp(entry->d_name, &name[l]) == 0)
      {
        result = type;
      }
    }
    closedir(dirhandle);
  }
#else
  /* use stat() to separate subdirectories from files */
  dirhandle = opendir(dirname);
  if (dirhandle)
  {
    for (entry = readdir(dirhandle); entry; entry = readdir(dirhandle))
    {
      /* construct full path for this entry */
      n = l + strlen(entry->d_name);
      fullname = vtkParse_NewString(info->Strings, n);
      memcpy(fullname, name, l);
      strcpy(&fullname[l], entry->d_name);

      if (stat(fullname, &fs) != 0)
      {
        type = VTK_PARSE_NOFILE;
      }
      else if (S_ISDIR(fs.st_mode))
      {
        type = VTK_PARSE_ISDIR;
      }
      else
      {
        type = VTK_PARSE_ISFILE;
        /* add the file to the cache */
        system_file_add(info, fullname, type);
      }

      /* check if this directory entry is the file we are looking for */
      if (result == VTK_PARSE_NOFILE && strcmp(entry->d_name, &name[l]) == 0)
      {
        result = type;
      }
    }
    closedir(dirhandle);
  }
#endif

  return result;
}

/**
 * Free the memory that the used to cache the files.
 */
VTKWRAPPINGTOOLS_EXPORT
void vtkParse_FreeFileCache(SystemInfo* info)
{
  unsigned int n = FILE_HASH_TABLE_SIZE;
  unsigned int i;

  if (info->FileHashTable)
  {
    for (i = 0; i < n; i++)
    {
      free((char**)info->FileHashTable[i]);
    }
    free((char***)info->FileHashTable);
  }

  if (info->DirHashTable)
  {
    for (i = 0; i < n; i++)
    {
      free((char**)info->DirHashTable[i]);
    }
    free((char***)info->DirHashTable);
  }

  info->Strings = NULL;
  info->FileHashTable = NULL;
  info->DirHashTable = NULL;
}
