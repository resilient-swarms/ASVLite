/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkWrapPythonType.c

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkWrapPythonType.h"
#include "vtkWrapPythonClass.h"
#include "vtkWrapPythonConstant.h"
#include "vtkWrapPythonEnum.h"
#include "vtkWrapPythonMethod.h"
#include "vtkWrapPythonMethodDef.h"
#include "vtkWrapPythonTemplate.h"

#include "vtkParseExtras.h"
#include "vtkWrap.h"
#include "vtkWrapText.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------------------- */
/* A struct for special types to store info about the type, it is fairly
 * small because not many operators or special features are wrapped */
typedef struct
{
  int has_print;    /* there is "<<" stream operator */
  int has_compare;  /* there are comparison operators e.g. "<" */
  int has_sequence; /* the [] operator takes a single integer */
} SpecialTypeInfo;

/* -------------------------------------------------------------------- */
/* The following functions are for generating code for special types,
 * i.e. types that are not derived from vtkObjectBase */
/* -------------------------------------------------------------------- */

/* -------------------------------------------------------------------- */
/* check if class has a wrapped constructor, and return its name if so */
static const char* vtkWrapPython_WrappedConstructor(
  ClassInfo* data, HierarchyInfo* hinfo, size_t* np)
{
  const char* constructor = data->Name;
  size_t n, m;
  int i;

  /* remove namespaces and template parameters from the
   * class name to get the constructor name */
  m = vtkParse_UnscopedNameLength(constructor);
  while (constructor[m] == ':' && constructor[m + 1] == ':')
  {
    constructor += m + 2;
    m = vtkParse_UnscopedNameLength(constructor);
  }
  for (n = 0; n < m; n++)
  {
    if (constructor[n] == '<')
    {
      break;
    }
  }

  /* check if a public constructor exists */
  for (i = 0; i < data->NumberOfFunctions; i++)
  {
    FunctionInfo* theFunc = data->Functions[i];

    if (theFunc->Name && strncmp(theFunc->Name, constructor, n) == 0 && theFunc->Name[n] == '\0' &&
      !theFunc->Template && vtkWrapPython_MethodCheck(data, theFunc, hinfo))
    {
      *np = n;
      return constructor;
    }
  }

  return NULL;
}

/* -------------------------------------------------------------------- */
/* generate function for printing a special object */
static void vtkWrapPython_NewDeleteProtocol(
  FILE* fp, const char* classname, ClassInfo* data, HierarchyInfo* hinfo)
{
  size_t n = 0;
  const char* constructor = NULL;

  if (!data->IsAbstract)
  {
    constructor = vtkWrapPython_WrappedConstructor(data, hinfo, &n);
  }

  /* the "new" method */
  if (constructor)
  {
    fprintf(fp,
      "static PyObject *\n"
      "Py%s_New(PyTypeObject *, PyObject *args, PyObject *kwds)\n"
      "{\n"
      "  if (kwds && PyDict_Size(kwds))\n"
      "  {\n"
      "    PyErr_SetString(PyExc_TypeError,\n"
      "                    \"this function takes no keyword arguments\");\n"
      "    return nullptr;\n"
      "  }\n"
      "\n"
      "  return Py%s_%*.*s(nullptr, args);\n"
      "}\n"
      "\n",
      classname, classname, (int)n, (int)n, constructor);
  }
  else
  {
    fprintf(fp,
      "static PyObject *\n"
      "Py%s_New(PyTypeObject * /*unused*/, PyObject * /*unused*/, PyObject * /*unused*/)\n"
      "{\n"
      "  PyErr_SetString(PyExc_TypeError,\n"
      "                  \"this class cannot be instantiated\");\n"
      "\n"
      "  return nullptr;\n"
      "}\n"
      "\n",
      classname);
  }

  /* the delete method */
  if (vtkWrap_HasPublicDestructor(data))
  {
    fprintf(fp,
      "static void Py%s_Delete(PyObject *self)\n"
      "{\n"
      "  PyVTKSpecialObject *obj = (PyVTKSpecialObject *)self;\n"
      "  delete static_cast<%s *>(obj->vtk_ptr);\n"
      "  PyObject_Del(self);\n"
      "}\n"
      "\n",
      classname, data->Name);
  }
  else
  {
    fprintf(fp,
      "static void Py%s_Delete(PyObject *self)\n"
      "{\n"
      "  PyObject_Del(self);\n"
      "}\n"
      "\n",
      classname);
  }
}

/* -------------------------------------------------------------------- */
/* generate function for printing a special object */
static void vtkWrapPython_PrintProtocol(
  FILE* fp, const char* classname, ClassInfo* data, FileInfo* finfo, SpecialTypeInfo* info)
{
  int i;
  FunctionInfo* func;

  /* look in the file for "operator<<" for printing */
  for (i = 0; i < finfo->Contents->NumberOfFunctions; i++)
  {
    func = finfo->Contents->Functions[i];
    if (func->Name && func->IsOperator && !func->IsDeleted && !func->IsExcluded &&
      strcmp(func->Name, "operator<<") == 0)
    {
      if (func->NumberOfParameters == 2 &&
        (func->Parameters[0]->Type & VTK_PARSE_UNQUALIFIED_TYPE) == VTK_PARSE_OSTREAM_REF &&
        vtkWrap_IsSpecialObject(func->Parameters[1]) &&
        !vtkWrap_IsNonConstRef(func->Parameters[1]) &&
        strcmp(func->Parameters[1]->Class, data->Name) == 0)
      {
        info->has_print = 1;
      }
    }
  }

  /* the str function */
  if (info->has_print)
  {
    fprintf(fp,
      "static PyObject *Py%s_String(PyObject *self)\n"
      "{\n"
      "  PyVTKSpecialObject *obj = (PyVTKSpecialObject *)self;\n"
      "  std::ostringstream os;\n"
      "  if (obj->vtk_ptr)\n"
      "  {\n"
      "    os << *static_cast<const %s *>(obj->vtk_ptr);\n"
      "  }\n"
      "  const std::string &s = os.str();\n"
      "  return PyString_FromStringAndSize(s.data(), s.size());\n"
      "}\n"
      "\n",
      classname, data->Name);
  }
}

/* -------------------------------------------------------------------- */
/* generate function for comparing special objects */
static void vtkWrapPython_RichCompareProtocol(
  FILE* fp, const char* classname, ClassInfo* data, FileInfo* finfo, SpecialTypeInfo* info)
{
  static const char* compare_consts[6] = { "Py_LT", "Py_LE", "Py_EQ", "Py_NE", "Py_GT", "Py_GE" };
  static const char* compare_tokens[6] = { "<", "<=", "==", "!=", ">", ">=" };
  int compare_ops = 0;
  int i, n;
  FunctionInfo* func;

  /* look for comparison operator methods */
  n = data->NumberOfFunctions + finfo->Contents->NumberOfFunctions;
  for (i = 0; i < n; i++)
  {
    if (i < data->NumberOfFunctions)
    {
      /* member function */
      func = data->Functions[i];
      if (func->NumberOfParameters != 1 || !vtkWrap_IsSpecialObject(func->Parameters[0]) ||
        strcmp(func->Parameters[0]->Class, data->Name) != 0)
      {
        continue;
      }
    }
    else
    {
      /* non-member function: both args must be of our type */
      func = finfo->Contents->Functions[i - data->NumberOfFunctions];
      if (func->NumberOfParameters != 2 || !vtkWrap_IsSpecialObject(func->Parameters[0]) ||
        strcmp(func->Parameters[0]->Class, data->Name) != 0 ||
        !vtkWrap_IsSpecialObject(func->Parameters[1]) ||
        strcmp(func->Parameters[1]->Class, data->Name) != 0)
      {
        continue;
      }
    }
    if (func->IsOperator && func->Name != NULL && !func->IsDeleted && !func->IsExcluded)
    {
      if (strcmp(func->Name, "operator<") == 0)
      {
        compare_ops = (compare_ops | (1 << 0));
      }
      else if (strcmp(func->Name, "operator<=") == 0)
      {
        compare_ops = (compare_ops | (1 << 1));
      }
      else if (strcmp(func->Name, "operator==") == 0)
      {
        compare_ops = (compare_ops | (1 << 2));
      }
      else if (strcmp(func->Name, "operator!=") == 0)
      {
        compare_ops = (compare_ops | (1 << 3));
      }
      else if (strcmp(func->Name, "operator>") == 0)
      {
        compare_ops = (compare_ops | (1 << 4));
      }
      else if (strcmp(func->Name, "operator>=") == 0)
      {
        compare_ops = (compare_ops | (1 << 5));
      }
    }
  }

  /* the compare function */
  if (compare_ops != 0)
  {
    info->has_compare = 1;

    fprintf(fp, "static int Py%s_CheckExact(PyObject *ob);\n\n", classname);

    fprintf(fp,
      "static PyObject *Py%s_RichCompare(\n"
      "  PyObject *o1, PyObject *o2, int opid)\n"
      "{\n"
      "  PyObject *n1 = nullptr;\n"
      "  PyObject *n2 = nullptr;\n"
      "  const %s *so1 = nullptr;\n"
      "  const %s *so2 = nullptr;\n"
      "  int result = -1;\n"
      "\n",
      classname, data->Name, data->Name);

    for (i = 1; i <= 2; i++)
    {
      /* use GetPointerFromSpecialObject to do type conversion, but
       * at least one of the args will already be the correct type */
      fprintf(fp,
        "  if (Py%s_CheckExact(o%d))\n"
        "  {\n"
        "    PyVTKSpecialObject *s%d = (PyVTKSpecialObject *)o%d;\n"
        "    so%d = static_cast<const %s *>(s%d->vtk_ptr);\n"
        "  }\n"
        "  else\n"
        "  {\n"
        "    so%d = static_cast<const %s *>(\n"
        "      vtkPythonUtil::GetPointerFromSpecialObject(\n"
        "        o%d, \"%s\", &n%d));\n"
        "    if (so%d == nullptr)\n"
        "    {\n"
        "      PyErr_Clear();\n"
        "      Py_INCREF(Py_NotImplemented);\n"
        "      return Py_NotImplemented;\n"
        "    }\n"
        "  }\n"
        "\n",
        classname, i, i, i, i, data->Name, i, i, data->Name, i, classname, i, i);
    }

    /* the switch statement for all possible compare ops */
    fprintf(fp,
      "  switch (opid)\n"
      "  {\n");

    for (i = 0; i < 6; i++)
    {
      if (((compare_ops >> i) & 1) != 0)
      {
        fprintf(fp,
          "    case %s:\n"
          "      result = ((*so1) %s (*so2));\n"
          "      break;\n",
          compare_consts[i], compare_tokens[i]);
      }
      else
      {
        fprintf(fp,
          "    case %s:\n"
          "      break;\n",
          compare_consts[i]);
      }
    }

    fprintf(fp,
      "  }\n"
      "\n");

    /* delete temporary objects, there will be at most one */
    fprintf(fp,
      "  if (n1)\n"
      "  {\n"
      "    Py_DECREF(n1);\n"
      "  }\n"
      "  else if (n2)\n"
      "  {\n"
      "    Py_DECREF(n2);\n"
      "  }\n"
      "\n");

    /* return the result */
    fprintf(fp,
      "  if (result == -1)\n"
      "  {\n"
      "    PyErr_SetString(PyExc_TypeError, \"operation not available\");\n"
      "    return nullptr;\n"
      "  }\n"
      "\n"
      "  // avoids aliasing issues with Py_INCREF(Py_False)\n"
      "  return PyBool_FromLong((long)result);\n"
      "}\n"
      "\n");
  }
}

/* -------------------------------------------------------------------- */
/* generate functions for indexing into special objects */
static void vtkWrapPython_SequenceProtocol(
  FILE* fp, const char* classname, ClassInfo* data, HierarchyInfo* hinfo, SpecialTypeInfo* info)
{
  int i;
  FunctionInfo* func;
  FunctionInfo* getItemFunc = 0;
  FunctionInfo* setItemFunc = 0;

  /* look for [] operator */
  for (i = 0; i < data->NumberOfFunctions; i++)
  {
    func = data->Functions[i];

    if (func->Name && func->IsOperator && !func->IsDeleted && !func->IsExcluded &&
      strcmp(func->Name, "operator[]") == 0 && vtkWrapPython_MethodCheck(data, func, hinfo))
    {
      if (func->NumberOfParameters == 1 && func->ReturnValue &&
        vtkWrap_IsInteger(func->Parameters[0]))
      {
        if (!setItemFunc && vtkWrap_IsNonConstRef(func->ReturnValue))
        {
          setItemFunc = func;
        }
        if (!getItemFunc || (func->IsConst && !getItemFunc->IsConst))
        {
          getItemFunc = func;
        }
      }
    }
  }

  if (getItemFunc && getItemFunc->SizeHint)
  {
    info->has_sequence = 1;

    fprintf(fp,
      "Py_ssize_t Py%s_SequenceSize(PyObject *self)\n"
      "{\n"
      "  void *vp = vtkPythonArgs::GetSelfSpecialPointer(self);\n"
      "  %s *op = static_cast<%s *>(vp);\n"
      "\n"
      "  return static_cast<Py_ssize_t>(op->%s);\n"
      "}\n\n",
      classname, data->Name, data->Name, getItemFunc->SizeHint);

    fprintf(fp,
      "PyObject *Py%s_SequenceItem(PyObject *self, Py_ssize_t i)\n"
      "{\n"
      "  void *vp = vtkPythonArgs::GetSelfSpecialPointer(self);\n"
      "  %s *op = static_cast<%s *>(vp);\n"
      "\n",
      classname, data->Name, data->Name);

    vtkWrapPython_DeclareVariables(fp, data, getItemFunc);

    fprintf(fp,
      "  temp0 = static_cast<%s>(i);\n"
      "\n"
      "  if (temp0 < 0 || temp0 >= op->%s)\n"
      "  {\n"
      "    PyErr_SetString(PyExc_IndexError, \"index out of range\");\n"
      "  }\n"
      "  else\n"
      "  {\n",
      vtkWrap_GetTypeName(getItemFunc->Parameters[0]), getItemFunc->SizeHint);

    fprintf(fp, "  ");
    vtkWrap_DeclareVariable(
      fp, data, getItemFunc->ReturnValue, "tempr", -1, VTK_WRAP_RETURN | VTK_WRAP_NOSEMI);

    fprintf(fp,
      " = %s(*op)[temp0];\n"
      "\n",
      (vtkWrap_IsRef(getItemFunc->ReturnValue) ? "&" : ""));

    vtkWrapPython_ReturnValue(fp, data, getItemFunc->ReturnValue, 1);

    fprintf(fp,
      "  }\n"
      "\n"
      "  return result;\n"
      "}\n\n");

    if (setItemFunc)
    {
      fprintf(fp,
        "int Py%s_SequenceSetItem(\n"
        "  PyObject *self, Py_ssize_t i, PyObject *arg1)\n"
        "{\n"
        "  void *vp = vtkPythonArgs::GetSelfSpecialPointer(self);\n"
        "  %s *op = static_cast<%s *>(vp);\n"
        "\n",
        classname, data->Name, data->Name);

      vtkWrap_DeclareVariable(fp, data, setItemFunc->Parameters[0], "temp", 0, VTK_WRAP_ARG);
      vtkWrap_DeclareVariable(fp, data, setItemFunc->ReturnValue, "temp", 1, VTK_WRAP_ARG);

      fprintf(fp,
        "  int result = -1;\n"
        "\n"
        "  temp0 = static_cast<%s>(i);\n"
        "\n"
        "  if (temp0 < 0 || temp0 >= op->%s)\n"
        "  {\n"
        "    PyErr_SetString(PyExc_IndexError, \"index out of range\");\n"
        "  }\n"
        "  else if (",
        vtkWrap_GetTypeName(setItemFunc->Parameters[0]), getItemFunc->SizeHint);

      vtkWrapPython_GetSingleArgument(fp, data, 1, setItemFunc->ReturnValue, 1);

      fprintf(fp,
        ")\n"
        "  {\n"
        "    (*op)[temp0] = %stemp1;\n"
        "\n",
        ((vtkWrap_IsRef(getItemFunc->ReturnValue) && vtkWrap_IsObject(getItemFunc->ReturnValue))
            ? "*"
            : ""));

      fprintf(fp,
        "    if (PyErr_Occurred() == nullptr)\n"
        "    {\n"
        "      result = 0;\n"
        "    }\n"
        "  }\n"
        "\n"
        "  return result;\n"
        "}\n\n");
    }

    fprintf(fp,
      "static PySequenceMethods Py%s_AsSequence = {\n"
      "  Py%s_SequenceSize, // sq_length\n"
      "  nullptr, // sq_concat\n"
      "  nullptr, // sq_repeat\n"
      "  Py%s_SequenceItem, // sq_item\n"
      "  nullptr, // sq_slice\n",
      classname, classname, classname);

    if (setItemFunc)
    {
      fprintf(fp, "  Py%s_SequenceSetItem, // sq_ass_item\n", classname);
    }
    else
    {
      fprintf(fp, "  nullptr, // sq_ass_item\n");
    }

    fprintf(fp,
      "  nullptr, // sq_ass_slice\n"
      "  nullptr, // sq_contains\n"
      "  nullptr, // sq_inplace_concat\n"
      "  nullptr, // sq_inplace_repeat\n"
      "};\n\n");
  }
}

/* -------------------------------------------------------------------- */
/* generate function for hashing special objects */
static void vtkWrapPython_HashProtocol(FILE* fp, const char* classname, ClassInfo* data)
{
  /* the hash function, defined only for specific types */
  fprintf(fp, "static Py_hash_t Py%s_Hash(PyObject *self)\n", classname);

  if (strcmp(data->Name, "vtkTimeStamp") == 0)
  {
    /* hash for vtkTimeStamp is just the timestamp itself */
    fprintf(fp,
      "{\n"
      "  PyVTKSpecialObject *obj = (PyVTKSpecialObject *)self;\n"
      "  const vtkTimeStamp *op = static_cast<const vtkTimeStamp *>(obj->vtk_ptr);\n"
      "  vtkMTimeType mtime = *op;\n"
      "  long h = (long)mtime;\n"
      "  if (h != -1) { return h; }\n"
      "  return -2;\n"
      "}\n"
      "\n");
  }
  else if (strcmp(data->Name, "vtkVariant") == 0)
  {
    /* hash for vtkVariant is cached to avoid recomputation, this is
     * safe because vtkVariant is an immutable object, and is necessary
     * because computing the hash for vtkVariant is very expensive */
    fprintf(fp,
      "{\n"
      "  PyVTKSpecialObject *obj = (PyVTKSpecialObject *)self;\n"
      "  const vtkVariant *op = static_cast<const vtkVariant *>(obj->vtk_ptr);\n"
      "  long h = obj->vtk_hash;\n"
      "  if (h != -1)\n"
      "  {\n"
      "    return h;\n"
      "  }\n"
      "  h = vtkPythonUtil::VariantHash(op);\n"
      "  obj->vtk_hash = h;\n"
      "  return h;\n"
      "}\n"
      "\n");
  }
  else
  {
    /* if hash is not implemented, raise an exception */
    fprintf(fp,
      "{\n"
      "  return PyObject_HashNotImplemented(self);\n"
      "}\n"
      "\n");
  }
}

/* -------------------------------------------------------------------- */
/* generate extra functions for a special object */
static void vtkWrapPython_SpecialTypeProtocols(FILE* fp, const char* classname, ClassInfo* data,
  FileInfo* finfo, HierarchyInfo* hinfo, SpecialTypeInfo* info)
{
  /* clear all info about the type */
  info->has_print = 0;
  info->has_compare = 0;
  info->has_sequence = 0;

  vtkWrapPython_NewDeleteProtocol(fp, classname, data, hinfo);
  vtkWrapPython_PrintProtocol(fp, classname, data, finfo, info);
  vtkWrapPython_RichCompareProtocol(fp, classname, data, finfo, info);
  vtkWrapPython_SequenceProtocol(fp, classname, data, hinfo, info);
  vtkWrapPython_HashProtocol(fp, classname, data);
}

/* -------------------------------------------------------------------- */
/* For classes that aren't derived from vtkObjectBase, check to see if
 * they are wrappable */
int vtkWrapPython_IsSpecialTypeWrappable(ClassInfo* data)
{
  /* wrapping templates is only possible after template instantiation */
  if (data->Template)
  {
    return 0;
  }

  /* restrict wrapping to classes that have a "vtk" prefix */
  if (strncmp(data->Name, "vtk", 3) != 0)
  {
    return 0;
  }

  return 1;
}

/* -------------------------------------------------------------------- */
/* write out a special type object */
void vtkWrapPython_GenerateSpecialType(FILE* fp, const char* module, const char* classname,
  ClassInfo* data, FileInfo* finfo, HierarchyInfo* hinfo)
{
  char supername[1024];
  const char* supermodule;
  const char* name;
  SpecialTypeInfo info;
  const char* constructor = NULL;
  size_t n = 0;
  int i;
  int has_constants = 0;
  int has_superclass = 0;
  int has_copycons = 0;

  /* remove namespaces and template parameters from the
   * class name to get the constructor name */
  if (!data->IsAbstract)
  {
    constructor = vtkWrapPython_WrappedConstructor(data, hinfo, &n);
  }

  /* get the superclass */
  name = vtkWrapPython_GetSuperClass(data, hinfo, &supermodule);
  if (name)
  {
    has_superclass = 1;
    vtkWrapText_PythonName(name, supername);
  }

  /* generate all constructor methods */
  if (constructor)
  {
    vtkWrapPython_GenerateMethods(fp, classname, data, finfo, hinfo, 0, 1);
  }

  /* the docstring for the class, as a static var ending in "Doc" */
  fprintf(fp, "\nstatic const char *Py%s_Doc =\n", classname);
  vtkWrapPython_ClassDoc(fp, finfo, data, hinfo, 0);
  fprintf(fp, ";\n\n");

  /* generate all functions and protocols needed for the type */
  vtkWrapPython_SpecialTypeProtocols(fp, classname, data, finfo, hinfo, &info);

  /* Generate the TypeObject */
  fprintf(fp,
    "#ifdef VTK_PYTHON_NEEDS_DEPRECATION_WARNING_SUPPRESSION\n"
    "#pragma GCC diagnostic ignored \"-Wdeprecated-declarations\"\n"
    "#endif\n"
    "\n"
    "static PyTypeObject Py%s_Type = {\n"
    "  PyVarObject_HEAD_INIT(&PyType_Type, 0)\n"
    "  PYTHON_PACKAGE_SCOPE \"%s.%s\", // tp_name\n"
    "  sizeof(PyVTKSpecialObject), // tp_basicsize\n"
    "  0, // tp_itemsize\n"
    "  Py%s_Delete, // tp_dealloc\n"
    "#if PY_VERSION_HEX >= 0x03080000\n"
    "  0, // tp_vectorcall_offset\n"
    "#else\n"
    "  nullptr, // tp_print\n"
    "#endif\n"
    "  nullptr, // tp_getattr\n"
    "  nullptr, // tp_setattr\n"
    "  nullptr, // tp_compare\n"
    "  PyVTKSpecialObject_Repr, // tp_repr\n",
    classname, module, classname, classname);

  fprintf(fp, "  nullptr, // tp_as_number\n");

  if (info.has_sequence)
  {
    fprintf(fp, "  &Py%s_AsSequence, // tp_as_sequence\n", classname);
  }
  else
  {
    fprintf(fp, "  nullptr, // tp_as_sequence\n");
  }

  fprintf(fp,
    "  nullptr, // tp_as_mapping\n"
    "  Py%s_Hash, // tp_hash\n"
    "  nullptr, // tp_call\n",
    classname);

  if (info.has_print)
  {
    fprintf(fp, "  Py%s_String, // tp_str\n", classname);
  }
  else if (info.has_sequence)
  {
    fprintf(fp, "  PyVTKSpecialObject_SequenceString, // tp_str\n");
  }
  else
  {
    fprintf(fp, "  nullptr, // tp_str\n");
  }

  fprintf(fp,
    "  PyObject_GenericGetAttr, // tp_getattro\n"
    "  nullptr, // tp_setattro\n"
    "  nullptr, // tp_as_buffer\n"
    "  Py_TPFLAGS_DEFAULT, // tp_flags\n"
    "  Py%s_Doc, // tp_doc\n"
    "  nullptr, // tp_traverse\n"
    "  nullptr, // tp_clear\n",
    classname);

  if (info.has_compare)
  {
    fprintf(fp, "  Py%s_RichCompare, // tp_richcompare\n", classname);
  }
  else
  {
    fprintf(fp, "  nullptr, // tp_richcompare\n");
  }

  fprintf(fp,
    "  0, // tp_weaklistoffset\n"
    "  nullptr, // tp_iter\n"
    "  nullptr, // tp_iternext\n"
    "  nullptr, // tp_methods\n"
    "  nullptr, // tp_members\n"
    "  nullptr, // tp_getset\n"
    "  nullptr, // tp_base\n"
    "  nullptr, // tp_dict\n"
    "  nullptr, // tp_descr_get\n"
    "  nullptr, // tp_descr_set\n"
    "  0, // tp_dictoffset\n"
    "  nullptr, // tp_init\n"
    "  nullptr, // tp_alloc\n"
    "  Py%s_New, // tp_new\n"
    "  PyObject_Del, // tp_free\n"
    "  nullptr, // tp_is_gc\n",
    classname);

  /* fields set by python itself */
  fprintf(fp,
    "  nullptr, // tp_bases\n"
    "  nullptr, // tp_mro\n"
    "  nullptr, // tp_cache\n"
    "  nullptr, // tp_subclasses\n"
    "  nullptr, // tp_weaklist\n");

  /* internal struct members */
  fprintf(fp,
    "  VTK_WRAP_PYTHON_SUPPRESS_UNINITIALIZED\n"
    "};\n"
    "\n");

  /* need a check function for some protocols */
  if (info.has_compare)
  {
    fprintf(fp,
      "static int Py%s_CheckExact(PyObject *ob)\n"
      "{\n"
      "  return (Py_TYPE(ob) == &Py%s_Type);\n"
      "}\n\n",
      classname, classname);
  }

  /* generate the copy constructor helper function */
  if (constructor && vtkWrap_HasPublicCopyConstructor(data))
  {
    has_copycons = 1;

    fprintf(fp,
      "static void *Py%s_CCopy(const void *obj)\n"
      "{\n"
      "  if (obj)\n"
      "  {\n"
      "    return new %s(*static_cast<const %s*>(obj));\n"
      "  }\n"
      "  return nullptr;\n"
      "}\n"
      "\n",
      classname, data->Name, data->Name);
  }

  /* export New method for use by subclasses */
  fprintf(fp,
    "#ifndef DECLARED_Py%s_TypeNew\n"
    "extern \"C\" { PyObject *Py%s_TypeNew(); }\n"
    "#define DECLARED_Py%s_TypeNew\n"
    "#endif\n\n",
    classname, classname, classname);

  /* import New method of the superclass */
  if (has_superclass && !supermodule)
  {
    fprintf(fp,
      "#ifndef DECLARED_Py%s_TypeNew\n"
      "extern \"C\" { PyObject *Py%s_TypeNew(); }\n"
      "#define DECLARED_Py%s_TypeNew\n"
      "#endif\n",
      supername, supername, supername);
  }

  /* the method for adding the VTK extras to the type,
   * the unused "const char *" arg is the module name */
  fprintf(fp,
    "PyObject *Py%s_TypeNew()\n"
    "{\n",
    classname);

  if (has_copycons)
  {
    fprintf(fp,
      "  PyTypeObject *pytype = PyVTKSpecialType_Add(\n"
      "    &Py%s_Type,\n"
      "    Py%s_Methods,\n"
      "    Py%s_%*.*s_Methods,\n"
      "    &Py%s_CCopy);\n"
      "\n",
      classname, classname, classname, (int)n, (int)n, constructor, classname);
  }
  else if (constructor)
  {
    fprintf(fp,
      "  PyTypeObject *pytype = PyVTKSpecialType_Add(\n"
      "    &Py%s_Type,\n"
      "    Py%s_Methods,\n"
      "    Py%s_%*.*s_Methods,\n"
      "    nullptr);\n"
      "\n",
      classname, classname, classname, (int)n, (int)n, constructor);
  }
  else
  {
    fprintf(fp,
      "  PyTypeObject *pytype = PyVTKSpecialType_Add(\n"
      "    &Py%s_Type,\n"
      "    Py%s_Methods,\n"
      "    nullptr,\n"
      "    nullptr);\n"
      "\n",
      classname, classname);
  }

  /* if type is already ready, then return */
  fprintf(fp,
    "  if ((pytype->tp_flags & Py_TPFLAGS_READY) != 0)\n"
    "  {\n"
    "    return (PyObject *)pytype;\n"
    "  }\n\n");

  /* call the superclass New (initialize in dependency order) */
  if (has_superclass)
  {
    if (!supermodule) /* superclass is in the same module */
    {
      fprintf(fp, "  pytype->tp_base = (PyTypeObject *)Py%s_TypeNew();\n\n", supername);
    }
    else /* superclass is in a different module */
    {
      fprintf(fp,
        "  pytype->tp_base = vtkPythonUtil::FindSpecialTypeObject("
        "\"%s\");\n\n",
        supername);
    }
  }

  /* check whether the class has any constants as members */
  for (i = 0; i < data->NumberOfConstants; i++)
  {
    if (data->Constants[i]->Access == VTK_ACCESS_PUBLIC)
    {
      has_constants = 1;
    }
  }

  if (has_constants)
  {
    fprintf(fp,
      "  PyObject *d = pytype->tp_dict;\n"
      "  PyObject *o;\n\n");

    /* add any enum types defined in the class to its dict */
    vtkWrapPython_AddPublicEnumTypes(fp, "  ", "d", "o", data);

    /* add any constants defined in the class to its dict */
    vtkWrapPython_AddPublicConstants(fp, "  ", "d", "o", data);
  }

  fprintf(fp,
    "  PyType_Ready(pytype);\n"
    "  return (PyObject *)pytype;\n"
    "}\n\n");
}
