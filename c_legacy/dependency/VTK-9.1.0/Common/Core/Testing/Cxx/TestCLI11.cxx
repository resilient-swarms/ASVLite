/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestCLI11.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkLogger.h"
#include <vtk_cli11.h>

int TestCLI11(int argc, char* argv[])
{

  CLI::App app("VTK-based Application");

  std::string file;
  auto opt = app.add_option("-f,--file,file", file, "File name");
  opt->required();

  int count{ 0 };
  auto copt = app.add_option("-c,--count", count, "Counter");

  int v{ 0 };
  auto flag = app.add_flag("--flag", v, "Some flag that can be passed multiple times");

  double value{ 0.0 }; // = 3.14;
  app.add_option("-d,--double", value, "Some Value");

  CLI11_PARSE(app, argc, argv);

  vtkLog(INFO,
    "Working on file: " << file << ", direct count: " << app.count("--file")
                        << ", opt count: " << opt->count());
  vtkLog(INFO,
    "Working on count: " << count << ", direct count: " << app.count("--count")
                         << ", opt count: " << copt->count());
  vtkLog(INFO, "Received flag: " << v << " (" << flag->count() << ") times\n");
  vtkLog(INFO, "Some value: " << value);
  return EXIT_SUCCESS;
}
