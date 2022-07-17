/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkMPIMultiBlockPLOT3DReader.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkMPIMultiBlockPLOT3DReader
 * @brief   vtkMultiBlockPLOT3DReader subclass that
 * uses MPI-IO to efficiently read binary files for 3D domains in parallel using
 * MPI-IO.
 *
 * vtkMPIMultiBlockPLOT3DReader extends vtkMultiBlockPLOT3DReader to use MPI-IO
 * instead of POSIX IO to read file in parallel.
 */

#ifndef vtkMPIMultiBlockPLOT3DReader_h
#define vtkMPIMultiBlockPLOT3DReader_h

#include "vtkIOMPIParallelModule.h" // For export macro
#include "vtkMultiBlockPLOT3DReader.h"

class VTKIOMPIPARALLEL_EXPORT vtkMPIMultiBlockPLOT3DReader : public vtkMultiBlockPLOT3DReader
{
public:
  static vtkMPIMultiBlockPLOT3DReader* New();
  vtkTypeMacro(vtkMPIMultiBlockPLOT3DReader, vtkMultiBlockPLOT3DReader);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Use this to override using MPI-IO. When set to false (default is true),
   * this class will simply forward all method calls to the superclass.
   */
  vtkSetMacro(UseMPIIO, bool);
  vtkGetMacro(UseMPIIO, bool);
  vtkBooleanMacro(UseMPIIO, bool);
  ///@}

protected:
  vtkMPIMultiBlockPLOT3DReader();
  ~vtkMPIMultiBlockPLOT3DReader() override;

  /**
   * Determines we should use MPI-IO for the current file. We don't use MPI-IO
   * for 2D files or ASCII files.
   */
  bool CanUseMPIIO();

  int OpenFileForDataRead(void*& fp, const char* fname) override;
  void CloseFile(void* fp) override;

  int ReadIntScalar(void* vfp, int extent[6], int wextent[6], vtkDataArray* scalar,
    vtkTypeUInt64 offset, const vtkMultiBlockPLOT3DReaderRecord& currentRecord) override;
  int ReadScalar(void* vfp, int extent[6], int wextent[6], vtkDataArray* scalar,
    vtkTypeUInt64 offset, const vtkMultiBlockPLOT3DReaderRecord& currentRecord) override;
  int ReadVector(void* vfp, int extent[6], int wextent[6], int numDims, vtkDataArray* vector,
    vtkTypeUInt64 offset, const vtkMultiBlockPLOT3DReaderRecord& currentRecord) override;
  bool UseMPIIO;

private:
  vtkMPIMultiBlockPLOT3DReader(const vtkMPIMultiBlockPLOT3DReader&) = delete;
  void operator=(const vtkMPIMultiBlockPLOT3DReader&) = delete;
};

#endif
