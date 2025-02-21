/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPIOReader.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkPIOReader
 * @brief   class for reading PIO (Parallel Input Output) data files
 *
 * This class reads in dump files generated from xRage, a LANL physics code.
 * The PIO (Parallel Input Output) library is used to create the dump files.
 *
 * @sa
 * vtkMultiBlockReader
 */

#ifndef vtkPIOReader_h
#define vtkPIOReader_h

#include "vtkIOPIOModule.h" // For export macro
#include "vtkMultiBlockDataSetAlgorithm.h"
#include "vtkStdString.h" // for vtkStdString

class vtkCallbackCommand;
class vtkDataArraySelection;
class vtkFloatArray;
class vtkInformation;
class vtkMultiBlockDataSet;
class vtkMultiProcessController;
class vtkStringArray;

class PIOAdaptor;
class PIO_DATA;

class VTKIOPIO_EXPORT vtkPIOReader : public vtkMultiBlockDataSetAlgorithm
{
public:
  static vtkPIOReader* New();
  vtkTypeMacro(vtkPIOReader, vtkMultiBlockDataSetAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  ///@{
  /**
   * Specify file name of PIO data file to read.
   */
  vtkSetFilePathMacro(FileName);
  vtkGetFilePathMacro(FileName);
  ///@}

  ///@{
  /**
   * Specify the timestep to be loaded
   */
  vtkSetMacro(CurrentTimeStep, int);
  vtkGetMacro(CurrentTimeStep, int);
  ///@}

  ///@{
  /**
   * Specify the creation of hypertree grid
   */
  vtkGetMacro(HyperTreeGrid, bool);
  vtkSetMacro(HyperTreeGrid, bool);
  ///@}

  ///@{
  /**
   * Specify the creation of tracer data
   */
  vtkSetMacro(Tracers, bool);
  vtkGetMacro(Tracers, bool);
  ///@}

  ///@{
  /**
   * Specify the use of float64 for data
   */
  vtkSetMacro(Float64, bool);
  vtkGetMacro(Float64, bool);
  ///@}

  ///@{
  /**
   * Get the reader's output
   */
  vtkMultiBlockDataSet* GetOutput();
  vtkMultiBlockDataSet* GetOutput(int index);
  ///@}

  ///@{
  /**
   * The following methods allow selective reading of solutions fields.
   * By default, ALL data fields on the nodes are read, but this can
   * be modified.
   */
  int GetNumberOfCellArrays();
  const char* GetCellArrayName(int index);
  int GetCellArrayStatus(const char* name);
  void SetCellArrayStatus(const char* name, int status);
  void DisableAllCellArrays();
  void EnableAllCellArrays();
  vtkGetObjectMacro(CellDataArraySelection, vtkDataArraySelection);
  ///@}

  ///@{
  /**
   * Getters for time data array candidates.
   */
  int GetNumberOfTimeDataArrays() const;
  const char* GetTimeDataArray(int idx) const;
  vtkGetObjectMacro(TimeDataStringArray, vtkStringArray);
  ///@}

  ///@{
  /**
   * Setter / Getter on ActiveTimeDataArrayName. This string
   * holds the selected time array name. If set to `nullptr`,
   * time values are the sequence of positive integers starting at zero.
   */
  vtkGetStringMacro(ActiveTimeDataArrayName);
  vtkSetStringMacro(ActiveTimeDataArrayName);
  ///@}

protected:
  vtkPIOReader();
  ~vtkPIOReader() override;

  char* FileName; // First field part file giving path

  int Rank;      // Number of this processor
  int TotalRank; // Number of processors

  PIOAdaptor* pioAdaptor; // Adapts data format to VTK

  int NumberOfVariables; // Number of variables to display

  int NumberOfTimeSteps; // Temporal domain
  double* TimeSteps;     // Times available for request
  int CurrentTimeStep;   // Time currently displayed

  bool HyperTreeGrid; // Create HTG rather than UnstructuredGrid
  bool Tracers;       // Create UnstructuredGrid for tracer info
  bool Float64;       // Load variable data as 64 bit float

  // Controls initializing and querrying MPI
  vtkMultiProcessController* Controller;

  // Selected field of interest
  vtkDataArraySelection* CellDataArraySelection;

  // Time array selection
  vtkStringArray* TimeDataStringArray;

  // Active index of array used for time. If no time array is used, value should be -1.
  char* ActiveTimeDataArrayName;
  vtkStdString CurrentTimeDataArrayName;

  // Observer to modify this object when array selections are modified
  vtkCallbackCommand* SelectionObserver;

  int RequestData(vtkInformation*, vtkInformationVector**, vtkInformationVector*) override;
  int RequestInformation(
    vtkInformation*, vtkInformationVector** inVector, vtkInformationVector*) override;

  static void SelectionModifiedCallback(
    vtkObject* caller, unsigned long eid, void* clientdata, void* calldata);

private:
  vtkPIOReader(const vtkPIOReader&) = delete;
  void operator=(const vtkPIOReader&) = delete;
};

#endif
