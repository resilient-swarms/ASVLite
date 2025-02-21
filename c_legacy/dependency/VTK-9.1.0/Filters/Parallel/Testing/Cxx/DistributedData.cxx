/*=========================================================================

  Program:   Visualization Toolkit
  Module:    DistributedData.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// Test of vtkDistributedDataFilter and supporting classes, covering as much
// code as possible.  This test requires 4 MPI processes.
//
// To cover ghost cell creation, use vtkDataSetSurfaceFilter.
//
// To cover clipping code:  SetBoundaryModeToSplitBoundaryCells()
//
// To run fast redistribution: SetUseMinimalMemoryOff() (Default)
// To run memory conserving code instead: SetUseMinimalMemoryOn()

#include "vtkActor.h"
#include "vtkCamera.h"
#include "vtkCellTypeSource.h"
#include "vtkCompositeRenderManager.h"
#include "vtkDataSetReader.h"
#include "vtkDataSetSurfaceFilter.h"
#include "vtkDistributedDataFilter.h"
#include "vtkFloatArray.h"
#include "vtkMPIController.h"
#include "vtkObjectFactory.h"
#include "vtkPieceScalars.h"
#include "vtkPointData.h"
#include "vtkPolyDataMapper.h"
#include "vtkRegressionTestImage.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkTestUtilities.h"
#include "vtkUnstructuredGrid.h"
/*
** This test only builds if MPI is in use
*/
#include "vtkMPICommunicator.h"

#include "vtkProcess.h"

namespace
{

class MyProcess : public vtkProcess
{
public:
  static MyProcess* New();

  void Execute() override;

  void SetArgs(int anArgc, char* anArgv[]);

protected:
  MyProcess();

  int Argc;
  char** Argv;
};

vtkStandardNewMacro(MyProcess);

MyProcess::MyProcess()
{
  this->Argc = 0;
  this->Argv = nullptr;
}

void MyProcess::SetArgs(int anArgc, char* anArgv[])
{
  this->Argc = anArgc;
  this->Argv = anArgv;
}

void MyProcess::Execute()
{
  this->ReturnValue = 1;
  int numProcs = this->Controller->GetNumberOfProcesses();
  int me = this->Controller->GetLocalProcessId();

  int i, go;

  vtkCompositeRenderManager* prm = vtkCompositeRenderManager::New();

  // READER

  vtkDataSetReader* dsr = vtkDataSetReader::New();
  vtkUnstructuredGrid* ug = vtkUnstructuredGrid::New();

  vtkDataSet* ds = nullptr;

  if (me == 0)
  {
    char* fname =
      vtkTestUtilities::ExpandDataFileName(this->Argc, this->Argv, "Data/tetraMesh.vtk");

    dsr->SetFileName(fname);

    ds = dsr->GetOutput();

    dsr->Update();

    vtkFloatArray* fa = vtkFloatArray::New();
    fa->SetName("ones");
    fa->SetNumberOfTuples(ds->GetNumberOfPoints());
    fa->FillComponent(0, 1);
    ds->GetPointData()->AddArray(fa);
    fa->Delete();

    delete[] fname;

    go = 1;

    if ((ds == nullptr) || (ds->GetNumberOfCells() == 0))
    {
      if (ds)
      {
        cout << "Failure: input file has no cells" << endl;
      }
      go = 0;
    }
  }
  else
  {
    ds = static_cast<vtkDataSet*>(ug);
  }

  vtkMPICommunicator* comm = vtkMPICommunicator::SafeDownCast(this->Controller->GetCommunicator());

  comm->Broadcast(&go, 1, 0);

  if (!go)
  {
    dsr->Delete();
    ug->Delete();
    prm->Delete();
    return;
  }

  // DATA DISTRIBUTION FILTER

  vtkDistributedDataFilter* dd = vtkDistributedDataFilter::New();

  dd->SetInputData(ds);
  dd->SetController(this->Controller);

  dd->UseMinimalMemoryOff();
  dd->SetBoundaryModeToSplitBoundaryCells(); // clipping

  // COLOR BY PROCESS NUMBER

  vtkPieceScalars* ps = vtkPieceScalars::New();
  ps->SetInputConnection(dd->GetOutputPort());
  ps->SetScalarModeToCellData();

  // MORE FILTERING - this will request ghost cells

  vtkDataSetSurfaceFilter* dss = vtkDataSetSurfaceFilter::New();
  dss->SetPieceInvariant(1);
  dss->SetInputConnection(ps->GetOutputPort());

  // COMPOSITE RENDER

  vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
  mapper->SetInputConnection(dss->GetOutputPort());

  mapper->SetColorModeToMapScalars();
  mapper->SetScalarModeToUseCellFieldData();
  mapper->SelectColorArray("Piece");
  mapper->SetScalarRange(0, numProcs - 1);

  vtkActor* actor = vtkActor::New();
  actor->SetMapper(mapper);

  vtkRenderer* renderer = prm->MakeRenderer();
  renderer->AddActor(actor);

  vtkRenderWindow* renWin = prm->MakeRenderWindow();
  renWin->AddRenderer(renderer);

  renderer->SetBackground(0, 0, 0);
  renWin->SetSize(300, 300);
  renWin->SetPosition(0, 360 * me);

  prm->SetRenderWindow(renWin);
  prm->SetController(this->Controller);

  prm->InitializeOffScreen(); // Mesa GL only

  // Test the minimum ghost cell settings:
  bool ghostCellSuccess = true;
  {
    dd->UseMinimalMemoryOn();
    dd->SetBoundaryModeToAssignToOneRegion();

    dd->SetMinimumGhostLevel(0);
    dd->Update();
    int ncells = static_cast<vtkUnstructuredGrid*>(dd->GetOutput())->GetNumberOfCells();
    if (me == 0 && ncells != 79)
    {
      std::cerr << "Invalid number of cells for ghost level 0: " << ncells << "\n";
      ghostCellSuccess = false;
    }

    dd->SetMinimumGhostLevel(2);
    dd->Update();
    ncells = static_cast<vtkUnstructuredGrid*>(dd->GetOutput())->GetNumberOfCells();
    if (me == 0 && ncells != 160)
    {
      std::cerr << "Invalid number of cells for ghost level 2: " << ncells << "\n";
      ghostCellSuccess = false;
    }

    dd->SetMinimumGhostLevel(0);
    dd->UseMinimalMemoryOff();
    dd->SetBoundaryModeToSplitBoundaryCells(); // clipping
  }

  // We must update the whole pipeline here, otherwise node 0
  // goes into GetActiveCamera which updates the pipeline, putting
  // it into vtkDistributedDataFilter::Execute() which then hangs.
  // If it executes here, dd will be up-to-date won't have to
  // execute in GetActiveCamera.

  mapper->SetPiece(me);
  mapper->SetNumberOfPieces(numProcs);
  mapper->Update();

  const int MY_RETURN_VALUE_MESSAGE = 0x11;

  if (me == 0)
  {
    renderer->ResetCamera();
    vtkCamera* camera = renderer->GetActiveCamera();
    // camera->UpdateViewport(renderer);
    camera->ParallelProjectionOn();
    camera->SetParallelScale(16);

    dd->UseMinimalMemoryOn();
    dd->SetBoundaryModeToAssignToOneRegion();

    renWin->Render();
    renWin->Render();

    int ncells = vtkUnstructuredGrid::SafeDownCast(dd->GetOutput())->GetNumberOfCells();

    prm->StopServices();

    dd->UseMinimalMemoryOff();
    dd->SetBoundaryModeToSplitBoundaryCells(); // clipping

    this->ReturnValue = vtkRegressionTester::Test(this->Argc, this->Argv, renWin, 10);

    if (this->ReturnValue == vtkTesting::PASSED && !ghostCellSuccess)
    {
      this->ReturnValue = vtkTesting::FAILED;
    }

    if (ncells != 152)
    {
      this->ReturnValue = vtkTesting::FAILED;
    }
    for (i = 1; i < numProcs; i++)
    {
      this->Controller->Send(&this->ReturnValue, 1, i, MY_RETURN_VALUE_MESSAGE);
    }

    prm->StopServices();
  }
  else
  {
    dd->UseMinimalMemoryOn();
    dd->SetBoundaryModeToAssignToOneRegion();

    prm->StartServices();

    dd->UseMinimalMemoryOff();
    dd->SetBoundaryModeToSplitBoundaryCells(); // clipping

    prm->StartServices();
    this->Controller->Receive(&this->ReturnValue, 1, 0, MY_RETURN_VALUE_MESSAGE);
  }

  // CLEAN UP

  mapper->Delete();
  actor->Delete();
  renderer->Delete();
  renWin->Delete();

  dd->Delete();
  dsr->Delete();
  ug->Delete();

  ps->Delete();
  dss->Delete();

  prm->Delete();
}

class MyProcess2 : public vtkProcess
{
public:
  static MyProcess2* New();

  void Execute() override;
  void SetArgs(int anArgc, char* anArgv[]);

protected:
  MyProcess2();

  int Argc;
  char** Argv;
};

vtkStandardNewMacro(MyProcess2);

MyProcess2::MyProcess2()
{
  this->Argc = 0;
  this->Argv = nullptr;
}

void MyProcess2::SetArgs(int anArgc, char* anArgv[])
{
  this->Argc = anArgc;
  this->Argv = anArgv;
}

void MyProcess2::Execute()
{
  int me = this->Controller->GetLocalProcessId();

  // generate 1 cell in proc 0 and no cells on other procs
  vtkUnstructuredGrid* input = nullptr;
  if (me == 0)
  {
    vtkNew<vtkCellTypeSource> source;
    source->SetCellType(VTK_HEXAHEDRON);
    source->SetBlocksDimensions(1, 1, 1);
    source->Update();

    input = source->GetOutput();
    input->Register(nullptr);
  }
  else
  {
    input = vtkUnstructuredGrid::New();
  }

  vtkNew<vtkDistributedDataFilter> dd;
  dd->SetInputData(input);
  dd->SetController(this->Controller);
  dd->Update();

  input->Delete();

  // compute total number of cells
  vtkIdType nbCellsTot = 0;
  vtkUnstructuredGrid* output = vtkUnstructuredGrid::SafeDownCast(dd->GetOutput());
  vtkIdType nbCells = output->GetNumberOfCells();
  this->Controller->AllReduce(&nbCells, &nbCellsTot, 1, vtkCommunicator::SUM_OP);

  // number of cells should be 1
  this->ReturnValue = (nbCellsTot == 1);
}

}

int DistributedData(int argc, char* argv[])
{
  int retVal = 1;

  vtkMPIController* contr = vtkMPIController::New();
  contr->Initialize(&argc, &argv);

  vtkMultiProcessController::SetGlobalController(contr);

  int numProcs = contr->GetNumberOfProcesses();
  int me = contr->GetLocalProcessId();

  if (numProcs != 2)
  {
    if (me == 0)
    {
      cout << "DistributedData test requires 2 processes" << endl;
    }
    contr->Delete();
    return retVal;
  }

  if (!contr->IsA("vtkMPIController"))
  {
    if (me == 0)
    {
      cout << "DistributedData test requires MPI" << endl;
    }
    contr->Delete();
    return retVal; // is this the right error val?   TODO
  }

  MyProcess* p = MyProcess::New();
  p->SetArgs(argc, argv);
  contr->SetSingleProcessObject(p);
  contr->SingleMethodExecute();

  retVal = p->GetReturnValue();
  p->Delete();

  // test special case 'numCells < numProcs'
  vtkNew<MyProcess2> p2;
  p2->SetArgs(argc, argv);
  contr->SetSingleProcessObject(p2.Get());
  contr->SingleMethodExecute();
  if (retVal == vtkTesting::PASSED)
  {
    retVal = p2->GetReturnValue();
  }

  contr->Finalize();
  contr->Delete();

  return !retVal;
}
