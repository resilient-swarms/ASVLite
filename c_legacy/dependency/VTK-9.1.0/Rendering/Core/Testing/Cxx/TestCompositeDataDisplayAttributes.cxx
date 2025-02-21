/*=========================================================================

  Program:   Visualization Toolkit

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkCompositeDataDisplayAttributes.h"
#include "vtkLogger.h"
#include "vtkMultiBlockDataSet.h"
#include "vtkMultiPieceDataSet.h"
#include "vtkNew.h"
#include "vtkTestUtilities.h"
#include "vtkXMLMultiBlockDataReader.h"

#define Verify(x)                                                                                  \
  if (!(x))                                                                                        \
  {                                                                                                \
    vtkLogF(ERROR, "check failed for " #x);                                                        \
    return EXIT_FAILURE;                                                                           \
  }
int TestCompositeDataDisplayAttributes(int argc, char* argv[])
{
  char* fname = vtkTestUtilities::ExpandDataFileName(argc, argv, "Data/mb_with_pieces.vtm");
  vtkNew<vtkXMLMultiBlockDataReader> reader;
  reader->SetFileName(fname);
  delete[] fname;
  reader->Update();

  // change structure a little to have null pieces.
  auto mb = vtkMultiBlockDataSet::SafeDownCast(reader->GetOutput());
  auto block1 = vtkMultiBlockDataSet::SafeDownCast(mb->GetBlock(0));
  auto block2 = vtkMultiPieceDataSet::SafeDownCast(block1->GetBlock(0));
  block2->SetPiece(1, nullptr);

  auto block5 = vtkMultiBlockDataSet::SafeDownCast(mb->GetBlock(1));
  auto block6 = vtkMultiPieceDataSet::SafeDownCast(block5->GetBlock(0));
  block6->SetPiece(1, nullptr);

  auto block9 = vtkMultiPieceDataSet::SafeDownCast(block5->GetBlock(1));
  block9->SetPiece(1, nullptr);

  vtkNew<vtkCompositeDataDisplayAttributes> cdda;
  Verify(cdda->DataObjectFromIndex(0, mb) == mb);
  Verify(cdda->DataObjectFromIndex(5, mb) == block5);
  Verify(cdda->DataObjectFromIndex(9, mb) == block9);
  return EXIT_SUCCESS;
}
