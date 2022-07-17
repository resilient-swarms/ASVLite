/*=========================================================================

  Program:   Visualization Toolkit
  Module:    TestBitArray.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkBitArray.h"

#include "vtkNew.h"

#include <bitset>
#include <string>

// This test makes sure that the unsused reachable bits of the last byte are set
// to zero.
int TestBitArray(int, char*[])
{
  vtkNew<vtkBitArray> array;

  array->SetNumberOfComponents(1);
  array->SetNumberOfValues(1);

  // [1]
  array->SetValue(0, 1);
  unsigned char* data = static_cast<unsigned char*>(array->GetVoidPointer(0));
  std::string str;
  str = std::bitset<8>(data[0]).to_string();
  if (str != "10000000")
  {
    std::cerr << "Bit array not initialized as expected. The raw data is " << str
              << ", it should be 10000000" << std::endl;
    return EXIT_FAILURE;
  }

  array->SetNumberOfValues(0);

  // [1111 1011 | 101]
  array->InsertValue(0, 1);
  array->InsertNextValue(1);
  array->InsertNextValue(1);
  array->InsertNextValue(1);
  array->InsertNextValue(1);
  array->InsertNextValue(0);
  array->InsertNextValue(1);
  array->InsertNextValue(1);
  array->InsertNextValue(1);
  array->InsertNextValue(0);
  array->InsertNextValue(1);

  data = static_cast<unsigned char*>(array->GetVoidPointer(0));
  str = std::bitset<8>(data[0]).to_string() + " " + std::bitset<8>(data[1]).to_string();
  if (str != std::string("11111011 10100000"))
  {
    std::cerr << "Bit array not initialized as expected. The raw data is " << str
              << ", it should be 11111011 10100000" << std::endl;
    return EXIT_FAILURE;
  }

  unsigned char* ptr = array->WritePointer(0, 18);
  // [1111 1011 | 1111 0011 | 10]
  ptr[1] = 0xf3;
  ptr[2] = (ptr[2] & 0x3f) | 0x80;
  data = static_cast<unsigned char*>(array->GetVoidPointer(0));
  str = std::bitset<8>(data[0]).to_string() + std::string(" ") +
    std::bitset<8>(data[1]).to_string() + std::string(" ") + std::bitset<8>(data[2]).to_string();
  if (str != "11111011 11110011 10000000")
  {
    std::cerr << "Bit array not initialized as expected. The raw data is " << str
              << ", it should be 11111011 10110011 10000000" << std::endl;
    return EXIT_FAILURE;
  }

  array->Resize(2);
  data = static_cast<unsigned char*>(array->GetVoidPointer(0));
  str = std::bitset<8>(data[0]).to_string();
  if (str != "11000000")
  {
    std::cerr << "Bit array not initialized as expected. The raw data is " << str
              << ", it should be 11000000" << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
