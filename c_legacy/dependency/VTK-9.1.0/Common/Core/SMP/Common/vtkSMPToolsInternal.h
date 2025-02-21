/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkSMPToolsInternal.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef vtkSMPToolsInternal_h
#define vtkSMPToolsInternal_h

#include <iterator> // For std::advance

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace vtk
{
namespace detail
{
namespace smp
{

template <typename InputIt, typename OutputIt, typename Functor>
class UnaryTransformCall
{
protected:
  InputIt In;
  OutputIt Out;
  Functor& Transform;

public:
  UnaryTransformCall(InputIt _in, OutputIt _out, Functor& _transform)
    : In(_in)
    , Out(_out)
    , Transform(_transform)
  {
  }

  void Execute(vtkIdType begin, vtkIdType end)
  {
    InputIt itIn(In);
    OutputIt itOut(Out);
    std::advance(itIn, begin);
    std::advance(itOut, begin);
    for (vtkIdType it = begin; it < end; it++)
    {
      *itOut = Transform(*itIn);
      ++itIn;
      ++itOut;
    }
  }
};

template <typename InputIt1, typename InputIt2, typename OutputIt, typename Functor>
class BinaryTransformCall : public UnaryTransformCall<InputIt1, OutputIt, Functor>
{
  InputIt2 In2;

public:
  BinaryTransformCall(InputIt1 _in1, InputIt2 _in2, OutputIt _out, Functor& _transform)
    : UnaryTransformCall<InputIt1, OutputIt, Functor>(_in1, _out, _transform)
    , In2(_in2)
  {
  }

  void Execute(vtkIdType begin, vtkIdType end)
  {
    InputIt1 itIn1(this->In);
    InputIt2 itIn2(In2);
    OutputIt itOut(this->Out);
    std::advance(itIn1, begin);
    std::advance(itIn2, begin);
    std::advance(itOut, begin);
    for (vtkIdType it = begin; it < end; it++)
    {
      *itOut = this->Transform(*itIn1, *itIn2);
      ++itIn1;
      ++itIn2;
      ++itOut;
    }
  }
};

template <typename T>
struct FillFunctor
{
  const T& Value;

public:
  FillFunctor(const T& _value)
    : Value(_value)
  {
  }

  T operator()(T vtkNotUsed(inValue)) { return Value; }
};

} // namespace smp
} // namespace detail
} // namespace vtk
#endif // DOXYGEN_SHOULD_SKIP_THIS

#endif
