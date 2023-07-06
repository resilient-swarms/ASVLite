//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//============================================================================

#ifndef vtk_m_worklet_waveletcompressor_h
#define vtk_m_worklet_waveletcompressor_h

#include <vtkm/worklet/wavelets/WaveletDWT.h>

#include <vtkm/cont/ArrayCopy.h>
#include <vtkm/cont/ArrayGetValues.h>

namespace vtkm
{
namespace worklet
{

class WaveletCompressor : public vtkm::worklet::wavelets::WaveletDWT
{
public:
  // Constructor
  WaveletCompressor(wavelets::WaveletName name)
    : WaveletDWT(name)
  {
  }

  // Multi-level 1D wavelet decomposition
  template <typename SignalArrayType, typename CoeffArrayType>
  VTKM_CONT vtkm::Id WaveDecompose(const SignalArrayType& sigIn, // Input
                                   vtkm::Id nLevels,             // n levels of DWT
                                   CoeffArrayType& coeffOut,
                                   std::vector<vtkm::Id>& L)
  {
    vtkm::Id sigInLen = sigIn.GetNumberOfValues();
    if (nLevels < 0 || nLevels > WaveletBase::GetWaveletMaxLevel(sigInLen))
    {
      throw vtkm::cont::ErrorBadValue("Number of levels of transform is not supported! ");
    }
    if (nLevels == 0) //  0 levels means no transform
    {
      vtkm::cont::ArrayCopy(sigIn, coeffOut);
      return 0;
    }

    this->ComputeL(sigInLen, nLevels, L); // memory for L is allocated by ComputeL().
    vtkm::Id CLength = this->ComputeCoeffLength(L, nLevels);
    VTKM_ASSERT(CLength == sigInLen);

    vtkm::Id sigInPtr = 0; // pseudo pointer for the beginning of input array
    vtkm::Id len = sigInLen;
    vtkm::Id cALen = WaveletBase::GetApproxLength(len);
    vtkm::Id cptr; // pseudo pointer for the beginning of output array
    vtkm::Id tlen = 0;
    std::vector<vtkm::Id> L1d(3, 0);

    // Use an intermediate array
    using OutputValueType = typename CoeffArrayType::ValueType;
    using InterArrayType = vtkm::cont::ArrayHandle<OutputValueType>;

    // Define a few more types
    using IdArrayType = vtkm::cont::ArrayHandleCounting<vtkm::Id>;
    using PermutArrayType = vtkm::cont::ArrayHandlePermutation<IdArrayType, CoeffArrayType>;

    vtkm::cont::ArrayCopy(sigIn, coeffOut);

    for (vtkm::Id i = nLevels; i > 0; i--)
    {
      tlen += L[size_t(i)];
      cptr = 0 + CLength - tlen - cALen;

      // make input array (permutation array)
      IdArrayType inputIndices(sigInPtr, 1, len);
      PermutArrayType input(inputIndices, coeffOut);
      // make output array
      InterArrayType output;

      WaveletDWT::DWT1D(input, output, L1d);

      // move intermediate results to final array
      WaveletBase::DeviceCopyStartX(output, coeffOut, cptr);

      // update pseudo pointers
      len = cALen;
      cALen = WaveletBase::GetApproxLength(cALen);
      sigInPtr = cptr;
    }

    return 0;
  }

  // Multi-level 1D wavelet reconstruction
  template <typename CoeffArrayType, typename SignalArrayType>
  VTKM_CONT vtkm::Id WaveReconstruct(const CoeffArrayType& coeffIn, // Input
                                     vtkm::Id nLevels,              // n levels of DWT
                                     std::vector<vtkm::Id>& L,
                                     SignalArrayType& sigOut)
  {
    VTKM_ASSERT(nLevels > 0);
    vtkm::Id LLength = nLevels + 2;
    VTKM_ASSERT(vtkm::Id(L.size()) == LLength);

    std::vector<vtkm::Id> L1d(3, 0); // three elements
    L1d[0] = L[0];
    L1d[1] = L[1];

    using OutValueType = typename SignalArrayType::ValueType;
    using OutArrayBasic = vtkm::cont::ArrayHandle<OutValueType>;
    using IdArrayType = vtkm::cont::ArrayHandleCounting<vtkm::Id>;
    using PermutArrayType = vtkm::cont::ArrayHandlePermutation<IdArrayType, SignalArrayType>;

    vtkm::cont::ArrayCopy(coeffIn, sigOut);

    for (vtkm::Id i = 1; i <= nLevels; i++)
    {
      L1d[2] = this->GetApproxLengthLevN(L[size_t(LLength - 1)], nLevels - i);

      // Make an input array
      IdArrayType inputIndices(0, 1, L1d[2]);
      PermutArrayType input(inputIndices, sigOut);

      // Make an output array
      OutArrayBasic output;

      WaveletDWT::IDWT1D(input, L1d, output);
      VTKM_ASSERT(output.GetNumberOfValues() == L1d[2]);

      // Move output to intermediate array
      WaveletBase::DeviceCopyStartX(output, sigOut, 0);

      L1d[0] = L1d[2];
      L1d[1] = L[size_t(i + 1)];
    }

    return 0;
  }

  // Multi-level 3D wavelet decomposition
  template <typename InArrayType, typename OutArrayType>
  VTKM_CONT vtkm::Float64 WaveDecompose3D(InArrayType& sigIn, // Input
                                          vtkm::Id nLevels,   // n levels of DWT
                                          vtkm::Id inX,
                                          vtkm::Id inY,
                                          vtkm::Id inZ,
                                          OutArrayType& coeffOut,
                                          bool discardSigIn) // can we discard sigIn on devices?
  {
    vtkm::Id sigInLen = sigIn.GetNumberOfValues();
    VTKM_ASSERT(inX * inY * inZ == sigInLen);
    if (nLevels < 0 || nLevels > WaveletBase::GetWaveletMaxLevel(inX) ||
        nLevels > WaveletBase::GetWaveletMaxLevel(inY) ||
        nLevels > WaveletBase::GetWaveletMaxLevel(inZ))
    {
      throw vtkm::cont::ErrorBadValue("Number of levels of transform is not supported! ");
    }
    if (nLevels == 0) //  0 levels means no transform
    {
      vtkm::cont::ArrayCopy(sigIn, coeffOut);
      return 0;
    }

    vtkm::Id currentLenX = inX;
    vtkm::Id currentLenY = inY;
    vtkm::Id currentLenZ = inZ;

    using OutValueType = typename OutArrayType::ValueType;
    using OutBasicArray = vtkm::cont::ArrayHandle<OutValueType>;

    // First level transform writes to the output array
    vtkm::Float64 computationTime = WaveletDWT::DWT3D(
      sigIn, inX, inY, inZ, 0, 0, 0, currentLenX, currentLenY, currentLenZ, coeffOut, discardSigIn);

    // Successor transforms writes to a temporary array
    for (vtkm::Id i = nLevels - 1; i > 0; i--)
    {
      currentLenX = WaveletBase::GetApproxLength(currentLenX);
      currentLenY = WaveletBase::GetApproxLength(currentLenY);
      currentLenZ = WaveletBase::GetApproxLength(currentLenZ);

      OutBasicArray tempOutput;

      computationTime += WaveletDWT::DWT3D(
        coeffOut, inX, inY, inZ, 0, 0, 0, currentLenX, currentLenY, currentLenZ, tempOutput, false);

      // copy results to coeffOut
      WaveletBase::DeviceCubeCopyTo(
        tempOutput, currentLenX, currentLenY, currentLenZ, coeffOut, inX, inY, inZ, 0, 0, 0);
    }

    return computationTime;
  }

  // Multi-level 3D wavelet reconstruction
  template <typename InArrayType, typename OutArrayType>
  VTKM_CONT vtkm::Float64 WaveReconstruct3D(
    InArrayType& arrIn, // Input
    vtkm::Id nLevels,   // n levels of DWT
    vtkm::Id inX,
    vtkm::Id inY,
    vtkm::Id inZ,
    OutArrayType& arrOut,
    bool discardArrIn) // can we discard input for more memory?
  {
    vtkm::Id arrInLen = arrIn.GetNumberOfValues();
    VTKM_ASSERT(inX * inY * inZ == arrInLen);
    if (nLevels < 0 || nLevels > WaveletBase::GetWaveletMaxLevel(inX) ||
        nLevels > WaveletBase::GetWaveletMaxLevel(inY) ||
        nLevels > WaveletBase::GetWaveletMaxLevel(inZ))
    {
      throw vtkm::cont::ErrorBadValue("Number of levels of transform is not supported! ");
    }
    using OutValueType = typename OutArrayType::ValueType;
    using OutBasicArray = vtkm::cont::ArrayHandle<OutValueType>;
    vtkm::Float64 computationTime = 0.0;

    OutBasicArray outBuffer;
    if (nLevels == 0) //  0 levels means no transform
    {
      vtkm::cont::ArrayCopy(arrIn, arrOut);
      return 0;
    }
    else if (discardArrIn)
    {
      outBuffer = arrIn;
    }
    else
    {
      vtkm::cont::ArrayCopy(arrIn, outBuffer);
    }

    std::vector<vtkm::Id> L;
    this->ComputeL3(inX, inY, inZ, nLevels, L);
    std::vector<vtkm::Id> L3d(27, 0);

    // All transforms but the last level operate on temporary arrays
    for (size_t i = 0; i < 24; i++)
    {
      L3d[i] = L[i];
    }
    for (size_t i = 1; i < static_cast<size_t>(nLevels); i++)
    {
      L3d[24] = L3d[0] + L3d[12]; // Total X dim; this is always true for Biorthogonal wavelets
      L3d[25] = L3d[1] + L3d[7];  // Total Y dim
      L3d[26] = L3d[2] + L3d[5];  // Total Z dim

      OutBasicArray tempOutput;

      // IDWT
      computationTime +=
        WaveletDWT::IDWT3D(outBuffer, inX, inY, inZ, 0, 0, 0, L3d, tempOutput, false);

      // copy back reconstructed block
      WaveletBase::DeviceCubeCopyTo(
        tempOutput, L3d[24], L3d[25], L3d[26], outBuffer, inX, inY, inZ, 0, 0, 0);

      // update L3d array
      L3d[0] = L3d[24];
      L3d[1] = L3d[25];
      L3d[2] = L3d[26];
      for (size_t j = 3; j < 24; j++)
      {
        L3d[j] = L[21 * i + j];
      }
    }

    // The last transform outputs to the final output
    L3d[24] = L3d[0] + L3d[12];
    L3d[25] = L3d[1] + L3d[7];
    L3d[26] = L3d[2] + L3d[5];
    computationTime += WaveletDWT::IDWT3D(outBuffer, inX, inY, inZ, 0, 0, 0, L3d, arrOut, true);

    return computationTime;
  }

  // Multi-level 2D wavelet decomposition
  template <typename InArrayType, typename OutArrayType>
  VTKM_CONT vtkm::Float64 WaveDecompose2D(const InArrayType& sigIn, // Input
                                          vtkm::Id nLevels,         // n levels of DWT
                                          vtkm::Id inX,             // Input X dim
                                          vtkm::Id inY,             // Input Y dim
                                          OutArrayType& coeffOut,
                                          std::vector<vtkm::Id>& L)
  {
    vtkm::Id sigInLen = sigIn.GetNumberOfValues();
    VTKM_ASSERT(inX * inY == sigInLen);
    if (nLevels < 0 || nLevels > WaveletBase::GetWaveletMaxLevel(inX) ||
        nLevels > WaveletBase::GetWaveletMaxLevel(inY))
    {
      throw vtkm::cont::ErrorBadValue("Number of levels of transform is not supported! ");
    }
    if (nLevels == 0) //  0 levels means no transform
    {
      vtkm::cont::ArrayCopy(sigIn, coeffOut);
      return 0;
    }

    this->ComputeL2(inX, inY, nLevels, L);
    vtkm::Id CLength = this->ComputeCoeffLength2(L, nLevels);
    VTKM_ASSERT(CLength == sigInLen);

    vtkm::Id currentLenX = inX;
    vtkm::Id currentLenY = inY;
    std::vector<vtkm::Id> L2d(10, 0);
    vtkm::Float64 computationTime = 0.0;

    using OutValueType = typename OutArrayType::ValueType;
    using OutBasicArray = vtkm::cont::ArrayHandle<OutValueType>;

    // First level transform operates writes to the output array
    computationTime += WaveletDWT::DWT2D(
      sigIn, currentLenX, currentLenY, 0, 0, currentLenX, currentLenY, coeffOut, L2d);
    VTKM_ASSERT(coeffOut.GetNumberOfValues() == currentLenX * currentLenY);
    currentLenX = WaveletBase::GetApproxLength(currentLenX);
    currentLenY = WaveletBase::GetApproxLength(currentLenY);

    // Successor transforms writes to a temporary array
    for (vtkm::Id i = nLevels - 1; i > 0; i--)
    {
      OutBasicArray tempOutput;

      computationTime +=
        WaveletDWT::DWT2D(coeffOut, inX, inY, 0, 0, currentLenX, currentLenY, tempOutput, L2d);

      // copy results to coeffOut
      WaveletBase::DeviceRectangleCopyTo(
        tempOutput, currentLenX, currentLenY, coeffOut, inX, inY, 0, 0);

      // update currentLen
      currentLenX = WaveletBase::GetApproxLength(currentLenX);
      currentLenY = WaveletBase::GetApproxLength(currentLenY);
    }

    return computationTime;
  }

  // Multi-level 2D wavelet reconstruction
  template <typename InArrayType, typename OutArrayType>
  VTKM_CONT vtkm::Float64 WaveReconstruct2D(const InArrayType& arrIn, // Input
                                            vtkm::Id nLevels,         // n levels of DWT
                                            vtkm::Id inX,             // Input X dim
                                            vtkm::Id inY,             // Input Y dim
                                            OutArrayType& arrOut,
                                            std::vector<vtkm::Id>& L)
  {
    vtkm::Id arrInLen = arrIn.GetNumberOfValues();
    VTKM_ASSERT(inX * inY == arrInLen);
    if (nLevels < 0 || nLevels > WaveletBase::GetWaveletMaxLevel(inX) ||
        nLevels > WaveletBase::GetWaveletMaxLevel(inY))
    {
      throw vtkm::cont::ErrorBadValue("Number of levels of transform is not supported! ");
    }
    using OutValueType = typename OutArrayType::ValueType;
    using OutBasicArray = vtkm::cont::ArrayHandle<OutValueType>;
    vtkm::Float64 computationTime = 0.0;

    OutBasicArray outBuffer;
    if (nLevels == 0) //  0 levels means no transform
    {
      vtkm::cont::ArrayCopy(arrIn, arrOut);
      return 0;
    }
    else
    {
      vtkm::cont::ArrayCopy(arrIn, outBuffer);
    }

    VTKM_ASSERT(vtkm::Id(L.size()) == 6 * nLevels + 4);

    std::vector<vtkm::Id> L2d(10, 0);
    L2d[0] = L[0];
    L2d[1] = L[1];
    L2d[2] = L[2];
    L2d[3] = L[3];
    L2d[4] = L[4];
    L2d[5] = L[5];
    L2d[6] = L[6];
    L2d[7] = L[7];

    // All transforms but the last operate on temporary arrays
    for (size_t i = 1; i < static_cast<size_t>(nLevels); i++)
    {
      L2d[8] = L2d[0] + L2d[4]; // This is always true for Biorthogonal wavelets
      L2d[9] = L2d[1] + L2d[3]; // (same above)

      OutBasicArray tempOutput;

      // IDWT
      computationTime += WaveletDWT::IDWT2D(outBuffer, inX, inY, 0, 0, L2d, tempOutput);

      // copy back reconstructed block
      WaveletBase::DeviceRectangleCopyTo(tempOutput, L2d[8], L2d[9], outBuffer, inX, inY, 0, 0);

      // update L2d array
      L2d[0] = L2d[8];
      L2d[1] = L2d[9];
      L2d[2] = L[6 * i + 2];
      L2d[3] = L[6 * i + 3];
      L2d[4] = L[6 * i + 4];
      L2d[5] = L[6 * i + 5];
      L2d[6] = L[6 * i + 6];
      L2d[7] = L[6 * i + 7];
    }

    // The last transform outputs to the final output
    L2d[8] = L2d[0] + L2d[4];
    L2d[9] = L2d[1] + L2d[3];
    computationTime += WaveletDWT::IDWT2D(outBuffer, inX, inY, 0, 0, L2d, arrOut);

    return computationTime;
  }

  // Squash coefficients smaller than a threshold
  template <typename CoeffArrayType>
  vtkm::Id SquashCoefficients(CoeffArrayType& coeffIn, vtkm::Float64 ratio)
  {
    if (ratio > 1.0)
    {
      vtkm::Id coeffLen = coeffIn.GetNumberOfValues();
      using ValueType = typename CoeffArrayType::ValueType;
      using CoeffArrayBasic = vtkm::cont::ArrayHandle<ValueType>;
      CoeffArrayBasic sortedArray;
      vtkm::cont::ArrayCopy(coeffIn, sortedArray);

      WaveletBase::DeviceSort(sortedArray);

      vtkm::Id n = coeffLen - static_cast<vtkm::Id>(static_cast<vtkm::Float64>(coeffLen) / ratio);
      vtkm::Float64 nthVal = static_cast<vtkm::Float64>(vtkm::cont::ArrayGetValue(n, sortedArray));
      if (nthVal < 0.0)
      {
        nthVal *= -1.0;
      }

      using ThresholdType = vtkm::worklet::wavelets::ThresholdWorklet;
      ThresholdType thresholdWorklet(nthVal);
      vtkm::worklet::DispatcherMapField<ThresholdType> dispatcher(thresholdWorklet);
      dispatcher.Invoke(coeffIn);
    }

    return 0;
  }

  // Report statistics on reconstructed array
  template <typename ArrayType>
  vtkm::Id EvaluateReconstruction(const ArrayType& original, const ArrayType& reconstruct)
  {
#define VAL vtkm::Float64
#define MAKEVAL(a) (static_cast<VAL>(a))
    VAL VarOrig = WaveletBase::DeviceCalculateVariance(original);

    using ValueType = typename ArrayType::ValueType;
    using ArrayBasic = vtkm::cont::ArrayHandle<ValueType>;
    ArrayBasic errorArray, errorSquare;

    // Use a worklet to calculate point-wise error, and its square
    using DifferencerWorklet = vtkm::worklet::wavelets::Differencer;
    DifferencerWorklet dw;
    vtkm::worklet::DispatcherMapField<DifferencerWorklet> dwDispatcher(dw);
    dwDispatcher.Invoke(original, reconstruct, errorArray);

    using SquareWorklet = vtkm::worklet::wavelets::SquareWorklet;
    SquareWorklet sw;
    vtkm::worklet::DispatcherMapField<SquareWorklet> swDispatcher(sw);
    swDispatcher.Invoke(errorArray, errorSquare);

    VAL varErr = WaveletBase::DeviceCalculateVariance(errorArray);
    VAL snr, decibels;
    if (varErr != 0.0)
    {
      snr = VarOrig / varErr;
      decibels = 10 * vtkm::Log10(snr);
    }
    else
    {
      snr = vtkm::Infinity64();
      decibels = vtkm::Infinity64();
    }

    VAL origMax = WaveletBase::DeviceMax(original);
    VAL origMin = WaveletBase::DeviceMin(original);
    VAL errorMax = WaveletBase::DeviceMaxAbs(errorArray);
    VAL range = origMax - origMin;

    VAL squareSum = WaveletBase::DeviceSum(errorSquare);
    VAL rmse = vtkm::Sqrt(MAKEVAL(squareSum) / MAKEVAL(errorArray.GetNumberOfValues()));

    std::cout << "Data range             = " << range << std::endl;
    std::cout << "SNR                    = " << snr << std::endl;
    std::cout << "SNR in decibels        = " << decibels << std::endl;
    std::cout << "L-infy norm            = " << errorMax
              << ", after normalization  = " << errorMax / range << std::endl;
    std::cout << "RMSE                   = " << rmse << ", after normalization  = " << rmse / range
              << std::endl;
#undef MAKEVAL
#undef VAL

    return 0;
  }

  // Compute the book keeping array L for 1D DWT
  void ComputeL(vtkm::Id sigInLen, vtkm::Id nLev, std::vector<vtkm::Id>& L)
  {
    size_t nLevels = static_cast<size_t>(nLev); // cast once
    L.resize(nLevels + 2);
    L[nLevels + 1] = sigInLen;
    L[nLevels] = sigInLen;
    for (size_t i = nLevels; i > 0; i--)
    {
      L[i - 1] = WaveletBase::GetApproxLength(L[i]);
      L[i] = WaveletBase::GetDetailLength(L[i]);
    }
  }

  // Compute the book keeping array L for 2D DWT
  void ComputeL2(vtkm::Id inX, vtkm::Id inY, vtkm::Id nLev, std::vector<vtkm::Id>& L)
  {
    size_t nLevels = static_cast<size_t>(nLev);
    L.resize(nLevels * 6 + 4);
    L[nLevels * 6] = inX;
    L[nLevels * 6 + 1] = inY;
    L[nLevels * 6 + 2] = inX;
    L[nLevels * 6 + 3] = inY;

    for (size_t i = nLevels; i > 0; i--)
    {
      // cA
      L[i * 6 - 6] = WaveletBase::GetApproxLength(L[i * 6 + 0]);
      L[i * 6 - 5] = WaveletBase::GetApproxLength(L[i * 6 + 1]);

      // cDh
      L[i * 6 - 4] = WaveletBase::GetApproxLength(L[i * 6 + 0]);
      L[i * 6 - 3] = WaveletBase::GetDetailLength(L[i * 6 + 1]);

      // cDv
      L[i * 6 - 2] = WaveletBase::GetDetailLength(L[i * 6 + 0]);
      L[i * 6 - 1] = WaveletBase::GetApproxLength(L[i * 6 + 1]);

      // cDv - overwrites previous value!
      L[i * 6 - 0] = WaveletBase::GetDetailLength(L[i * 6 + 0]);
      L[i * 6 + 1] = WaveletBase::GetDetailLength(L[i * 6 + 1]);
    }
  }

  // Compute the bookkeeping array L for 3D DWT
  void ComputeL3(vtkm::Id inX, vtkm::Id inY, vtkm::Id inZ, vtkm::Id nLev, std::vector<vtkm::Id>& L)
  {
    size_t n = static_cast<size_t>(nLev);
    L.resize(n * 21 + 6);
    L[n * 21 + 0] = inX;
    L[n * 21 + 1] = inY;
    L[n * 21 + 2] = inZ;
    L[n * 21 + 3] = inX;
    L[n * 21 + 4] = inY;
    L[n * 21 + 5] = inZ;

    for (size_t i = n; i > 0; i--)
    {
      // cLLL
      L[i * 21 - 21] = WaveletBase::GetApproxLength(L[i * 21 + 0]);
      L[i * 21 - 20] = WaveletBase::GetApproxLength(L[i * 21 + 1]);
      L[i * 21 - 19] = WaveletBase::GetApproxLength(L[i * 21 + 2]);

      // cLLH
      L[i * 21 - 18] = L[i * 21 - 21];
      L[i * 21 - 17] = L[i * 21 - 20];
      L[i * 21 - 16] = WaveletBase::GetDetailLength(L[i * 21 + 2]);

      // cLHL
      L[i * 21 - 15] = L[i * 21 - 21];
      L[i * 21 - 14] = WaveletBase::GetDetailLength(L[i * 21 + 1]);
      L[i * 21 - 13] = L[i * 21 - 19];

      // cLHH
      L[i * 21 - 12] = L[i * 21 - 21];
      L[i * 21 - 11] = L[i * 21 - 14];
      L[i * 21 - 10] = L[i * 21 - 16];

      // cHLL
      L[i * 21 - 9] = WaveletBase::GetDetailLength(L[i * 21 + 0]);
      L[i * 21 - 8] = L[i * 21 - 20];
      L[i * 21 - 7] = L[i * 21 - 19];

      // cHLH
      L[i * 21 - 6] = L[i * 21 - 9];
      L[i * 21 - 5] = L[i * 21 - 20];
      L[i * 21 - 3] = L[i * 21 - 16];

      // cHHL
      L[i * 21 - 3] = L[i * 21 - 9];
      L[i * 21 - 2] = L[i * 21 - 14];
      L[i * 21 - 1] = L[i * 21 - 19];

      // cHHH - overwrites previous value
      L[i * 21 + 0] = L[i * 21 - 9];
      L[i * 21 + 1] = L[i * 21 - 14];
      L[i * 21 + 2] = L[i * 21 - 16];
    }
  }

  // Compute the length of coefficients for 1D transforms
  vtkm::Id ComputeCoeffLength(std::vector<vtkm::Id>& L, vtkm::Id nLevels)
  {
    vtkm::Id sum = L[0]; // 1st level cA
    for (size_t i = 1; i <= size_t(nLevels); i++)
    {
      sum += L[i];
    }
    return sum;
  }
  // Compute the length of coefficients for 2D transforms
  vtkm::Id ComputeCoeffLength2(std::vector<vtkm::Id>& L, vtkm::Id nLevels)
  {
    vtkm::Id sum = (L[0] * L[1]); // 1st level cA
    for (size_t i = 1; i <= size_t(nLevels); i++)
    {
      sum += L[i * 6 - 4] * L[i * 6 - 3]; // cDh
      sum += L[i * 6 - 2] * L[i * 6 - 1]; // cDv
      sum += L[i * 6 - 0] * L[i * 6 + 1]; // cDd
    }
    return sum;
  }

  // Compute approximate coefficient length at a specific level
  vtkm::Id GetApproxLengthLevN(vtkm::Id sigInLen, vtkm::Id levN)
  {
    vtkm::Id cALen = sigInLen;
    for (vtkm::Id i = 0; i < levN; i++)
    {
      cALen = WaveletBase::GetApproxLength(cALen);
      if (cALen == 0)
      {
        return cALen;
      }
    }

    return cALen;
  }

}; // class WaveletCompressor

} // namespace worklet
} // namespace vtkm

#endif
