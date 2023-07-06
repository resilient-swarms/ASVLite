/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkMath.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================
  Copyright 2011 Sandia Corporation.
  Under the terms of Contract DE-AC04-94AL85000, there is a non-exclusive
  license for use of this work by or on behalf of the
  U.S. Government. Redistribution and use in source and binary forms, with
  or without modification, are permitted provided that this Notice and any
  statement of authorship are reproduced on all copies.

  Contact: pppebay@sandia.gov,dcthomp@sandia.gov,

=========================================================================*/
#include "vtkMath.h"

#include "vtkBoxMuellerRandomSequence.h"
#include "vtkDataArray.h"
#include "vtkDebugLeaks.h"
#include "vtkMinimalStandardRandomSequence.h"
#include "vtkObjectFactory.h"
#include "vtkTypeTraits.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

vtkStandardNewMacro(vtkMath);

class vtkMathInternal : public vtkObjectBase
{
public:
  vtkBaseTypeMacro(vtkMathInternal, vtkObjectBase);
  static vtkMathInternal* New()
  {
    // Can't use object factory macros, since they cast to vtkObject*.
    vtkMathInternal* ret = new vtkMathInternal;
    ret->InitializeObjectBase();
    return ret;
  }

  vtkMinimalStandardRandomSequence* Uniform;
  vtkBoxMuellerRandomSequence* Gaussian;
  std::vector<vtkTypeInt64> MemoizeFactorial;

private:
  vtkMathInternal();
  ~vtkMathInternal() override;
};

vtkMathInternal::vtkMathInternal()
{
  this->Gaussian = vtkBoxMuellerRandomSequence::New();

  // This line assumes the current vtkBoxMuellerRandomSequence behavior:
  // an initial vtkMinimalStandardRandomSequence is created.
  this->Uniform =
    static_cast<vtkMinimalStandardRandomSequence*>(this->Gaussian->GetUniformSequence());
  this->Uniform->SetSeedOnly(1177); // One author's home address
  this->MemoizeFactorial.resize(21, 0);
}

vtkMathInternal::~vtkMathInternal()
{
  this->Gaussian->Delete();
}

vtkSmartPointer<vtkMathInternal> vtkMath::Internal = vtkSmartPointer<vtkMathInternal>::New();

//
// Some useful macros and functions
//

//------------------------------------------------------------------------------
// Return the lowest value "i" for which 2^i >= x
int vtkMath::CeilLog2(vtkTypeUInt64 x)
{
  static const vtkTypeUInt64 t[6] = { 0xffffffff00000000ull, 0x00000000ffff0000ull,
    0x000000000000ff00ull, 0x00000000000000f0ull, 0x000000000000000cull, 0x0000000000000002ull };

  int j = 32;

  // if x is not a power of two, add 1 to final answer
  // (this is the "ceil" part of the computation)
  int y = (((x & (x - 1)) == 0) ? 0 : 1);

  // loop through the table (this unrolls nicely)
  for (int i = 0; i < 6; ++i)
  {
    int k = (((x & t[i]) == 0) ? 0 : j);
    y += k;
    x >>= k;
    j >>= 1;
  }

  return y;
}

//------------------------------------------------------------------------------
// Generate pseudo-random numbers distributed according to the uniform
// distribution between 0.0 and 1.0.
// This is used to provide portability across different systems.
double vtkMath::Random()
{
  vtkMath::Internal->Uniform->Next();
  return vtkMath::Internal->Uniform->GetValue();
}

//------------------------------------------------------------------------------
// Initialize seed value. NOTE: Random() has the bad property that
// the first random number returned after RandomSeed() is called
// is proportional to the seed value! To help solve this, call
// RandomSeed() a few times inside seed. This doesn't ruin the
// repeatability of Random().
void vtkMath::RandomSeed(int s)
{
  vtkMath::Internal->Uniform->SetSeed(s);
}

//------------------------------------------------------------------------------
// Description:
// Return the current seed used by the random number generator.
int vtkMath::GetSeed()
{
  return vtkMath::Internal->Uniform->GetSeed();
}

//------------------------------------------------------------------------------
double vtkMath::Random(double min, double max)
{
  vtkMath::Internal->Uniform->Next();
  return vtkMath::Internal->Uniform->GetRangeValue(min, max);
}

//------------------------------------------------------------------------------
double vtkMath::Gaussian()
{
  vtkMath::Internal->Gaussian->Next();
  return vtkMath::Internal->Gaussian->GetValue();
}

//------------------------------------------------------------------------------
double vtkMath::Gaussian(double mean, double std)
{
  vtkMath::Internal->Gaussian->Next();
  return vtkMath::Internal->Gaussian->GetScaledValue(mean, std);
}

//------------------------------------------------------------------------------
vtkTypeInt64 vtkMath::Factorial(int N)
{
  if (N > 20)
  {
    vtkGenericWarningMacro("Factorial(" << N << ") would overflow.");
    return vtkTypeTraits<vtkTypeInt64>::Max();
  }

  if (N == 0)
  {
    return 1;
  }

  if (vtkMath::Internal->MemoizeFactorial[N] != 0)
  {
    return vtkMath::Internal->MemoizeFactorial[N];
  }

  vtkTypeInt64 r = vtkMath::Factorial(N - 1) * N;
  vtkMath::Internal->MemoizeFactorial[N] = r;
  return r;
}

//------------------------------------------------------------------------------
// The number of combinations of n objects from a pool of m objects (m>n).
vtkTypeInt64 vtkMath::Binomial(int m, int n)
{
  double r = 1;
  for (int i = 1; i <= n; ++i)
  {
    r *= static_cast<double>(m - i + 1) / i;
  }
  return static_cast<vtkTypeInt64>(r);
}

//------------------------------------------------------------------------------
// Start iterating over "m choose n" objects.
// This function returns an array of n integers, each from 0 to m-1.
// These integers represent the n items chosen from the set [0,m[.
int* vtkMath::BeginCombination(int m, int n)
{
  if (m < n)
  {
    return nullptr;
  }

  int* r = new int[n];
  for (int i = 0; i < n; ++i)
  {
    r[i] = i;
  }
  return r;
}

//------------------------------------------------------------------------------
// Given \a m, \a n, and a valid \a combination of \a n integers in
// the range [0,m[, this function alters the integers into the next
// combination in a sequence of all combinations of \a n items from
// a pool of \a m.
// If the \a combination is the last item in the sequence on input,
// then \a combination is unaltered and 0 is returned.
// Otherwise, 1 is returned and \a combination is updated.
int vtkMath::NextCombination(int m, int n, int* combination)
{
  int status = 0;
  for (int i = n - 1; i >= 0; --i)
  {
    if (combination[i] < m - n + i)
    {
      int j = combination[i] + 1;
      while (i < n)
      {
        combination[i++] = j++;
      }
      status = 1;
      break;
    }
  }
  return status;
}

//------------------------------------------------------------------------------
// Free the "iterator" array created by vtkMath::BeginCombination.
//
void vtkMath::FreeCombination(int* combination)
{
  delete[] combination;
}

//------------------------------------------------------------------------------
// Given a unit vector v1, find two other unit vectors v2 and v3 which
// which form an orthonormal set.
template <class T1, class T2, class T3>
inline void vtkMathPerpendiculars(const T1 v1[3], T2 v2[3], T3 v3[3], double theta)
{
  double v1sq = v1[0] * v1[0];
  double v2sq = v1[1] * v1[1];
  double v3sq = v1[2] * v1[2];
  double r = std::sqrt(v1sq + v2sq + v3sq);

  // transpose the vector to avoid divide-by-zero error
  int dv1, dv2, dv3;
  if (v1sq > v2sq && v1sq > v3sq)
  {
    dv1 = 0;
    dv2 = 1;
    dv3 = 2;
  }
  else if (v2sq > v3sq)
  {
    dv1 = 1;
    dv2 = 2;
    dv3 = 0;
  }
  else
  {
    dv1 = 2;
    dv2 = 0;
    dv3 = 1;
  }

  double a = v1[dv1] / r;
  double b = v1[dv2] / r;
  double c = v1[dv3] / r;

  double tmp = std::sqrt(a * a + c * c);

  if (theta != 0.0)
  {
    double sintheta = sin(theta);
    double costheta = cos(theta);

    if (v2)
    {
      v2[dv1] = (c * costheta - a * b * sintheta) / tmp;
      v2[dv2] = sintheta * tmp;
      v2[dv3] = (-a * costheta - b * c * sintheta) / tmp;
    }

    if (v3)
    {
      v3[dv1] = (-c * sintheta - a * b * costheta) / tmp;
      v3[dv2] = costheta * tmp;
      v3[dv3] = (a * sintheta - b * c * costheta) / tmp;
    }
  }
  else
  {
    if (v2)
    {
      v2[dv1] = c / tmp;
      v2[dv2] = 0;
      v2[dv3] = -a / tmp;
    }

    if (v3)
    {
      v3[dv1] = -a * b / tmp;
      v3[dv2] = tmp;
      v3[dv3] = -b * c / tmp;
    }
  }
}

void vtkMath::Perpendiculars(const double v1[3], double v2[3], double v3[3], double theta)
{
  vtkMathPerpendiculars(v1, v2, v3, theta);
}

void vtkMath::Perpendiculars(const float v1[3], float v2[3], float v3[3], double theta)
{
  vtkMathPerpendiculars(v1, v2, v3, theta);
}

//------------------------------------------------------------------------------
// Solve linear equation Ax = b using Gaussian Elimination with Partial Pivoting
// for a 2x2 system. If the matrix is found to be singular within a small numerical
// tolerance close to machine precision then 0 is returned.
vtkTypeBool vtkMath::SolveLinearSystemGEPP2x2(
  double a00, double a01, double a10, double a11, double b0, double b1, double& x0, double& x1)
{
  // Check if any of the matrix coefficients is zero.
  // If so then swap rows/columns to form an upper triangular matrix without
  // having to use GEPP.
  bool cols_swapped = false;
  if ((a00 == 0) || (a01 == 0) || (a10 == 0) || (a11 == 0))
  {
    // zero in either row of the 2nd column?
    if ((a01 == 0) || (a11 == 0))
    {
      // swap columns
      std::swap(a00, a01);
      std::swap(a10, a11);
      cols_swapped = true;
    }
    // zero in a00?
    if (a00 == 0)
    {
      // swap rows
      std::swap(a00, a10);
      std::swap(a01, a11);
      std::swap(b0, b1);
    }
  }
  else
  {
    // None of the matrix coefficients are exactly zero.
    // Use GEPP to form upper triangular matrix, i.e. so that a10 == 0.
    // Select pivot by looking at largest absolute value in a00, a10
    if (fabs(a00) < fabs(a10))
    {
      // swap rows so largest coefficient in first column is in the first row
      std::swap(a00, a10);
      std::swap(a01, a11);
      std::swap(b0, b1);
    }
    // a10 = 0;            // bookkeeping only, value is no longer required
    const double f = -a10 / a00;
    a11 += a01 * f;
    b1 += b0 * f;
  }
  // Have now an exact zero in a10.
  // Need to check for singularity by looking at a11.
  // Note the choice of eps is reasonable but somewhat arbitrary.
  static const double eps = 256 * std::numeric_limits<double>::epsilon();
  if (fabs(a11) < eps)
  {
    // matrix is singular within small numerical tolerance
    return 0;
  }
  // Solve the triangular system
  if (a11 != 0)
  {
    x1 = b1 / a11;
  }
  else
  {
    return 0;
  }
  if (a00 != 0)
  {
    x0 = (b0 - a01 * x1) / a00;
  }
  else
  {
    return 0;
  }
  // other failures in solution?
  if (!std::isfinite(x0) || !std::isfinite(x1))
  {
    return 0;
  }
  // If necessary swap solution vector rows.
  if (cols_swapped)
  {
    std::swap(x0, x1);
  }
  return 1;
}

#define VTK_SMALL_NUMBER 1.0e-12

//------------------------------------------------------------------------------
// Solve linear equations Ax = b using Crout's method. Input is square matrix A
// and load vector b. Solution x is written over load vector. The dimension of
// the matrix is specified in size. If error is found, method returns a 0.
vtkTypeBool vtkMath::SolveLinearSystem(double** A, double* x, int size)
{
  // if we solving something simple, just solve it
  //
  if (size == 2)
  {
    return SolveLinearSystemGEPP2x2(A[0][0], A[0][1], A[1][0], A[1][1], x[0], x[1], x[0], x[1]);
  }
  else if (size == 1)
  {
    if (A[0][0] == 0.0)
    {
      // Unable to solve linear system
      return 0;
    }

    x[0] /= A[0][0];
    return 1;
  }

  //
  // System of equations is not trivial, use Crout's method
  //

  // Check on allocation of working vectors
  //
  int *index, scratch[10];
  index = (size < 10 ? scratch : new int[size]);

  //
  // Factor and solve matrix
  //
  if (vtkMath::LUFactorLinearSystem(A, index, size) == 0)
  {
    return 0;
  }
  vtkMath::LUSolveLinearSystem(A, index, x, size);

  if (size >= 10)
  {
    delete[] index;
  }
  return 1;
}

//------------------------------------------------------------------------------
// Invert input square matrix A into matrix AI. Note that A is modified during
// the inversion. The size variable is the dimension of the matrix. Returns 0
// if inverse not computed.
vtkTypeBool vtkMath::InvertMatrix(double** A, double** AI, int size)
{
  int *index, iScratch[10];
  double *column, dScratch[10];

  // Check on allocation of working vectors
  //
  if (size <= 10)
  {
    index = iScratch;
    column = dScratch;
  }
  else
  {
    index = new int[size];
    column = new double[size];
  }

  vtkTypeBool retVal = vtkMath::InvertMatrix(A, AI, size, index, column);

  if (size > 10)
  {
    delete[] index;
    delete[] column;
  }

  return retVal;
}

#define VTK_MAX_WARNS 3
//------------------------------------------------------------------------------
// Factor linear equations Ax = b using LU decomposition A = LU where L is
// lower triangular matrix and U is upper triangular matrix. Input is
// square matrix A, integer array of pivot indices index[0->n-1], and size
// of square matrix n. Output factorization LU is in matrix A. If error is
// found, method returns 0.
vtkTypeBool vtkMath::LUFactorLinearSystem(double** A, int* index, int size)
{
  double scratch[10];
  double* scale = (size < 10 ? scratch : new double[size]);

  int i, j, k;
  int maxI = 0;
  double largest, temp1, temp2, sum;

  // Manage number of output warnings
  static int numWarns = 0;

  //
  // Loop over rows to get implicit scaling information
  //
  for (i = 0; i < size; ++i)
  {
    for (largest = 0.0, j = 0; j < size; ++j)
    {
      if ((temp2 = fabs(A[i][j])) > largest)
      {
        largest = temp2;
      }
    }

    if (largest == 0.0 && numWarns++ < VTK_MAX_WARNS)
    {
      vtkGenericWarningMacro(<< "Unable to factor linear system");
      return 0;
    }
    scale[i] = 1.0 / largest;
  }
  //
  // Loop over all columns using Crout's method
  //
  for (j = 0; j < size; ++j)
  {
    for (i = 0; i < j; ++i)
    {
      sum = A[i][j];
      for (k = 0; k < i; ++k)
      {
        sum -= A[i][k] * A[k][j];
      }
      A[i][j] = sum;
    }
    //
    // Begin search for largest pivot element
    //
    for (largest = 0.0, i = j; i < size; ++i)
    {
      sum = A[i][j];
      for (k = 0; k < j; ++k)
      {
        sum -= A[i][k] * A[k][j];
      }
      A[i][j] = sum;

      if ((temp1 = scale[i] * fabs(sum)) >= largest)
      {
        largest = temp1;
        maxI = i;
      }
    }
    //
    // Check for row interchange
    //
    if (j != maxI)
    {
      for (k = 0; k < size; ++k)
      {
        temp1 = A[maxI][k];
        A[maxI][k] = A[j][k];
        A[j][k] = temp1;
      }
      scale[maxI] = scale[j];
    }
    //
    // Divide by pivot element and perform elimination
    //
    index[j] = maxI;

    if (fabs(A[j][j]) <= VTK_SMALL_NUMBER && numWarns++ < VTK_MAX_WARNS)
    {
      vtkGenericWarningMacro(<< "Unable to factor linear system");
      return 0;
    }

    if (j != (size - 1))
    {
      temp1 = 1.0 / A[j][j];
      for (i = j + 1; i < size; ++i)
      {
        A[i][j] *= temp1;
      }
    }
  }

  if (size >= 10)
  {
    delete[] scale;
  }

  return 1;
}

//------------------------------------------------------------------------------
// Solve linear equations Ax = b using LU decomposition A = LU where L is
// lower triangular matrix and U is upper triangular matrix. Input is
// factored matrix A=LU, integer array of pivot indices index[0->n-1],
// load vector x[0->n-1], and size of square matrix n. Note that A=LU and
// index[] are generated from method LUFactorLinearSystem). Also, solution
// vector is written directly over input load vector.
void vtkMath::LUSolveLinearSystem(double** A, int* index, double* x, int size)
{
  int i, j, ii, idx;
  double sum;
  //
  // Proceed with forward and backsubstitution for L and U
  // matrices.  First, forward substitution.
  //
  for (ii = -1, i = 0; i < size; ++i)
  {
    idx = index[i];
    sum = x[idx];
    x[idx] = x[i];

    if (ii >= 0)
    {
      for (j = ii; j <= (i - 1); ++j)
      {
        sum -= A[i][j] * x[j];
      }
    }
    else if (sum != 0.0)
    {
      ii = i;
    }

    x[i] = sum;
  }
  //
  // Now, back substitution
  //
  for (i = size - 1; i >= 0; i--)
  {
    sum = x[i];
    for (j = i + 1; j < size; ++j)
    {
      sum -= A[i][j] * x[j];
    }
    x[i] = sum / A[i][i];
  }
}

#undef VTK_SMALL_NUMBER

#define VTK_ROTATE(a, i, j, k, l)                                                                  \
  g = a[i][j];                                                                                     \
  h = a[k][l];                                                                                     \
  a[i][j] = g - s * (h + g * tau);                                                                 \
  a[k][l] = h + s * (g - h * tau)

#define VTK_MAX_ROTATIONS 20

//#undef VTK_MAX_ROTATIONS

//#define VTK_MAX_ROTATIONS 50

// Jacobi iteration for the solution of eigenvectors/eigenvalues of a nxn
// real symmetric matrix. Square nxn matrix a; size of matrix in n;
// output eigenvalues in w; and output eigenvectors in v. Resulting
// eigenvalues/vectors are sorted in decreasing order; eigenvectors are
// normalized.
// It assumes a is symmetric and uses only its upper right triangular part.
template <class T>
vtkTypeBool vtkJacobiN(T** a, int n, T* w, T** v)
{
  int i, j, k, iq, ip, numPos;
  T tresh, theta, tau, t, sm, s, h, g, c, tmp;
  T bspace[4], zspace[4];
  T* b = bspace;
  T* z = zspace;

  // only allocate memory if the matrix is large
  if (n > 4)
  {
    b = new T[n];
    z = new T[n];
  }

  // initialize
  for (ip = 0; ip < n; ip++)
  {
    for (iq = 0; iq < n; iq++)
    {
      v[ip][iq] = 0.0;
    }
    v[ip][ip] = 1.0;
  }
  for (ip = 0; ip < n; ip++)
  {
    b[ip] = w[ip] = a[ip][ip];
    z[ip] = 0.0;
  }

  // begin rotation sequence
  for (i = 0; i < VTK_MAX_ROTATIONS; ++i)
  {
    sm = 0.0;
    for (ip = 0; ip < n - 1; ip++)
    {
      for (iq = ip + 1; iq < n; iq++)
      {
        sm += fabs(a[ip][iq]);
      }
    }
    if (sm == 0.0)
    {
      break;
    }

    if (i < 3) // first 3 sweeps
    {
      tresh = 0.2 * sm / (n * n);
    }
    else
    {
      tresh = 0.0;
    }

    for (ip = 0; ip < n - 1; ip++)
    {
      for (iq = ip + 1; iq < n; iq++)
      {
        g = 100.0 * fabs(a[ip][iq]);

        // after 4 sweeps
        if (i > 3 && (fabs(w[ip]) + g) == fabs(w[ip]) && (fabs(w[iq]) + g) == fabs(w[iq]))
        {
          a[ip][iq] = 0.0;
        }
        else if (fabs(a[ip][iq]) > tresh)
        {
          h = w[iq] - w[ip];
          if ((fabs(h) + g) == fabs(h))
          {
            t = (a[ip][iq]) / h;
          }
          else
          {
            theta = 0.5 * h / (a[ip][iq]);
            t = 1.0 / (fabs(theta) + std::sqrt(1.0 + theta * theta));
            if (theta < 0.0)
            {
              t = -t;
            }
          }
          c = 1.0 / std::sqrt(1 + t * t);
          s = t * c;
          tau = s / (1.0 + c);
          h = t * a[ip][iq];
          z[ip] -= h;
          z[iq] += h;
          w[ip] -= h;
          w[iq] += h;
          a[ip][iq] = 0.0;

          // ip already shifted left by 1 unit
          for (j = 0; j <= ip - 1; ++j)
          {
            VTK_ROTATE(a, j, ip, j, iq);
          }
          // ip and iq already shifted left by 1 unit
          for (j = ip + 1; j <= iq - 1; ++j)
          {
            VTK_ROTATE(a, ip, j, j, iq);
          }
          // iq already shifted left by 1 unit
          for (j = iq + 1; j < n; ++j)
          {
            VTK_ROTATE(a, ip, j, iq, j);
          }
          for (j = 0; j < n; ++j)
          {
            VTK_ROTATE(v, j, ip, j, iq);
          }
        }
      }
    }

    for (ip = 0; ip < n; ip++)
    {
      b[ip] += z[ip];
      w[ip] = b[ip];
      z[ip] = 0.0;
    }
  }

  //// this is NEVER called
  if (i >= VTK_MAX_ROTATIONS)
  {
    vtkGenericWarningMacro("vtkMath::Jacobi: Error extracting eigenfunctions");
    return 0;
  }

  // sort eigenfunctions                 these changes do not affect accuracy
  for (j = 0; j < n - 1; ++j) // boundary incorrect
  {
    k = j;
    tmp = w[k];
    for (i = j + 1; i < n; ++i) // boundary incorrect, shifted already
    {
      if (w[i] >= tmp) // why exchange if same?
      {
        k = i;
        tmp = w[k];
      }
    }
    if (k != j)
    {
      w[k] = w[j];
      w[j] = tmp;
      for (i = 0; i < n; ++i)
      {
        tmp = v[i][j];
        v[i][j] = v[i][k];
        v[i][k] = tmp;
      }
    }
  }
  // ensure eigenvector consistency (i.e., Jacobi can compute vectors that
  // are negative of one another (.707,.707,0) and (-.707,-.707,0). This can
  // reek havoc in hyperstreamline/other stuff. We will select the most
  // positive eigenvector.
  int ceil_half_n = (n >> 1) + (n & 1);
  for (j = 0; j < n; ++j)
  {
    for (numPos = 0, i = 0; i < n; ++i)
    {
      if (v[i][j] >= 0.0)
      {
        numPos++;
      }
    }
    if (numPos < ceil_half_n)
    {
      for (i = 0; i < n; ++i)
      {
        v[i][j] *= -1.0;
      }
    }
  }

  if (n > 4)
  {
    delete[] b;
    delete[] z;
  }
  return 1;
}

#undef VTK_ROTATE
#undef VTK_MAX_ROTATIONS

//------------------------------------------------------------------------------
vtkTypeBool vtkMath::JacobiN(float** a, int n, float* w, float** v)
{
  return vtkJacobiN(a, n, w, v);
}

//------------------------------------------------------------------------------
vtkTypeBool vtkMath::JacobiN(double** a, int n, double* w, double** v)
{
  return vtkJacobiN(a, n, w, v);
}

//------------------------------------------------------------------------------
// Jacobi iteration for the solution of eigenvectors/eigenvalues of a 3x3
// real symmetric matrix. Square 3x3 matrix a; output eigenvalues in w;
// and output eigenvectors in v. Resulting eigenvalues/vectors are sorted
// in decreasing order; eigenvectors are normalized.
vtkTypeBool vtkMath::Jacobi(float** a, float* w, float** v)
{
  return vtkMath::JacobiN(a, 3, w, v);
}

//------------------------------------------------------------------------------
vtkTypeBool vtkMath::Jacobi(double** a, double* w, double** v)
{
  return vtkMath::JacobiN(a, 3, w, v);
}

//------------------------------------------------------------------------------
// Estimate the condition number of a LU factored matrix. Used to judge the
// accuracy of the solution. The matrix A must have been previously factored
// using the method LUFactorLinearSystem. The condition number is the ratio
// of the infinity matrix norm (i.e., maximum value of matrix component)
// divided by the minimum diagonal value. (This works for triangular matrices
// only: see Conte and de Boor, Elementary Numerical Analysis.)
double vtkMath::EstimateMatrixCondition(const double* const* A, int size)
{
  int i;
  int j;
  double min = VTK_FLOAT_MAX, max = (-VTK_FLOAT_MAX);

  // find the maximum value
  for (i = 0; i < size; ++i)
  {
    for (j = i; j < size; ++j)
    {
      if (fabs(A[i][j]) > max)
      {
        max = fabs(A[i][j]);
      }
    }
  }

  // find the minimum diagonal value
  for (i = 0; i < size; ++i)
  {
    if (fabs(A[i][i]) < min)
    {
      min = fabs(A[i][i]);
    }
  }

  if (min == 0.0)
  {
    return VTK_FLOAT_MAX;
  }
  else
  {
    return (max / min);
  }
}

//------------------------------------------------------------------------------
// Solves for the least squares best fit matrix for the homogeneous equation X'M' = 0'.
// Uses the method described on pages 40-41 of Computer Vision by
// Forsyth and Ponce, which is that the solution is the eigenvector
// associated with the minimum eigenvalue of T(X)X, where T(X) is the
// transpose of X.
// The inputs and output are transposed matrices.
//    Dimensions: X' is numberOfSamples by xOrder,
//                M' dimension is xOrder by 1.
// M' should be pre-allocated. All matrices are row major. The resultant
// matrix M' should be pre-multiplied to X' to get 0', or transposed and
// then post multiplied to X to get 0.
// Returns success/fail.
vtkTypeBool vtkMath::SolveHomogeneousLeastSquares(
  int numberOfSamples, double** xt, int xOrder, double** mt)
{
  // check dimensional consistency
  if (numberOfSamples < xOrder)
  {
    vtkGenericWarningMacro("Insufficient number of samples. Underdetermined.");
    return 0;
  }

  // set up intermediate variables
  // Allocate matrix to hold X times transpose of X
  double** XXt = new double*[xOrder]; // size x by x
  // Allocate the array of eigenvalues and eigenvectors
  double* eigenvals = new double[xOrder];
  double** eigenvecs = new double*[xOrder];

  int i, j, k;

  // Clear the upper triangular region (and btw, allocate the eigenvecs as well)
  for (i = 0; i < xOrder; ++i)
  {
    eigenvecs[i] = new double[xOrder];
    XXt[i] = new double[xOrder];
    for (j = 0; j < xOrder; ++j)
    {
      XXt[i][j] = 0.0;
    }
  }

  // Calculate XXt upper half only, due to symmetry
  for (k = 0; k < numberOfSamples; ++k)
  {
    for (i = 0; i < xOrder; ++i)
    {
      for (j = i; j < xOrder; ++j)
      {
        XXt[i][j] += xt[k][i] * xt[k][j];
      }
    }
  }

  // now fill in the lower half of the XXt matrix
  for (i = 0; i < xOrder; ++i)
  {
    for (j = 0; j < i; ++j)
    {
      XXt[i][j] = XXt[j][i];
    }
  }

  // Compute the eigenvectors and eigenvalues
  vtkMath::JacobiN(XXt, xOrder, eigenvals, eigenvecs);

  // Smallest eigenval is at the end of the list (xOrder-1), and solution is
  // corresponding eigenvec.
  for (i = 0; i < xOrder; ++i)
  {
    mt[i][0] = eigenvecs[i][xOrder - 1];
  }

  // Clean up:
  for (i = 0; i < xOrder; ++i)
  {
    delete[] XXt[i];
    delete[] eigenvecs[i];
  }
  delete[] XXt;
  delete[] eigenvecs;
  delete[] eigenvals;

  return 1;
}

static const double VTK_SMALL_NUMBER = 1.0e-12;

//------------------------------------------------------------------------------
// Solves for the least squares best fit matrix for the equation X'M' = Y'.
// Uses pseudoinverse to get the ordinary least squares.
// The inputs and output are transposed matrices.
//    Dimensions: X' is numberOfSamples by xOrder,
//                Y' is numberOfSamples by yOrder,
//                M' dimension is xOrder by yOrder.
// M' should be pre-allocated. All matrices are row major. The resultant
// matrix M' should be pre-multiplied to X' to get Y', or transposed and
// then post multiplied to X to get Y
// By default, this method checks for the homogeneous condition where Y==0, and
// if so, invokes SolveHomogeneousLeastSquares. For better performance when
// the system is known not to be homogeneous, invoke with checkHomogeneous=0.
// Returns success/fail.
vtkTypeBool vtkMath::SolveLeastSquares(int numberOfSamples, double** xt, int xOrder, double** yt,
  int yOrder, double** mt, int checkHomogeneous)
{
  // check dimensional consistency
  if ((numberOfSamples < xOrder) || (numberOfSamples < yOrder))
  {
    vtkGenericWarningMacro("Insufficient number of samples. Underdetermined.");
    return 0;
  }

  int i, j, k;

  bool someHomogeneous = false;
  bool allHomogeneous = true;
  double** hmt = nullptr;
  vtkTypeBool homogRC = 0;
  int* homogenFlags = new int[yOrder];
  vtkTypeBool successFlag;

  // Ok, first init some flags check and see if all the systems are homogeneous
  if (checkHomogeneous)
  {
    // If Y' is zero, it's a homogeneous system and can't be solved via
    // the pseudoinverse method. Detect this case, warn the user, and
    // invoke SolveHomogeneousLeastSquares instead. Note that it doesn't
    // really make much sense for yOrder to be greater than one in this case,
    // since that's just yOrder occurrences of a 0 vector on the RHS, but
    // we allow it anyway. N

    // Initialize homogeneous flags on a per-right-hand-side basis
    for (j = 0; j < yOrder; ++j)
    {
      homogenFlags[j] = 1;
    }
    for (i = 0; i < numberOfSamples; ++i)
    {
      for (j = 0; j < yOrder; ++j)
      {
        if (fabs(yt[i][j]) > VTK_SMALL_NUMBER)
        {
          allHomogeneous = false;
          homogenFlags[j] = 0;
        }
      }
    }

    // If we've got one system, and it's homogeneous, do it and bail out quickly.
    if (allHomogeneous && yOrder == 1)
    {
      vtkGenericWarningMacro(
        "Detected homogeneous system (Y=0), calling SolveHomogeneousLeastSquares()");
      delete[] homogenFlags;
      return vtkMath::SolveHomogeneousLeastSquares(numberOfSamples, xt, xOrder, mt);
    }

    // Ok, we've got more than one system of equations.
    // Figure out if we need to calculate the homogeneous equation solution for
    // any of them.
    if (allHomogeneous)
    {
      someHomogeneous = true;
    }
    else
    {
      for (j = 0; j < yOrder; ++j)
      {
        if (homogenFlags[j])
        {
          someHomogeneous = true;
        }
      }
    }
  }

  // If necessary, solve the homogeneous problem
  if (someHomogeneous)
  {
    // hmt is the homogeneous equation version of mt, the general solution.
    hmt = new double*[xOrder];
    for (j = 0; j < xOrder; ++j)
    {
      // Only allocate 1 here, not yOrder, because here we're going to solve
      // just the one homogeneous equation subset of the entire problem
      hmt[j] = new double[1];
    }

    // Ok, solve the homogeneous problem
    homogRC = vtkMath::SolveHomogeneousLeastSquares(numberOfSamples, xt, xOrder, hmt);
  }

  // set up intermediate variables
  double** XXt = new double*[xOrder];  // size x by x
  double** XXtI = new double*[xOrder]; // size x by x
  double** XYt = new double*[xOrder];  // size x by y
  for (i = 0; i < xOrder; ++i)
  {
    XXt[i] = new double[xOrder];
    XXtI[i] = new double[xOrder];

    for (j = 0; j < xOrder; ++j)
    {
      XXt[i][j] = 0.0;
      XXtI[i][j] = 0.0;
    }

    XYt[i] = new double[yOrder];
    for (j = 0; j < yOrder; ++j)
    {
      XYt[i][j] = 0.0;
    }
  }

  // first find the pseudoinverse matrix
  for (k = 0; k < numberOfSamples; ++k)
  {
    for (i = 0; i < xOrder; ++i)
    {
      // first calculate the XXt matrix, only do the upper half (symmetrical)
      for (j = i; j < xOrder; ++j)
      {
        XXt[i][j] += xt[k][i] * xt[k][j];
      }

      // now calculate the XYt matrix
      for (j = 0; j < yOrder; ++j)
      {
        XYt[i][j] += xt[k][i] * yt[k][j];
      }
    }
  }

  // now fill in the lower half of the XXt matrix
  for (i = 0; i < xOrder; ++i)
  {
    for (j = 0; j < i; ++j)
    {
      XXt[i][j] = XXt[j][i];
    }
  }

  successFlag = vtkMath::InvertMatrix(XXt, XXtI, xOrder);

  // next get the inverse of XXt
  if (successFlag)
  {
    for (i = 0; i < xOrder; ++i)
    {
      for (j = 0; j < yOrder; ++j)
      {
        mt[i][j] = 0.0;
        for (k = 0; k < xOrder; ++k)
        {
          mt[i][j] += XXtI[i][k] * XYt[k][j];
        }
      }
    }
  }

  // Fix up any of the solutions that correspond to the homogeneous equation
  // problem.
  if (someHomogeneous)
  {
    for (j = 0; j < yOrder; ++j)
    {
      if (homogenFlags[j])
      {
        // Fix this one
        for (i = 0; i < xOrder; ++i)
        {
          mt[i][j] = hmt[i][0];
        }
      }
    }

    // Clean up
    for (i = 0; i < xOrder; ++i)
    {
      delete[] hmt[i];
    }
    delete[] hmt;
  }

  // clean up:
  // set up intermediate variables
  for (i = 0; i < xOrder; ++i)
  {
    delete[] XXt[i];
    delete[] XXtI[i];

    delete[] XYt[i];
  }
  delete[] XXt;
  delete[] XXtI;
  delete[] XYt;
  delete[] homogenFlags;

  if (someHomogeneous)
  {
    return homogRC && successFlag;
  }
  else
  {
    return successFlag;
  }
}

//=============================================================================
// Thread safe versions of math methods.
//=============================================================================

// Invert input square matrix A into matrix AI. Note that A is modified during
// the inversion. The size variable is the dimension of the matrix. Returns 0
// if inverse not computed.
// -----------------------
// For thread safe behavior, temporary arrays tmp1SIze and tmp2Size
// of length size must be passsed in.
vtkTypeBool vtkMath::InvertMatrix(
  double** A, double** AI, int size, int* tmp1Size, double* tmp2Size)
{
  int i, j;

  //
  // Factor matrix; then begin solving for inverse one column at a time.
  // Note: tmp1Size returned value is used later, tmp2Size is just working
  // memory whose values are not used in LUSolveLinearSystem
  //
  if (vtkMath::LUFactorLinearSystem(A, tmp1Size, size, tmp2Size) == 0)
  {
    return 0;
  }

  for (j = 0; j < size; ++j)
  {
    for (i = 0; i < size; ++i)
    {
      tmp2Size[i] = 0.0;
    }
    tmp2Size[j] = 1.0;

    vtkMath::LUSolveLinearSystem(A, tmp1Size, tmp2Size, size);

    for (i = 0; i < size; ++i)
    {
      AI[i][j] = tmp2Size[i];
    }
  }

  return 1;
}

// Factor linear equations Ax = b using LU decomposition A = LU where L is
// lower triangular matrix and U is upper triangular matrix. Input is
// square matrix A, integer array of pivot indices index[0->n-1], and size
// of square matrix n. Output factorization LU is in matrix A. If error is
// found, method returns 0.
//------------------------------------------------------------------
// For thread safe, temporary memory array tmpSize of length size
// must be passed in.
vtkTypeBool vtkMath::LUFactorLinearSystem(double** A, int* index, int size, double* tmpSize)
{
  int i, j, k;
  int maxI = 0;
  double largest, temp1, temp2, sum;

  // Manage number of output warnings
  static int numWarns = 0;

  //
  // Loop over rows to get implicit scaling information
  //
  for (i = 0; i < size; ++i)
  {
    for (largest = 0.0, j = 0; j < size; ++j)
    {
      if ((temp2 = fabs(A[i][j])) > largest)
      {
        largest = temp2;
      }
    }

    if (largest == 0.0 && numWarns++ < VTK_MAX_WARNS)
    {
      vtkGenericWarningMacro(<< "Unable to factor linear system");
      return 0;
    }
    tmpSize[i] = 1.0 / largest;
  }
  //
  // Loop over all columns using Crout's method
  //
  for (j = 0; j < size; ++j)
  {
    for (i = 0; i < j; ++i)
    {
      sum = A[i][j];
      for (k = 0; k < i; ++k)
      {
        sum -= A[i][k] * A[k][j];
      }
      A[i][j] = sum;
    }
    //
    // Begin search for largest pivot element
    //
    for (largest = 0.0, i = j; i < size; ++i)
    {
      sum = A[i][j];
      for (k = 0; k < j; ++k)
      {
        sum -= A[i][k] * A[k][j];
      }
      A[i][j] = sum;

      if ((temp1 = tmpSize[i] * fabs(sum)) >= largest)
      {
        largest = temp1;
        maxI = i;
      }
    }
    //
    // Check for row interchange
    //
    if (j != maxI)
    {
      for (k = 0; k < size; ++k)
      {
        temp1 = A[maxI][k];
        A[maxI][k] = A[j][k];
        A[j][k] = temp1;
      }
      tmpSize[maxI] = tmpSize[j];
    }
    //
    // Divide by pivot element and perform elimination
    //
    index[j] = maxI;

    if (fabs(A[j][j]) <= VTK_SMALL_NUMBER && numWarns++ < VTK_MAX_WARNS)
    {
      vtkGenericWarningMacro(<< "Unable to factor linear system");
      return 0;
    }

    if (j != (size - 1))
    {
      temp1 = 1.0 / A[j][j];
      for (i = j + 1; i < size; ++i)
      {
        A[i][j] *= temp1;
      }
    }
  }

  return 1;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// All of the following methods are for dealing with 3x3 matrices
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// helper function, swap two 3-vectors
template <class T>
inline void vtkSwapVectors3(T v1[3], T v2[3])
{
  for (int i = 0; i < 3; ++i)
  {
    T tmp = v1[i];
    v1[i] = v2[i];
    v2[i] = tmp;
  }
}

//------------------------------------------------------------------------------
// Unrolled LU factorization of a 3x3 matrix with pivoting.
template <class T>
inline void vtkLUFactor3x3(T A[3][3], int index[3])
{
  int i, maxI;
  T tmp, largest;
  T scale[3];

  // Loop over rows to get implicit scaling information

  for (i = 0; i < 3; ++i)
  {
    largest = fabs(A[i][0]);
    if ((tmp = fabs(A[i][1])) > largest)
    {
      largest = tmp;
    }
    if ((tmp = fabs(A[i][2])) > largest)
    {
      largest = tmp;
    }
    scale[i] = T(1.0) / largest;
  }

  // Loop over all columns using Crout's method

  // first column
  largest = scale[0] * fabs(A[0][0]);
  maxI = 0;
  if ((tmp = scale[1] * fabs(A[1][0])) >= largest)
  {
    largest = tmp;
    maxI = 1;
  }
  if ((tmp = scale[2] * fabs(A[2][0])) >= largest)
  {
    maxI = 2;
  }
  if (maxI != 0)
  {
    vtkSwapVectors3(A[maxI], A[0]);
    scale[maxI] = scale[0];
  }
  index[0] = maxI;

  A[1][0] /= A[0][0];
  A[2][0] /= A[0][0];

  // second column
  A[1][1] -= A[1][0] * A[0][1];
  A[2][1] -= A[2][0] * A[0][1];
  largest = scale[1] * fabs(A[1][1]);
  maxI = 1;
  if ((tmp = scale[2] * fabs(A[2][1])) >= largest)
  {
    maxI = 2;
    vtkSwapVectors3(A[2], A[1]);
    scale[2] = scale[1];
  }
  index[1] = maxI;
  A[2][1] /= A[1][1];

  // third column
  A[1][2] -= A[1][0] * A[0][2];
  A[2][2] -= A[2][0] * A[0][2] + A[2][1] * A[1][2];
  index[2] = 2;
}

//------------------------------------------------------------------------------
void vtkMath::LUFactor3x3(float A[3][3], int index[3])
{
  vtkLUFactor3x3(A, index);
}

//------------------------------------------------------------------------------
void vtkMath::LUFactor3x3(double A[3][3], int index[3])
{
  vtkLUFactor3x3(A, index);
}

//------------------------------------------------------------------------------
// Backsubstitution with an LU-decomposed matrix.
template <class T1, class T2>
inline void vtkLUSolve3x3(const T1 A[3][3], const int index[3], T2 x[3])
{
  T2 sum;

  // forward substitution

  sum = x[index[0]];
  x[index[0]] = x[0];
  x[0] = sum;

  sum = x[index[1]];
  x[index[1]] = x[1];
  x[1] = sum - A[1][0] * x[0];

  sum = x[index[2]];
  x[index[2]] = x[2];
  x[2] = sum - A[2][0] * x[0] - A[2][1] * x[1];

  // back substitution

  x[2] = x[2] / A[2][2];
  x[1] = (x[1] - A[1][2] * x[2]) / A[1][1];
  x[0] = (x[0] - A[0][1] * x[1] - A[0][2] * x[2]) / A[0][0];
}

//------------------------------------------------------------------------------
void vtkMath::LUSolve3x3(const float A[3][3], const int index[3], float x[3])
{
  vtkLUSolve3x3(A, index, x);
}

//------------------------------------------------------------------------------
void vtkMath::LUSolve3x3(const double A[3][3], const int index[3], double x[3])
{
  vtkLUSolve3x3(A, index, x);
}

//------------------------------------------------------------------------------
// this method solves Ay = x for y
template <class T1, class T2, class T3>
inline void vtkLinearSolve3x3(const T1 A[3][3], const T2 x[3], T3 y[3])
{
  double a1 = A[0][0];
  double b1 = A[0][1];
  double c1 = A[0][2];
  double a2 = A[1][0];
  double b2 = A[1][1];
  double c2 = A[1][2];
  double a3 = A[2][0];
  double b3 = A[2][1];
  double c3 = A[2][2];

  // Compute the adjoint
  double d1 = vtkMath::Determinant2x2(b2, b3, c2, c3);
  double d2 = -vtkMath::Determinant2x2(a2, a3, c2, c3);
  double d3 = vtkMath::Determinant2x2(a2, a3, b2, b3);

  double e1 = -vtkMath::Determinant2x2(b1, b3, c1, c3);
  double e2 = vtkMath::Determinant2x2(a1, a3, c1, c3);
  double e3 = -vtkMath::Determinant2x2(a1, a3, b1, b3);

  double f1 = vtkMath::Determinant2x2(b1, b2, c1, c2);
  double f2 = -vtkMath::Determinant2x2(a1, a2, c1, c2);
  double f3 = vtkMath::Determinant2x2(a1, a2, b1, b2);

  // Compute the determinant
  double det = a1 * d1 + b1 * d2 + c1 * d3;

  // Multiply by the adjoint
  double v1 = d1 * x[0] + e1 * x[1] + f1 * x[2];
  double v2 = d2 * x[0] + e2 * x[1] + f2 * x[2];
  double v3 = d3 * x[0] + e3 * x[1] + f3 * x[2];

  // Divide by the determinant
  y[0] = v1 / det;
  y[1] = v2 / det;
  y[2] = v3 / det;
}

//------------------------------------------------------------------------------
void vtkMath::LinearSolve3x3(const float A[3][3], const float x[3], float y[3])
{
  vtkLinearSolve3x3(A, x, y);
}

//------------------------------------------------------------------------------
void vtkMath::LinearSolve3x3(const double A[3][3], const double x[3], double y[3])
{
  vtkLinearSolve3x3(A, x, y);
}

//------------------------------------------------------------------------------
template <class T1, class T2, class T3>
inline void vtkMultiply3x3(const T1 A[3][3], const T2 v[3], T3 u[3])
{
  T3 x = A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2];
  T3 y = A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2];
  T3 z = A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2];

  u[0] = x;
  u[1] = y;
  u[2] = z;
}

//------------------------------------------------------------------------------
void vtkMath::Multiply3x3(const float A[3][3], const float v[3], float u[3])
{
  vtkMultiply3x3(A, v, u);
}

//------------------------------------------------------------------------------
void vtkMath::Multiply3x3(const double A[3][3], const double v[3], double u[3])
{
  vtkMultiply3x3(A, v, u);
}

//------------------------------------------------------------------------------
template <class T, class T2, class T3>
inline void vtkMultiplyMatrix3x3(const T A[3][3], const T2 B[3][3], T3 C[3][3])
{
  T3 D[3][3];

  for (int i = 0; i < 3; ++i)
  {
    D[0][i] = A[0][0] * B[0][i] + A[0][1] * B[1][i] + A[0][2] * B[2][i];
    D[1][i] = A[1][0] * B[0][i] + A[1][1] * B[1][i] + A[1][2] * B[2][i];
    D[2][i] = A[2][0] * B[0][i] + A[2][1] * B[1][i] + A[2][2] * B[2][i];
  }

  for (int j = 0; j < 3; ++j)
  {
    C[j][0] = D[j][0];
    C[j][1] = D[j][1];
    C[j][2] = D[j][2];
  }
}

//------------------------------------------------------------------------------
void vtkMath::Multiply3x3(const float A[3][3], const float B[3][3], float C[3][3])
{
  vtkMultiplyMatrix3x3(A, B, C);
}

//------------------------------------------------------------------------------
void vtkMath::Multiply3x3(const double A[3][3], const double B[3][3], double C[3][3])
{
  vtkMultiplyMatrix3x3(A, B, C);
}

//------------------------------------------------------------------------------
void vtkMath::MultiplyMatrix(const double* const* A, const double* const* B, unsigned int rowA,
  unsigned int colA, unsigned int rowB, unsigned int colB, double** C)
{
  // we need colA == rowB
  if (colA != rowB)
  {
    vtkGenericWarningMacro("Number of columns of A must match number of rows of B.");
  }

  // output matrix is rowA*colB

  // output row
  for (unsigned int i = 0; i < rowA; ++i)
  {
    // output col
    for (unsigned int j = 0; j < colB; ++j)
    {
      C[i][j] = 0;
      // sum for this point
      for (unsigned int k = 0; k < colA; ++k)
      {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

//------------------------------------------------------------------------------
template <class T1, class T2>
inline void vtkTranspose3x3(const T1 A[3][3], T2 AT[3][3])
{
  T2 tmp;
  tmp = A[1][0];
  AT[1][0] = A[0][1];
  AT[0][1] = tmp;
  tmp = A[2][0];
  AT[2][0] = A[0][2];
  AT[0][2] = tmp;
  tmp = A[2][1];
  AT[2][1] = A[1][2];
  AT[1][2] = tmp;

  AT[0][0] = A[0][0];
  AT[1][1] = A[1][1];
  AT[2][2] = A[2][2];
}

//------------------------------------------------------------------------------
void vtkMath::Transpose3x3(const float A[3][3], float AT[3][3])
{
  vtkTranspose3x3(A, AT);
}

//------------------------------------------------------------------------------
void vtkMath::Transpose3x3(const double A[3][3], double AT[3][3])
{
  vtkTranspose3x3(A, AT);
}

//------------------------------------------------------------------------------
template <class T1, class T2>
inline void vtkInvert3x3(const T1 A[3][3], T2 AI[3][3])
{
  double a1 = A[0][0];
  double b1 = A[0][1];
  double c1 = A[0][2];
  double a2 = A[1][0];
  double b2 = A[1][1];
  double c2 = A[1][2];
  double a3 = A[2][0];
  double b3 = A[2][1];
  double c3 = A[2][2];

  // Compute the adjoint
  double d1 = vtkMath::Determinant2x2(b2, b3, c2, c3);
  double d2 = -vtkMath::Determinant2x2(a2, a3, c2, c3);
  double d3 = vtkMath::Determinant2x2(a2, a3, b2, b3);

  double e1 = -vtkMath::Determinant2x2(b1, b3, c1, c3);
  double e2 = vtkMath::Determinant2x2(a1, a3, c1, c3);
  double e3 = -vtkMath::Determinant2x2(a1, a3, b1, b3);

  double f1 = vtkMath::Determinant2x2(b1, b2, c1, c2);
  double f2 = -vtkMath::Determinant2x2(a1, a2, c1, c2);
  double f3 = vtkMath::Determinant2x2(a1, a2, b1, b2);

  // Divide by the determinant
  double det = a1 * d1 + b1 * d2 + c1 * d3;

  AI[0][0] = d1 / det;
  AI[1][0] = d2 / det;
  AI[2][0] = d3 / det;

  AI[0][1] = e1 / det;
  AI[1][1] = e2 / det;
  AI[2][1] = e3 / det;

  AI[0][2] = f1 / det;
  AI[1][2] = f2 / det;
  AI[2][2] = f3 / det;
}

//------------------------------------------------------------------------------
void vtkMath::Invert3x3(const float A[3][3], float AI[3][3])
{
  vtkInvert3x3(A, AI);
}

//------------------------------------------------------------------------------
void vtkMath::Invert3x3(const double A[3][3], double AI[3][3])
{
  vtkInvert3x3(A, AI);
}

//------------------------------------------------------------------------------
template <class T>
inline void vtkIdentity3x3(T A[3][3])
{
  for (int i = 0; i < 3; ++i)
  {
    A[i][0] = A[i][1] = A[i][2] = T(0.0);
    A[i][i] = 1.0;
  }
}

//------------------------------------------------------------------------------
void vtkMath::Identity3x3(float A[3][3])
{
  vtkIdentity3x3(A);
}

//------------------------------------------------------------------------------
void vtkMath::Identity3x3(double A[3][3])
{
  vtkIdentity3x3(A);
}

//------------------------------------------------------------------------------
// Multiplying two quaternions
template <class T>
inline void vtkQuaternionMultiplication(const T q1[4], const T q2[4], T q[4])
{
  T ww = q1[0] * q2[0];
  T wx = q1[0] * q2[1];
  T wy = q1[0] * q2[2];
  T wz = q1[0] * q2[3];

  T xw = q1[1] * q2[0];
  T xx = q1[1] * q2[1];
  T xy = q1[1] * q2[2];
  T xz = q1[1] * q2[3];

  T yw = q1[2] * q2[0];
  T yx = q1[2] * q2[1];
  T yy = q1[2] * q2[2];
  T yz = q1[2] * q2[3];

  T zw = q1[3] * q2[0];
  T zx = q1[3] * q2[1];
  T zy = q1[3] * q2[2];
  T zz = q1[3] * q2[3];

  q[0] = ww - xx - yy - zz;
  q[1] = wx + xw + yz - zy;
  q[2] = wy - xz + yw + zx;
  q[3] = wz + xy - yx + zw;
}

//------------------------------------------------------------------------------
void vtkMath::MultiplyQuaternion(const float q1[4], const float q2[4], float q[4])
{
  vtkQuaternionMultiplication(q1, q2, q);
}

//------------------------------------------------------------------------------
void vtkMath::MultiplyQuaternion(const double q1[4], const double q2[4], double q[4])
{
  vtkQuaternionMultiplication(q1, q2, q);
}

//----------------------------------------------------------------------------
void vtkMath::RotateVectorByNormalizedQuaternion(const float v[3], const float q[4], float r[3])
{
  float f = std::sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  float a[3];
  if (f != 0.0)
  {
    a[0] = q[1] / f;
    a[1] = q[2] / f;
    a[2] = q[3] / f;

    // atan2() provides a more accurate angle result than acos()
    float t = 2.0 * atan2(f, q[0]);

    float cosT = cos(t);
    float sinT = sin(t);
    float dotKV = a[0] * v[0] + a[1] * v[1] + a[2] * v[2];
    float crossKV[3];
    vtkMath::Cross(a, v, crossKV);

    r[0] = v[0] * cosT + crossKV[0] * sinT + a[0] * dotKV * (1.0 - cosT);
    r[1] = v[1] * cosT + crossKV[1] * sinT + a[1] * dotKV * (1.0 - cosT);
    r[2] = v[2] * cosT + crossKV[2] * sinT + a[2] * dotKV * (1.0 - cosT);
  }
  else
  {
    r[0] = v[0];
    r[1] = v[1];
    r[2] = v[2];
  }
}

void vtkMath::RotateVectorByNormalizedQuaternion(const double v[3], const double q[4], double r[3])
{
  double f = std::sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  double a[3];
  if (f != 0.0)
  {
    a[0] = q[1] / f;
    a[1] = q[2] / f;
    a[2] = q[3] / f;

    // atan2() provides a more accurate angle result than acos()
    double t = 2.0 * atan2(f, q[0]);

    double cosT = cos(t);
    double sinT = sin(t);
    double dotKV = a[0] * v[0] + a[1] * v[1] + a[2] * v[2];
    double crossKV[3];
    vtkMath::Cross(a, v, crossKV);

    r[0] = v[0] * cosT + crossKV[0] * sinT + a[0] * dotKV * (1.0 - cosT);
    r[1] = v[1] * cosT + crossKV[1] * sinT + a[1] * dotKV * (1.0 - cosT);
    r[2] = v[2] * cosT + crossKV[2] * sinT + a[2] * dotKV * (1.0 - cosT);
  }
  else
  {
    r[0] = v[0];
    r[1] = v[1];
    r[2] = v[2];
  }
}

void vtkMath::RotateVectorByWXYZ(const float v[3], const float q[4], float r[3])
{
  float cosT = cos(q[0]);
  float sinT = sin(q[0]);
  float dotKV = q[1] * v[0] + q[2] * v[1] + q[3] * v[2];
  float crossKV[3];
  vtkMath::Cross(&(q[1]), v, crossKV);

  r[0] = v[0] * cosT + crossKV[0] * sinT + q[1] * dotKV * (1.0 - cosT);
  r[1] = v[1] * cosT + crossKV[1] * sinT + q[2] * dotKV * (1.0 - cosT);
  r[2] = v[2] * cosT + crossKV[2] * sinT + q[3] * dotKV * (1.0 - cosT);
}

void vtkMath::RotateVectorByWXYZ(const double v[3], const double q[4], double r[3])
{
  double cosT = cos(q[0]);
  double sinT = sin(q[0]);
  double dotKV = q[1] * v[0] + q[2] * v[1] + q[3] * v[2];
  double crossKV[3];
  vtkMath::Cross(&(q[1]), v, crossKV);

  r[0] = v[0] * cosT + crossKV[0] * sinT + q[1] * dotKV * (1.0 - cosT);
  r[1] = v[1] * cosT + crossKV[1] * sinT + q[2] * dotKV * (1.0 - cosT);
  r[2] = v[2] * cosT + crossKV[2] * sinT + q[3] * dotKV * (1.0 - cosT);
}

//------------------------------------------------------------------------------
//  The orthogonalization is done via quaternions in order to avoid
//  having to use a singular value decomposition algorithm.
template <class T1, class T2>
inline void vtkOrthogonalize3x3(const T1 A[3][3], T2 B[3][3])
{
  int i;

  // copy the matrix
  for (i = 0; i < 3; ++i)
  {
    B[0][i] = A[0][i];
    B[1][i] = A[1][i];
    B[2][i] = A[2][i];
  }

  // Pivot the matrix to improve accuracy
  T2 scale[3];
  int index[3];
  T2 largest;

  // Loop over rows to get implicit scaling information
  for (i = 0; i < 3; ++i)
  {
    T2 x1 = fabs(B[i][0]);
    T2 x2 = fabs(B[i][1]);
    T2 x3 = fabs(B[i][2]);
    largest = (x2 > x1 ? x2 : x1);
    largest = (x3 > largest ? x3 : largest);
    scale[i] = 1;
    if (largest != 0)
    {
      scale[i] /= largest;
    }
  }

  // first column
  T2 x1 = fabs(B[0][0]) * scale[0];
  T2 x2 = fabs(B[1][0]) * scale[1];
  T2 x3 = fabs(B[2][0]) * scale[2];
  index[0] = 0;
  largest = x1;
  if (x2 >= largest)
  {
    largest = x2;
    index[0] = 1;
  }
  if (x3 >= largest)
  {
    index[0] = 2;
  }
  if (index[0] != 0)
  {
    vtkSwapVectors3(B[index[0]], B[0]);
    scale[index[0]] = scale[0];
  }

  // second column
  T2 y2 = fabs(B[1][1]) * scale[1];
  T2 y3 = fabs(B[2][1]) * scale[2];
  index[1] = 1;
  largest = y2;
  if (y3 >= largest)
  {
    index[1] = 2;
    vtkSwapVectors3(B[2], B[1]);
  }

  // third column
  index[2] = 2;

  // A quaternion can only describe a pure rotation, not
  // a rotation with a flip, therefore the flip must be
  // removed before the matrix is converted to a quaternion.
  bool flip = false;
  if (vtkDeterminant3x3(B) < 0)
  {
    flip = true;
    for (i = 0; i < 3; ++i)
    {
      B[0][i] = -B[0][i];
      B[1][i] = -B[1][i];
      B[2][i] = -B[2][i];
    }
  }

  // Do orthogonalization using a quaternion intermediate
  // (this, essentially, does the orthogonalization via
  // diagonalization of an appropriately constructed symmetric
  // 4x4 matrix rather than by doing SVD of the 3x3 matrix)
  T2 quat[4];
  vtkMath::Matrix3x3ToQuaternion(B, quat);
  vtkMath::QuaternionToMatrix3x3(quat, B);

  // Put the flip back into the orthogonalized matrix.
  if (flip)
  {
    for (i = 0; i < 3; ++i)
    {
      B[0][i] = -B[0][i];
      B[1][i] = -B[1][i];
      B[2][i] = -B[2][i];
    }
  }

  // Undo the pivoting
  if (index[1] != 1)
  {
    vtkSwapVectors3(B[index[1]], B[1]);
  }
  if (index[0] != 0)
  {
    vtkSwapVectors3(B[index[0]], B[0]);
  }
}

//------------------------------------------------------------------------------
void vtkMath::Orthogonalize3x3(const float A[3][3], float B[3][3])
{
  vtkOrthogonalize3x3(A, B);
}

//------------------------------------------------------------------------------
void vtkMath::Orthogonalize3x3(const double A[3][3], double B[3][3])
{
  vtkOrthogonalize3x3(A, B);
}

//------------------------------------------------------------------------------
float vtkMath::Norm(const float* x, int n)
{
  double sum = 0;
  for (int i = 0; i < n; ++i)
  {
    sum += x[i] * x[i];
  }

  return std::sqrt(sum);
}

//------------------------------------------------------------------------------
double vtkMath::Norm(const double* x, int n)
{
  double sum = 0;
  for (int i = 0; i < n; ++i)
  {
    sum += x[i] * x[i];
  }

  return std::sqrt(sum);
}

//------------------------------------------------------------------------------
bool vtkMath::ProjectVector(const float a[3], const float b[3], float projection[3])
{
  float bSquared = vtkMath::Dot(b, b);

  if (bSquared == 0)
  {
    projection[0] = 0;
    projection[1] = 0;
    projection[2] = 0;
    return false;
  }

  float scale = vtkMath::Dot(a, b) / bSquared;

  for (int i = 0; i < 3; ++i)
  {
    projection[i] = b[i];
  }
  vtkMath::MultiplyScalar(projection, scale);

  return true;
}

//------------------------------------------------------------------------------
bool vtkMath::ProjectVector(const double a[3], const double b[3], double projection[3])
{
  double bSquared = vtkMath::Dot(b, b);

  if (bSquared == 0)
  {
    projection[0] = 0;
    projection[1] = 0;
    projection[2] = 0;
    return false;
  }

  double scale = vtkMath::Dot(a, b) / bSquared;

  for (int i = 0; i < 3; ++i)
  {
    projection[i] = b[i];
  }
  vtkMath::MultiplyScalar(projection, scale);

  return true;
}

//------------------------------------------------------------------------------
bool vtkMath::ProjectVector2D(const float a[2], const float b[2], float projection[2])
{
  float bSquared = vtkMath::Dot2D(b, b);

  if (bSquared == 0)
  {
    projection[0] = 0;
    projection[1] = 0;
    return false;
  }

  float scale = vtkMath::Dot2D(a, b) / bSquared;

  for (int i = 0; i < 2; ++i)
  {
    projection[i] = b[i];
  }
  vtkMath::MultiplyScalar2D(projection, scale);

  return true;
}

//------------------------------------------------------------------------------
bool vtkMath::ProjectVector2D(const double a[2], const double b[2], double projection[2])
{
  double bSquared = vtkMath::Dot2D(b, b);

  if (bSquared == 0)
  {
    projection[0] = 0;
    projection[1] = 0;
    return false;
  }

  double scale = vtkMath::Dot2D(a, b) / bSquared;

  for (int i = 0; i < 2; ++i)
  {
    projection[i] = b[i];
  }
  vtkMath::MultiplyScalar2D(projection, scale);

  return true;
}

//------------------------------------------------------------------------------
// Extract the eigenvalues and eigenvectors from a 3x3 matrix.
// The eigenvectors (the columns of V) will be normalized.
// The eigenvectors are aligned optimally with the x, y, and z
// axes respectively.
template <class T1, class T2>
inline void vtkDiagonalize3x3(const T1 A[3][3], T2 w[3], T2 V[3][3])
{
  int i, j, k, maxI;
  T2 tmp, maxVal;

  // do the matrix[3][3] to **matrix conversion for Jacobi
  T2 C[3][3];
  T2 *ATemp[3], *VTemp[3];
  for (i = 0; i < 3; ++i)
  {
    C[i][0] = A[i][0];
    C[i][1] = A[i][1];
    C[i][2] = A[i][2];
    ATemp[i] = C[i];
    VTemp[i] = V[i];
  }

  // diagonalize using Jacobi
  vtkMath::JacobiN(ATemp, 3, w, VTemp);

  // if all the eigenvalues are the same, return identity matrix
  if (w[0] == w[1] && w[0] == w[2])
  {
    vtkMath::Identity3x3(V);
    return;
  }

  // transpose temporarily, it makes it easier to sort the eigenvectors
  vtkMath::Transpose3x3(V, V);

  // if two eigenvalues are the same, re-orthogonalize to optimally line
  // up the eigenvectors with the x, y, and z axes
  for (i = 0; i < 3; ++i)
  {
    if (w[(i + 1) % 3] == w[(i + 2) % 3]) // two eigenvalues are the same
    {
      // find maximum element of the independent eigenvector
      maxVal = fabs(V[i][0]);
      maxI = 0;
      for (j = 1; j < 3; ++j)
      {
        if (maxVal < (tmp = fabs(V[i][j])))
        {
          maxVal = tmp;
          maxI = j;
        }
      }
      // swap the eigenvector into its proper position
      if (maxI != i)
      {
        tmp = w[maxI];
        w[maxI] = w[i];
        w[i] = tmp;
        vtkSwapVectors3(V[i], V[maxI]);
      }
      // maximum element of eigenvector should be positive
      if (V[maxI][maxI] < 0)
      {
        V[maxI][0] = -V[maxI][0];
        V[maxI][1] = -V[maxI][1];
        V[maxI][2] = -V[maxI][2];
      }

      // re-orthogonalize the other two eigenvectors
      j = (maxI + 1) % 3;
      k = (maxI + 2) % 3;

      V[j][0] = 0.0;
      V[j][1] = 0.0;
      V[j][2] = 0.0;
      V[j][j] = 1.0;
      vtkMath::Cross(V[maxI], V[j], V[k]);
      vtkMath::Normalize(V[k]);
      vtkMath::Cross(V[k], V[maxI], V[j]);

      // transpose vectors back to columns
      vtkMath::Transpose3x3(V, V);
      return;
    }
  }

  // the three eigenvalues are different, just sort the eigenvectors
  // to align them with the x, y, and z axes

  // find the vector with the largest x element, make that vector
  // the first vector
  maxVal = fabs(V[0][0]);
  maxI = 0;
  for (i = 1; i < 3; ++i)
  {
    if (maxVal < (tmp = fabs(V[i][0])))
    {
      maxVal = tmp;
      maxI = i;
    }
  }
  // swap eigenvalue and eigenvector
  if (maxI != 0)
  {
    tmp = w[maxI];
    w[maxI] = w[0];
    w[0] = tmp;
    vtkSwapVectors3(V[maxI], V[0]);
  }
  // do the same for the y element
  if (fabs(V[1][1]) < fabs(V[2][1]))
  {
    tmp = w[2];
    w[2] = w[1];
    w[1] = tmp;
    vtkSwapVectors3(V[2], V[1]);
  }

  // ensure that the sign of the eigenvectors is correct
  for (i = 0; i < 2; ++i)
  {
    if (V[i][i] < 0)
    {
      V[i][0] = -V[i][0];
      V[i][1] = -V[i][1];
      V[i][2] = -V[i][2];
    }
  }
  // set sign of final eigenvector to ensure that determinant is positive
  if (vtkMath::Determinant3x3(V) < 0)
  {
    V[2][0] = -V[2][0];
    V[2][1] = -V[2][1];
    V[2][2] = -V[2][2];
  }

  // transpose the eigenvectors back again
  vtkMath::Transpose3x3(V, V);
}

//------------------------------------------------------------------------------
void vtkMath::Diagonalize3x3(const float A[3][3], float w[3], float V[3][3])
{
  vtkDiagonalize3x3(A, w, V);
}

//------------------------------------------------------------------------------
void vtkMath::Diagonalize3x3(const double A[3][3], double w[3], double V[3][3])
{
  vtkDiagonalize3x3(A, w, V);
}

//------------------------------------------------------------------------------
// Perform singular value decomposition on the matrix A:
//    A = U * W * VT
// where U and VT are orthogonal W is diagonal (the diagonal elements
// are returned in vector w).
// The matrices U and VT will both have positive determinants.
// The scale factors w are ordered according to how well the
// corresponding eigenvectors (in VT) match the x, y and z axes
// respectively.
//
// The singular value decomposition is used to decompose a linear
// transformation into a rotation, followed by a scale, followed
// by a second rotation.  The scale factors w will be negative if
// the determinant of matrix A is negative.
//
// Contributed by David Gobbi (dgobbi@irus.rri.on.ca)
template <class T1, class T2>
inline void vtkSingularValueDecomposition3x3(const T1 A[3][3], T2 U[3][3], T2 w[3], T2 VT[3][3])
{
  int i;
  T2 B[3][3];

  // copy so that A can be used for U or VT without risk
  for (i = 0; i < 3; ++i)
  {
    B[0][i] = A[0][i];
    B[1][i] = A[1][i];
    B[2][i] = A[2][i];
  }

  // temporarily flip if determinant is negative
  T2 d = vtkMath::Determinant3x3(B);
  if (d < 0)
  {
    for (i = 0; i < 3; ++i)
    {
      B[0][i] = -B[0][i];
      B[1][i] = -B[1][i];
      B[2][i] = -B[2][i];
    }
  }

  // orthogonalize, diagonalize, etc.
  vtkMath::Orthogonalize3x3(B, U);
  vtkMath::Transpose3x3(B, B);
  vtkMath::Multiply3x3(B, U, VT);
  vtkMath::Diagonalize3x3(VT, w, VT);
  vtkMath::Multiply3x3(U, VT, U);
  vtkMath::Transpose3x3(VT, VT);

  // re-create the flip
  if (d < 0)
  {
    w[0] = -w[0];
    w[1] = -w[1];
    w[2] = -w[2];
  }

  /* paranoia check: recombine to ensure that the SVD is correct
  vtkMath::Transpose3x3(B, B);

  if (d < 0)
  {
    for (i = 0; i < 3; ++i)
    {
      B[0][i] = -B[0][i];
      B[1][i] = -B[1][i];
      B[2][i] = -B[2][i];
    }
  }

  int j;
  T2 maxerr = 0;
  T2 tmp;
  T2 M[3][3];
  T2 W[3][3];
  vtkMath::Identity3x3(W);
  W[0][0] = w[0]; W[1][1] = w[1]; W[2][2] = w[2];
  vtkMath::Identity3x3(M);
  vtkMath::Multiply3x3(M, U, M);
  vtkMath::Multiply3x3(M, W, M);
  vtkMath::Multiply3x3(M, VT, M);

  for (i = 0; i < 3; ++i)
  {
    for (j = 0; j < 3; ++j)
    {
      if ((tmp = fabs(B[i][j] - M[i][j])) > maxerr)
      {
        maxerr = tmp;
      }
    }
  }

  vtkGenericWarningMacro("SingularValueDecomposition max error = " << maxerr);
  */
}

//------------------------------------------------------------------------------
void vtkMath::SingularValueDecomposition3x3(
  const float A[3][3], float U[3][3], float w[3], float VT[3][3])
{
  vtkSingularValueDecomposition3x3(A, U, w, VT);
}

//------------------------------------------------------------------------------
void vtkMath::SingularValueDecomposition3x3(
  const double A[3][3], double U[3][3], double w[3], double VT[3][3])
{
  vtkSingularValueDecomposition3x3(A, U, w, VT);
}

//------------------------------------------------------------------------------
void vtkMath::RGBToHSV(float r, float g, float b, float* h, float* s, float* v)
{
  double dh, ds, dv;
  vtkMath::RGBToHSV(r, g, b, &dh, &ds, &dv);
  *h = static_cast<float>(dh);
  *s = static_cast<float>(ds);
  *v = static_cast<float>(dv);
}

//------------------------------------------------------------------------------
void vtkMath::RGBToHSV(double r, double g, double b, double* h, double* s, double* v)
{
  const double onethird = 1.0 / 3.0;
  const double onesixth = 1.0 / 6.0;
  const double twothird = 2.0 / 3.0;

  double cmax = r;
  double cmin = r;
  if (g > cmax)
  {
    cmax = g;
  }
  else if (g < cmin)
  {
    cmin = g;
  }
  if (b > cmax)
  {
    cmax = b;
  }
  else if (b < cmin)
  {
    cmin = b;
  }
  *v = cmax;

  if (*v > 0.0)
  {
    *s = (cmax - cmin) / cmax;
  }
  else
  {
    *s = 0.0;
  }
  if (*s > 0)
  {
    if (r == cmax)
    {
      *h = onesixth * (g - b) / (cmax - cmin);
    }
    else if (g == cmax)
    {
      *h = onethird + onesixth * (b - r) / (cmax - cmin);
    }
    else
    {
      *h = twothird + onesixth * (r - g) / (cmax - cmin);
    }
    if (*h < 0.0)
    {
      *h += 1.0;
    }
  }
  else
  {
    *h = 0.0;
  }
}

//------------------------------------------------------------------------------
void vtkMath::HSVToRGB(float h, float s, float v, float* r, float* g, float* b)
{
  double dr, dg, db;
  vtkMath::HSVToRGB(h, s, v, &dr, &dg, &db);
  *r = static_cast<float>(dr);
  *g = static_cast<float>(dg);
  *b = static_cast<float>(db);
}

//------------------------------------------------------------------------------
void vtkMath::HSVToRGB(double h, double s, double v, double* r, double* g, double* b)
{
  const double onethird = 1.0 / 3.0;
  const double onesixth = 1.0 / 6.0;
  const double twothird = 2.0 / 3.0;
  const double fivesixth = 5.0 / 6.0;

  // compute RGB from HSV
  if (h > onesixth && h <= onethird) // green/red
  {
    *g = 1.0;
    *r = (onethird - h) / onesixth;
    *b = 0.0;
  }
  else if (h > onethird && h <= 0.5) // green/blue
  {
    *g = 1.0;
    *b = (h - onethird) / onesixth;
    *r = 0.0;
  }
  else if (h > 0.5 && h <= twothird) // blue/green
  {
    *b = 1.0;
    *g = (twothird - h) / onesixth;
    *r = 0.0;
  }
  else if (h > twothird && h <= fivesixth) // blue/red
  {
    *b = 1.0;
    *r = (h - twothird) / onesixth;
    *g = 0.0;
  }
  else if (h > fivesixth && h <= 1.0) // red/blue
  {
    *r = 1.0;
    *b = (1.0 - h) / onesixth;
    *g = 0.0;
  }
  else // red/green
  {
    *r = 1.0;
    *g = h / onesixth;
    *b = 0.0;
  }

  // add Saturation to the equation.
  *r = (s * *r + (1.0 - s));
  *g = (s * *g + (1.0 - s));
  *b = (s * *b + (1.0 - s));

  *r *= v;
  *g *= v;
  *b *= v;
}

//------------------------------------------------------------------------------
void vtkMath::LabToXYZ(double L, double a, double b, double* x, double* y, double* z)
{
  // LAB to XYZ
  double var_Y = (L + 16) / 116;
  double var_X = a / 500 + var_Y;
  double var_Z = var_Y - b / 200;

  if (pow(var_Y, 3) > 0.008856)
  {
    var_Y = pow(var_Y, 3);
  }
  else
  {
    var_Y = (var_Y - 16.0 / 116.0) / 7.787;
  }

  if (pow(var_X, 3) > 0.008856)
  {
    var_X = pow(var_X, 3);
  }
  else
  {
    var_X = (var_X - 16.0 / 116.0) / 7.787;
  }

  if (pow(var_Z, 3) > 0.008856)
  {
    var_Z = pow(var_Z, 3);
  }
  else
  {
    var_Z = (var_Z - 16.0 / 116.0) / 7.787;
  }
  const double ref_X = 0.9505;
  const double ref_Y = 1.000;
  const double ref_Z = 1.089;
  *x = ref_X * var_X; // ref_X = 0.9505  Observer= 2 deg Illuminant= D65
  *y = ref_Y * var_Y; // ref_Y = 1.000
  *z = ref_Z * var_Z; // ref_Z = 1.089
}

//------------------------------------------------------------------------------
void vtkMath::XYZToLab(double x, double y, double z, double* L, double* a, double* b)
{
  const double ref_X = 0.9505;
  const double ref_Y = 1.000;
  const double ref_Z = 1.089;
  double var_X = x / ref_X; // ref_X = 0.9505  Observer= 2 deg, Illuminant= D65
  double var_Y = y / ref_Y; // ref_Y = 1.000
  double var_Z = z / ref_Z; // ref_Z = 1.089

  if (var_X > 0.008856)
  {
    var_X = pow(var_X, 1.0 / 3.0);
  }
  else
  {
    var_X = (7.787 * var_X) + (16.0 / 116.0);
  }
  if (var_Y > 0.008856)
  {
    var_Y = pow(var_Y, 1.0 / 3.0);
  }
  else
  {
    var_Y = (7.787 * var_Y) + (16.0 / 116.0);
  }
  if (var_Z > 0.008856)
  {
    var_Z = pow(var_Z, 1.0 / 3.0);
  }
  else
  {
    var_Z = (7.787 * var_Z) + (16.0 / 116.0);
  }

  *L = (116 * var_Y) - 16;
  *a = 500 * (var_X - var_Y);
  *b = 200 * (var_Y - var_Z);
}

//------------------------------------------------------------------------------
void vtkMath::XYZToRGB(double x, double y, double z, double* r, double* g, double* b)
{
  // double ref_X = 0.9505;        //Observer = 2 deg Illuminant = D65
  // double ref_Y = 1.000;
  // double ref_Z = 1.089;

  *r = x * 3.2406 + y * -1.5372 + z * -0.4986;
  *g = x * -0.9689 + y * 1.8758 + z * 0.0415;
  *b = x * 0.0557 + y * -0.2040 + z * 1.0570;

  // The following performs a "gamma correction" specified by the sRGB color
  // space.  sRGB is defined by a canonical definition of a display monitor and
  // has been standardized by the International Electrotechnical Commission (IEC
  // 61966-2-1).  The nonlinearity of the correction is designed to make the
  // colors more perceptually uniform.  This color space has been adopted by
  // several applications including Adobe Photoshop and Microsoft Windows color
  // management.  OpenGL is agnostic on its RGB color space, but it is reasonable
  // to assume it is close to this one.
  if (*r > 0.0031308)
  {
    *r = 1.055 * (pow(*r, (1 / 2.4))) - 0.055;
  }
  else
  {
    *r = 12.92 * (*r);
  }
  if (*g > 0.0031308)
  {
    *g = 1.055 * (pow(*g, (1 / 2.4))) - 0.055;
  }
  else
  {
    *g = 12.92 * (*g);
  }
  if (*b > 0.0031308)
  {
    *b = 1.055 * (pow(*b, (1 / 2.4))) - 0.055;
  }
  else
  {
    *b = 12.92 * (*b);
  }

  // Clip colors. ideally we would do something that is perceptually closest
  // (since we can see colors outside of the display gamut), but this seems to
  // work well enough.
  double maxVal = *r;
  if (maxVal < *g)
  {
    maxVal = *g;
  }
  if (maxVal < *b)
  {
    maxVal = *b;
  }
  if (maxVal > 1.0)
  {
    *r /= maxVal;
    *g /= maxVal;
    *b /= maxVal;
  }
  if (*r < 0)
  {
    *r = 0;
  }
  if (*g < 0)
  {
    *g = 0;
  }
  if (*b < 0)
  {
    *b = 0;
  }
}

//------------------------------------------------------------------------------
void vtkMath::RGBToXYZ(double r, double g, double b, double* x, double* y, double* z)
{
  // The following performs a "gamma correction" specified by the sRGB color
  // space.  sRGB is defined by a canonical definition of a display monitor and
  // has been standardized by the International Electrotechnical Commission (IEC
  // 61966-2-1).  The nonlinearity of the correction is designed to make the
  // colors more perceptually uniform.  This color space has been adopted by
  // several applications including Adobe Photoshop and Microsoft Windows color
  // management.  OpenGL is agnostic on its RGB color space, but it is reasonable
  // to assume it is close to this one.
  if (r > 0.04045)
  {
    r = pow((r + 0.055) / 1.055, 2.4);
  }
  else
  {
    r = r / 12.92;
  }
  if (g > 0.04045)
  {
    g = pow((g + 0.055) / 1.055, 2.4);
  }
  else
  {
    g = g / 12.92;
  }
  if (b > 0.04045)
  {
    b = pow((b + 0.055) / 1.055, 2.4);
  }
  else
  {
    b = b / 12.92;
  }

  // Observer. = 2 deg, Illuminant = D65
  *x = r * 0.4124 + g * 0.3576 + b * 0.1805;
  *y = r * 0.2126 + g * 0.7152 + b * 0.0722;
  *z = r * 0.0193 + g * 0.1192 + b * 0.9505;
}

//------------------------------------------------------------------------------
void vtkMath::RGBToLab(double red, double green, double blue, double* L, double* a, double* b)
{
  double x, y, z;
  vtkMath::RGBToXYZ(red, green, blue, &x, &y, &z);
  vtkMath::XYZToLab(x, y, z, L, a, b);
}

//------------------------------------------------------------------------------
void vtkMath::LabToRGB(double L, double a, double b, double* red, double* green, double* blue)
{
  double x, y, z;
  vtkMath::LabToXYZ(L, a, b, &x, &y, &z);
  vtkMath::XYZToRGB(x, y, z, red, green, blue);
}

//------------------------------------------------------------------------------
void vtkMath::ClampValues(double* values, int nb_values, const double range[2])
{
  if (!values || nb_values <= 0 || !range)
  {
    return;
  }

  const double* values_end = values + nb_values;
  while (values < values_end)
  {
    *values = vtkMath::ClampValue(*values, range[0], range[1]);
    ++values;
  }
}

//------------------------------------------------------------------------------
void vtkMath::ClampValues(
  const double* values, int nb_values, const double range[2], double* clamped_values)
{
  if (!values || nb_values <= 0 || !range || !clamped_values)
  {
    return;
  }

  const double* values_end = values + nb_values;
  while (values < values_end)
  {
    *clamped_values = vtkMath::ClampValue(*values, range[0], range[1]);
    ++values;
    ++clamped_values;
  }
}

//------------------------------------------------------------------------------
int vtkMath::GetScalarTypeFittingRange(
  double range_min, double range_max, double scale, double shift)
{
  class TypeRange
  {
  public:
    int Type;
    double Min;
    double Max;
  };

  const TypeRange FloatTypes[] = { { VTK_FLOAT, VTK_FLOAT_MIN, VTK_FLOAT_MAX },
    { VTK_DOUBLE, VTK_DOUBLE_MIN, VTK_DOUBLE_MAX } };

  const TypeRange IntTypes[] = { { VTK_BIT, VTK_BIT_MIN, VTK_BIT_MAX },
    { VTK_CHAR, VTK_CHAR_MIN, VTK_CHAR_MAX },
    { VTK_SIGNED_CHAR, VTK_SIGNED_CHAR_MIN, VTK_SIGNED_CHAR_MAX },
    { VTK_UNSIGNED_CHAR, VTK_UNSIGNED_CHAR_MIN, VTK_UNSIGNED_CHAR_MAX },
    { VTK_SHORT, VTK_SHORT_MIN, VTK_SHORT_MAX },
    { VTK_UNSIGNED_SHORT, VTK_UNSIGNED_SHORT_MIN, VTK_UNSIGNED_SHORT_MAX },
    { VTK_INT, VTK_INT_MIN, VTK_INT_MAX },
    { VTK_UNSIGNED_INT, VTK_UNSIGNED_INT_MIN, VTK_UNSIGNED_INT_MAX },
    { VTK_LONG, static_cast<double>(VTK_LONG_MIN), static_cast<double>(VTK_LONG_MAX) },
    { VTK_UNSIGNED_LONG, static_cast<double>(VTK_UNSIGNED_LONG_MIN),
      static_cast<double>(VTK_UNSIGNED_LONG_MAX) },
    { VTK_LONG_LONG, static_cast<double>(VTK_LONG_LONG_MIN),
      static_cast<double>(VTK_LONG_LONG_MAX) },
    { VTK_UNSIGNED_LONG_LONG, static_cast<double>(VTK_UNSIGNED_LONG_LONG_MIN),
      static_cast<double>(VTK_UNSIGNED_LONG_LONG_MAX) } };

  // If the range, scale or shift are decimal number, just browse
  // the decimal types

  double intpart;

  int range_min_is_int = (modf(range_min, &intpart) == 0.0);
  int range_max_is_int = (modf(range_max, &intpart) == 0.0);
  int scale_is_int = (modf(scale, &intpart) == 0.0);
  int shift_is_int = (modf(shift, &intpart) == 0.0);

  range_min = range_min * scale + shift;
  range_max = range_max * scale + shift;

  if (range_min_is_int && range_max_is_int && scale_is_int && shift_is_int)
  {
    for (unsigned int i = 0; i < sizeof(IntTypes) / sizeof(TypeRange); ++i)
    {
      if (IntTypes[i].Min <= range_min && range_max <= IntTypes[i].Max)
      {
        return IntTypes[i].Type;
      }
    }
  }

  for (unsigned int i = 0; i < sizeof(FloatTypes) / sizeof(TypeRange); ++i)
  {
    if (FloatTypes[i].Min <= range_min && range_max <= FloatTypes[i].Max)
    {
      return FloatTypes[i].Type;
    }
  }

  return -1;
}

//------------------------------------------------------------------------------
vtkTypeBool vtkMath::GetAdjustedScalarRange(vtkDataArray* array, int comp, double range[2])
{
  if (!array || comp < 0 || comp >= array->GetNumberOfComponents())
  {
    return 0;
  }

  array->GetRange(range, comp);

  switch (array->GetDataType())
  {
    case VTK_UNSIGNED_CHAR:
      range[0] = static_cast<double>(array->GetDataTypeMin());
      range[1] = static_cast<double>(array->GetDataTypeMax());
      break;

    case VTK_UNSIGNED_SHORT:
      range[0] = static_cast<double>(array->GetDataTypeMin());
      if (range[1] <= 4095.0)
      {
        if (range[1] > VTK_UNSIGNED_CHAR_MAX)
        {
          range[1] = 4095.0;
        }
      }
      else
      {
        range[1] = static_cast<double>(array->GetDataTypeMax());
      }
      break;
    default:
      assert("check: impossible case." && 0); // reaching this line is a bug.
      break;
  }

  return 1;
}

//------------------------------------------------------------------------------
vtkTypeBool vtkMath::ExtentIsWithinOtherExtent(const int extent1[6], const int extent2[6])
{
  if (!extent1 || !extent2)
  {
    return 0;
  }

  for (int i = 0; i < 6; i += 2)
  {
    if (extent1[i] < extent2[i] || extent1[i] > extent2[i + 1] || extent1[i + 1] < extent2[i] ||
      extent1[i + 1] > extent2[i + 1])
    {
      return 0;
    }
  }

  return 1;
}

//------------------------------------------------------------------------------

vtkTypeBool vtkMath::BoundsIsWithinOtherBounds(
  const double bounds1[6], const double bounds2[6], const double delta[3])
{
  if (!bounds1 || !bounds2)
  {
    return 0;
  }
  for (int i = 0; i < 6; i += 2)
  {

    if (bounds1[i] + delta[i / 2] < bounds2[i] || bounds1[i] - delta[i / 2] > bounds2[i + 1] ||
      bounds1[i + 1] + delta[i / 2] < bounds2[i] || bounds1[i + 1] - delta[i / 2] > bounds2[i + 1])
      return 0;
  }
  return 1;
}

//------------------------------------------------------------------------------
vtkTypeBool vtkMath::PointIsWithinBounds(
  const double point[3], const double bounds[6], const double delta[3])
{
  if (!point || !bounds || !delta)
  {
    return 0;
  }
  for (int i = 0; i < 3; ++i)
  {
    if (point[i] + delta[i] < bounds[2 * i] || point[i] - delta[i] > bounds[2 * i + 1])
    {
      return 0;
    }
  }
  return 1;
}

//------------------------------------------------------------------------------
int vtkMath::PlaneIntersectsAABB(
  const double bounds[6], const double normal[3], const double point[3])
{
  if (!bounds || !point || !normal)
  {
    return -2;
  }

  double nPoint[3];
  double pPoint[3];

  // X Component
  if (normal[0] >= 0)
  {
    nPoint[0] = bounds[0];
    pPoint[0] = bounds[1];
  }
  else
  {
    nPoint[0] = bounds[1];
    pPoint[0] = bounds[0];
  }

  // Y Component
  if (normal[1] >= 0)
  {
    nPoint[1] = bounds[2];
    pPoint[1] = bounds[3];
  }
  else
  {
    nPoint[1] = bounds[3];
    pPoint[1] = bounds[2];
  }

  // Z Component
  if (normal[2] >= 0)
  {
    nPoint[2] = bounds[4];
    pPoint[2] = bounds[5];
  }
  else
  {
    nPoint[2] = bounds[5];
    pPoint[2] = bounds[4];
  }

  // Compute distances from nPoint/pPoint to the plane
  // Distance = unit_N  *  (P_x - P_plane)
  //          = a * px_1 + b * px_2 + c * px_3 - d
  double const d = vtkMath::Dot(normal, point);

  if ((nPoint[0] * normal[0] + nPoint[1] * normal[1] + nPoint[2] * normal[2] - d) > 0)
  {
    return 1;
  }
  else if ((pPoint[0] * normal[0] + pPoint[1] * normal[1] + pPoint[2] * normal[2] - d) < 0)
  {
    return -1;
  }

  return 0;
}

//------------------------------------------------------------------------------
double vtkMath::AngleBetweenVectors(const double v1[3], const double v2[3])
{
  double cross[3];
  vtkMath::Cross(v1, v2, cross);
  return atan2(vtkMath::Norm(cross), vtkMath::Dot(v1, v2));
}

//------------------------------------------------------------------------------
double vtkMath::SignedAngleBetweenVectors(
  const double v1[3], const double v2[3], const double vn[3])
{
  double cross[3];
  vtkMath::Cross(v1, v2, cross);
  double angle = atan2(vtkMath::Norm(cross), vtkMath::Dot(v1, v2));
  return vtkMath::Dot(cross, vn) >= 0 ? angle : -angle;
}

//------------------------------------------------------------------------------
double vtkMath::GaussianAmplitude(const double variance, const double distanceFromMean)
{
  return 1. / (std::sqrt(2. * vtkMath::Pi() * variance)) *
    exp(-(pow(distanceFromMean, 2)) / (2. * variance));
}

//------------------------------------------------------------------------------
double vtkMath::GaussianAmplitude(const double mean, const double variance, const double position)
{
  double distanceToMean = std::abs(mean - position);
  return GaussianAmplitude(variance, distanceToMean);
}

//------------------------------------------------------------------------------
double vtkMath::GaussianWeight(const double variance, const double distanceFromMean)
{
  return exp(-(pow(distanceFromMean, 2)) / (2. * variance));
}

//------------------------------------------------------------------------------
double vtkMath::GaussianWeight(const double mean, const double variance, const double position)
{
  double distanceToMean = std::abs(mean - position);
  return GaussianWeight(variance, distanceToMean);
}

//------------------------------------------------------------------------------
double vtkMath::Solve3PointCircle(
  const double p1[3], const double p2[3], const double p3[3], double center[3])
{
  double v21[3], v32[3], v13[3];
  double v12[3], v23[3], v31[3];
  for (int i = 0; i < 3; ++i)
  {
    v21[i] = p1[i] - p2[i];
    v32[i] = p2[i] - p3[i];
    v13[i] = p3[i] - p1[i];
    v12[i] = -v21[i];
    v23[i] = -v32[i];
    v31[i] = -v13[i];
  }

  double norm12 = vtkMath::Norm(v12);
  double norm23 = vtkMath::Norm(v23);
  double norm13 = vtkMath::Norm(v13);

  double crossv21v32[3];
  vtkMath::Cross(v21, v32, crossv21v32);
  double normCross = vtkMath::Norm(crossv21v32);

  double radius = (norm12 * norm23 * norm13) / (2. * normCross);

  double alpha = ((norm23 * norm23) * vtkMath::Dot(v21, v31)) / (2. * normCross * normCross);
  double beta = ((norm13 * norm13) * vtkMath::Dot(v12, v32)) / (2. * normCross * normCross);
  double gamma = ((norm12 * norm12) * vtkMath::Dot(v13, v23)) / (2. * normCross * normCross);

  for (int i = 0; i < 3; ++i)
  {
    center[i] = alpha * p1[i] + beta * p2[i] + gamma * p3[i];
  }
  return radius;
}

//------------------------------------------------------------------------------
void vtkMath::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Seed: " << vtkMath::Internal->Uniform->GetSeed() << "\n";
}

//------------------------------------------------------------------------------
double vtkMath::Inf()
{
  return std::numeric_limits<double>::infinity();
}

//------------------------------------------------------------------------------
double vtkMath::NegInf()
{
  return -std::numeric_limits<double>::infinity();
}

//------------------------------------------------------------------------------
double vtkMath::Nan()
{
  return std::numeric_limits<double>::quiet_NaN();
}

//------------------------------------------------------------------------------
#ifndef VTK_MATH_ISINF_IS_INLINE
vtkTypeBool vtkMath::IsInf(double x)
{
  return (!vtkMath::IsNan(x) && !((x < vtkMath::Inf()) && (x > vtkMath::NegInf())));
}
#endif

//------------------------------------------------------------------------------
#ifndef VTK_MATH_ISNAN_IS_INLINE
vtkTypeBool vtkMath::IsNan(double x)
{
  return !((x <= 0.0) || (x >= 0.0));
}
#endif

//------------------------------------------------------------------------------
#ifndef VTK_MATH_ISFINITE_IS_INLINE
bool vtkMath::IsFinite(double x)
{
  return !vtkMath::IsNan(x) && !vtkMath::IsInf(x);
}
#endif

//------------------------------------------------------------------------------
int vtkMath::QuadraticRoot(double a, double b, double c, double min, double max, double* u)
{
  if (a == 0.0) // then its close to 0
  {
    if (b != 0.0) // not close to 0
    {
      u[0] = -c / b;
      if (u[0] > min && u[0] < max) // its in the interval
      {
        return 1; // 1 soln found
      }
      else // its not in the interval
      {
        return 0;
      }
    }
    else
    {
      return 0;
    }
  }
  double d = b * b - 4 * a * c; // discriminant
  if (d <= 0.0)                 // single or no root
  {
    if (d == 0.0) // close to 0
    {
      u[0] = -b / a;
      if (u[0] > min && u[0] < max) // its in the interval
      {
        return 1;
      }
      else // its not in the interval
      {
        return 0;
      }
    }
    else // no root d must be below 0
    {
      return 0;
    }
  }
  double q = -0.5 * (b + copysign(sqrt(d), b));
  u[0] = c / q;
  u[1] = q / a;

  if ((u[0] > min && u[0] < max) && (u[1] > min && u[1] < max))
  {
    return 2;
  }
  else if (u[0] > min && u[0] < max) // then one wasn't in interval
  {
    return 1;
  }
  else if (u[1] > min && u[1] < max)
  { // make it easier, make u[0] be the valid one always
    std::swap(u[0], u[1]);
    return 1;
  }
  return 0;
}
