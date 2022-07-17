//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//
//  Copyright 2018 National Technology & Engineering Solutions of Sandia, LLC (NTESS).
//  Copyright 2018 UT-Battelle, LLC.
//  Copyright 2018 Los Alamos National Security.
//
//  Under the terms of Contract DE-NA0003525 with NTESS,
//  the U.S. Government retains certain rights in this software.
//
//  Under the terms of Contract DE-AC52-06NA25396 with Los Alamos National
//  Laboratory (LANL), the U.S. Government retains certain rights in
//  this software.
//============================================================================
#ifndef vtk_m_worklet_cellmetrics_TypeOfCellHexahedral
#define vtk_m_worklet_cellmetrics_TypeOfCellHexahedral
/**
 * The Verdict manual defines a set of commonly
 * used components of a hexahedra (hex). For example,
 * area, edge lengths, and so forth.
 *
 * These definitions can be found starting on
 * page 77 of the Verdict manual.
 *
 * This file contains a set of functions which
 * implement return the values of those commonly
 * used components for subsequent use in metrics.
 */

/**
 * Returns the L0 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL0(const CollectionOfPoints& pts)
{
  const Vector L0(pts[1] - pts[0]);
  return L0;
}

/**
 * Returns the L1 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL1(const CollectionOfPoints& pts)
{
  const Vector L1(pts[2] - pts[1]);
  return L1;
}

/**
 * Returns the L2 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL2(const CollectionOfPoints& pts)
{
  const Vector L2(pts[3] - pts[2]);
  return L2;
}

/**
 * Returns the L3 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL3(const CollectionOfPoints& pts)
{
  const Vector L3(pts[3] - pts[0]);
  return L3;
}

/**
 * Returns the L4 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL4(const CollectionOfPoints& pts)
{
  const Vector L4(pts[4] - pts[0]);
  return L4;
}

/**
 * Returns the L5 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL5(const CollectionOfPoints& pts)
{
  const Vector L5(pts[5] - pts[1]);
  return L5;
}

/**
 * Returns the L6 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL6(const CollectionOfPoints& pts)
{
  const Vector L6(pts[6] - pts[2]);
  return L6;
}

/**
 * Returns the L7 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL7(const CollectionOfPoints& pts)
{
  const Vector L7(pts[7] - pts[3]);
  return L7;
}

/**
 * Returns the L8 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL8(const CollectionOfPoints& pts)
{
  const Vector L8(pts[5] - pts[4]);
  return L8;
}

/**
 * Returns the L9 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL9(const CollectionOfPoints& pts)
{
  const Vector L9(pts[6] - pts[5]);
  return L9;
}

/**
 * Returns the L10 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL10(const CollectionOfPoints& pts)
{
  const Vector L10(pts[7] - pts[6]);
  return L10;
}
/**
 * Returns the L11 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexL11(const CollectionOfPoints& pts)
{
  const Vector L11(pts[7] - pts[4]);
  return L11;
}
/**
 * Returns the L0 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL0Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l0 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL0<Scalar, Vector, CollectionOfPoints>(pts)));
  return l0;
}
/**
 * Returns the L1 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL1Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l1 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL1<Scalar, Vector, CollectionOfPoints>(pts)));
  return l1;
}
/**
 * Returns the L2 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL2Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l2 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL2<Scalar, Vector, CollectionOfPoints>(pts)));
  return l2;
}
/**
 * Returns the L3 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL3Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l3 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL3<Scalar, Vector, CollectionOfPoints>(pts)));
  return l3;
}
/**
 * Returns the L4 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL4Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l4 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL4<Scalar, Vector, CollectionOfPoints>(pts)));
  return l4;
}
/**
 * Returns the L5 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL5Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l5 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL5<Scalar, Vector, CollectionOfPoints>(pts)));
  return l5;
}
/**
 * Returns the L6 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL6Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l6 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL6<Scalar, Vector, CollectionOfPoints>(pts)));
  return l6;
}
/**
 * Returns the L7 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL7Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l7 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL7<Scalar, Vector, CollectionOfPoints>(pts)));
  return l7;
}
/**
 * Returns the L8 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL8Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l8 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL8<Scalar, Vector, CollectionOfPoints>(pts)));
  return l8;
}
/**
 * Returns the L9 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL9Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l9 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL9<Scalar, Vector, CollectionOfPoints>(pts)));
  return l9;
}
/**
 * Returns the L10 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL10Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l10 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL10<Scalar, Vector, CollectionOfPoints>(pts)));
  return l10;
}
/**
 * Returns the L11 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexL11Magnitude(const CollectionOfPoints& pts)
{
  const Scalar l11 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexL11<Scalar, Vector, CollectionOfPoints>(pts)));
  return l11;
}
/**
 * Returns the Max of the magnitude of each vector which makes up the sides of the Hex.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexLMax(const CollectionOfPoints& pts)
{
  const Scalar l0 = GetHexL0Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l1 = GetHexL1Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l2 = GetHexL2Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l3 = GetHexL3Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l4 = GetHexL4Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l5 = GetHexL5Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l6 = GetHexL6Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l7 = GetHexL7Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l8 = GetHexL8Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l9 = GetHexL9Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l10 = GetHexL10Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l11 = GetHexL11Magnitude<Scalar, Vector, CollectionOfPoints>(pts);

  const Scalar lmax = vtkm::Max(
    l0,
    vtkm::Max(
      l1,
      vtkm::Max(
        l2,
        vtkm::Max(
          l3,
          vtkm::Max(
            l4,
            vtkm::Max(
              l5,
              vtkm::Max(l6, vtkm::Max(l7, vtkm::Max(l8, vtkm::Max(l9, vtkm::Max(l10, l11)))))))))));

  return lmax;
}

/**
 * Returns the Min of the magnitude of each vector which makes up the sides of the Hex.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexLMin(const CollectionOfPoints& pts)
{
  const Scalar l0 = GetHexL0Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l1 = GetHexL1Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l2 = GetHexL2Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l3 = GetHexL3Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l4 = GetHexL4Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l5 = GetHexL5Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l6 = GetHexL6Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l7 = GetHexL7Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l8 = GetHexL8Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l9 = GetHexL9Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l10 = GetHexL10Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar l11 = GetHexL11Magnitude<Scalar, Vector, CollectionOfPoints>(pts);

  const Scalar lmin = vtkm::Min(
    l0,
    vtkm::Min(
      l1,
      vtkm::Min(
        l2,
        vtkm::Min(
          l3,
          vtkm::Min(
            l4,
            vtkm::Min(
              l5,
              vtkm::Min(l6, vtkm::Min(l7, vtkm::Min(l8, vtkm::Min(l9, vtkm::Min(l10, l11)))))))))));

  return lmin;
}

/**
 * Returns the D0 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexD0(const CollectionOfPoints& pts)
{
  const Vector D0((pts[6] - pts[0]));
  return D0;
}

/**
 * Returns the D1 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexD1(const CollectionOfPoints& pts)
{
  const Vector D1(pts[7] - pts[1]);
  return D1;
}

/**
 * Returns the D2 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexD2(const CollectionOfPoints& pts)
{
  const Vector D2(pts[4] - pts[2]);
  return D2;
}

/**
 * Returns the D3 vector, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the hexahedra.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexD3(const CollectionOfPoints& pts)
{
  const Vector D3(pts[5] - pts[3]);
  return D3;
}

/**
 * Returns the D0 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexD0Magnitude(const CollectionOfPoints& pts)
{
  const Scalar d0 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexD0<Scalar, Vector, CollectionOfPoints>(pts)));
  return d0;
}

/**
 * Returns the D1 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexD1Magnitude(const CollectionOfPoints& pts)
{
  const Scalar d1 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexD1<Scalar, Vector, CollectionOfPoints>(pts)));
  return d1;
}

/**
 * Returns the D2 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexD2Magnitude(const CollectionOfPoints& pts)
{
  const Scalar d2 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexD2<Scalar, Vector, CollectionOfPoints>(pts)));
  return d2;
}

/**
 * Returns the D3 vector's magnitude, as defined by the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexD3Magnitude(const CollectionOfPoints& pts)
{
  const Scalar d3 =
    vtkm::Sqrt(vtkm::MagnitudeSquared(GetHexD3<Scalar, Vector, CollectionOfPoints>(pts)));
  return d3;
}

/**
 * Returns the Min of the magnitude of each vector which makes up the sides of the Hex.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexDMin(const CollectionOfPoints& pts)
{
  const Scalar d0 = GetHexD0Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar d1 = GetHexD1Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar d2 = GetHexD2Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar d3 = GetHexD3Magnitude<Scalar, Vector, CollectionOfPoints>(pts);

  const Scalar dmin = vtkm::Min(d0, vtkm::Min(d1, vtkm::Min(d2, d3)));

  return dmin;
}

/**
 * Returns the Min of the magnitude of each vector which makes up the sides of the Hex.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the magnitude of the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexDMax(const CollectionOfPoints& pts)
{
  const Scalar d0 = GetHexD0Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar d1 = GetHexD1Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar d2 = GetHexD2Magnitude<Scalar, Vector, CollectionOfPoints>(pts);
  const Scalar d3 = GetHexD3Magnitude<Scalar, Vector, CollectionOfPoints>(pts);

  const Scalar dmax = vtkm::Max(d0, vtkm::Max(d1, vtkm::Max(d2, d3)));

  return dmax;
}

/**
 * Returns the X1 vector defined in the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexX1(const CollectionOfPoints& pts)
{
  const Vector X1((pts[1] - pts[0]) + (pts[2] - pts[3]) + (pts[5] - pts[4]) + (pts[6] - pts[7]));
  return X1;
}

/**
 * Returns the X2 vector defined in the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexX2(const CollectionOfPoints& pts)
{
  const Vector X2((pts[3] - pts[0]) + (pts[2] - pts[1]) + (pts[7] - pts[4]) + (pts[6] - pts[5]));
  return X2;
}

/**
 * Returns the X3 vector defined in the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Vector GetHexX3(const CollectionOfPoints& pts)
{
  const Vector X3((pts[4] - pts[0]) + (pts[5] - pts[1]) + (pts[6] - pts[2]) + (pts[7] - pts[3]));
  return X3;
}

/**
 * Returns the A_i matrix defined in the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \param [in] index The index of A to load.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC vtkm::Vec<Vector, 3> GetHexAi(const CollectionOfPoints& pts, const vtkm::Id& index)
{
  const Scalar neg1(-1.0);
  if (index == 0)
  {
    const Vector v0 = GetHexL0<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v1 = GetHexL3<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v2 = GetHexL4<Scalar, Vector, CollectionOfPoints>(pts);
    const vtkm::Vec<Vector, 3> A = { v0, v1, v2 };
    return A;
  }
  else if (index == 1)
  {
    const Vector v0 = GetHexL1<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v1 = neg1 * GetHexL0<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v2 = GetHexL5<Scalar, Vector, CollectionOfPoints>(pts);
    const vtkm::Vec<Vector, 3> A = { v0, v1, v2 };
    return A;
  }
  else if (index == 2)
  {
    const Vector v0 = GetHexL2<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v1 = neg1 * GetHexL1<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v2 = GetHexL6<Scalar, Vector, CollectionOfPoints>(pts);
    const vtkm::Vec<Vector, 3> A = { v0, v1, v2 };
    return A;
  }
  else if (index == 3)
  {
    const Vector v0 = neg1 * GetHexL3<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v1 = neg1 * GetHexL2<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v2 = GetHexL7<Scalar, Vector, CollectionOfPoints>(pts);
    const vtkm::Vec<Vector, 3> A = { v0, v1, v2 };
    return A;
  }
  else if (index == 4)
  {
    const Vector v0 = GetHexL11<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v1 = GetHexL8<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v2 = neg1 * GetHexL4<Scalar, Vector, CollectionOfPoints>(pts);
    const vtkm::Vec<Vector, 3> A = { v0, v1, v2 };
    return A;
  }
  else if (index == 5)
  {
    const Vector v0 = neg1 * GetHexL8<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v1 = GetHexL9<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v2 = neg1 * GetHexL5<Scalar, Vector, CollectionOfPoints>(pts);
    const vtkm::Vec<Vector, 3> A = { v0, v1, v2 };
    return A;
  }
  else if (index == 6)
  {
    const Vector v0 = neg1 * GetHexL9<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v1 = GetHexL10<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v2 = neg1 * GetHexL6<Scalar, Vector, CollectionOfPoints>(pts);
    const vtkm::Vec<Vector, 3> A = { v0, v1, v2 };
    return A;
  }
  else if (index == 7)
  {
    const Vector v0 = neg1 * GetHexL10<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v1 = neg1 * GetHexL11<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v2 = neg1 * GetHexL7<Scalar, Vector, CollectionOfPoints>(pts);
    const vtkm::Vec<Vector, 3> A = { v0, v1, v2 };
    return A;
  }
  else
  {
    const Vector v0 = GetHexX1<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v1 = GetHexX2<Scalar, Vector, CollectionOfPoints>(pts);
    const Vector v2 = GetHexX3<Scalar, Vector, CollectionOfPoints>(pts);
    const vtkm::Vec<Vector, 3> A = { v0, v1, v2 };
    return A;
  }
}

/**
 * Returns ||A_i||^2 as defined in the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \param [in] index The index of A to load.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexAiNormSquared(const CollectionOfPoints& pts, const vtkm::Id& index)
{
  const vtkm::Vec<Vector, 3> Ai = GetHexAi<Scalar, Vector, CollectionOfPoints>(pts, index);
  const Scalar magSquared0 = vtkm::MagnitudeSquared(Ai[0]);
  const Scalar magSquared1 = vtkm::MagnitudeSquared(Ai[1]);
  const Scalar magSquared2 = vtkm::MagnitudeSquared(Ai[2]);

  const Scalar AiNormSquared = magSquared0 + magSquared1 + magSquared2;

  return AiNormSquared;
}

/**
 * Returns ||adj(A_i)||^2 as defined in the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \param [in] index The index of A to load.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexAiAdjNormSquared(const CollectionOfPoints& pts, const vtkm::Id& index)
{
  const vtkm::Vec<Vector, 3> Ai = GetHexAi<Scalar, Vector, CollectionOfPoints>(pts, index);
  const Scalar magSquared0 = vtkm::MagnitudeSquared(vtkm::Cross(Ai[0], Ai[1]));
  const Scalar magSquared1 = vtkm::MagnitudeSquared(vtkm::Cross(Ai[1], Ai[2]));
  const Scalar magSquared2 = vtkm::MagnitudeSquared(vtkm::Cross(Ai[2], Ai[0]));

  const Scalar AiAdjNormSquared = magSquared0 + magSquared1 + magSquared2;

  return AiAdjNormSquared;
}

/**
 * Returns alpha_i, the determinant of A_i, as defined in the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \param [in] index The index of A to load.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexAlphai(const CollectionOfPoints& pts, const vtkm::Id& index)
{
  const vtkm::Vec<Vector, 3> Ai = GetHexAi<Scalar, Vector, CollectionOfPoints>(pts, index);
  const Scalar alpha_i = vtkm::Dot(Ai[0], vtkm::Cross(Ai[1], Ai[2]));

  return alpha_i;
}

/**
 * Returns hat{A}_i, the "normalized" version of A_i, as defined in the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \param [in] index The index of A to load.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC vtkm::Vec<Vector, 3> GetHexAiHat(const CollectionOfPoints& pts, const vtkm::Id& index)
{
  const vtkm::Vec<Vector, 3> Ai = GetHexAi<Scalar, Vector, CollectionOfPoints>(pts, index);
  const Vector v0hat = Ai[0] / vtkm::Sqrt(vtkm::MagnitudeSquared(Ai[0]));
  const Vector v1hat = Ai[1] / vtkm::Sqrt(vtkm::MagnitudeSquared(Ai[1]));
  const Vector v2hat = Ai[2] / vtkm::Sqrt(vtkm::MagnitudeSquared(Ai[2]));

  const vtkm::Vec<Vector, 3> Ahat = { v0hat, v1hat, v2hat };
  return Ahat;
}
/**
 * Returns hat{alpha}_i, the determinant of hat{A}_i, as defined in the verdict manual.
 *
 *  \param [in] pts The eight points which define the Hex.
 *  \param [in] index The index of A to load.
 *  \return Returns the vector.
 */
template <typename Scalar, typename Vector, typename CollectionOfPoints>
VTKM_EXEC Scalar GetHexAlphaiHat(const CollectionOfPoints& pts, const vtkm::Id& index)
{
  const vtkm::Vec<Vector, 3> Ai = GetHexAiHat<Scalar, Vector, CollectionOfPoints>(pts, index);
  const Scalar hatAlpha_i = vtkm::Dot(Ai[0], vtkm::Cross(Ai[1], Ai[2]));

  return hatAlpha_i;
}
#endif
