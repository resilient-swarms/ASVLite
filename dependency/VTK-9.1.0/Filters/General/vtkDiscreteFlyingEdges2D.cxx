/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkDiscreteFlyingEdges2D.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkDiscreteFlyingEdges2D.h"

#include "vtkCellArray.h"
#include "vtkDataArrayRange.h"
#include "vtkFloatArray.h"
#include "vtkImageData.h"
#include "vtkImageTransform.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkSMPTools.h"
#include "vtkStreamingDemandDrivenPipeline.h"

#include <cmath>

vtkStandardNewMacro(vtkDiscreteFlyingEdges2D);

// This templated class is the heart of the algorithm. Templated across
// scalar type T. vtkDiscreteFlyingEdges2D populates the information in this class
// and then invokes ContourImage() to actually initiate executions.
template <class T>
class vtkDiscreteFlyingEdges2DAlgorithm
{
public:
  // Edge case table values. Must precisely match contour values.
  enum EdgeClass
  {
    BothOutside = 0,  // both vertices outside region
    RightOutside = 1, // right vertex is outside region, left is inside
    LeftOutside = 2,  // left vertex is outside region, right is inside
    BothInside = 3,   // both vertices inside region
  };

  // Dealing with boundary situations when processing images.
  enum CellClass
  {
    Interior = 0,
    MinBoundary = 1,
    MaxBoundary = 2
  };
  // Edges to generate output line primitives (aka case table).
  static const unsigned char EdgeCases[16][5];

  // A table that lists pixel point ids as a function of edge ids (edge ids
  // for edge-based case table).
  static const unsigned char VertMap[4][2];

  // A table describing vertex offsets (in index space) from the pixel axes
  // origin for each of the four vertices of a pixel.
  static const unsigned char VertOffsets[4][2];

  // This table is used to accelerate the generation of output lines and
  // points. The EdgeUses array, a function of the pixel case number,
  // indicates which pixel edges intersect with the contour (i.e., require
  // interpolation). This array is filled in at instantiation during the case
  // table generation process.
  unsigned char EdgeUses[16][4];

  // Flags indicate whether a particular case requires pixel axes to be
  // processed. A cheap acceleration structure computed from the case
  // tables at the point of instantiation.
  unsigned char IncludesAxes[16];

  // Algorithm-derived data
  unsigned char* XCases;
  vtkIdType* EdgeMetaData;

  // Internal variables used by the various algorithm methods. Interfaces VTK
  // image data in a form more convenient to the algorithm.
  vtkIdType Dims[2];
  int K;
  int Axis0;
  int Min0;
  int Max0;
  int Inc0;
  int Axis1;
  int Min1;
  int Max1;
  int Inc1;
  int Axis2;

  // Output data. Threads write to partitioned memory.
  T* Scalars;
  T* NewScalars;
  vtkCellArray* NewLines;
  float* NewPoints;

  // Instantiate and initialize key data members.
  vtkDiscreteFlyingEdges2DAlgorithm();

  // The three passes of the algorithm.
  void ProcessXEdge(double value, T* inPtr, vtkIdType row);   // PASS 1
  void ProcessYEdges(vtkIdType row);                          // PASS 2
  void GenerateOutput(double value, T* inPtr, vtkIdType row); // PASS 4

  // Place holder for now in case fancy bit fiddling is needed later.
  void SetXEdge(unsigned char* ePtr, unsigned char edgeCase) { *ePtr = edgeCase; }

  // Given the two x-edge cases defining this pixel, return the pixel case
  // number.
  unsigned char GetEdgeCase(unsigned char* ePtr0, unsigned char* ePtr1)
  {
    return ((*ePtr0) | ((*ePtr1) << 2));
  }

  // Return number of contouring primitives (line segments) for a particular case.
  unsigned char GetNumberOfPrimitives(unsigned char caseNum) { return this->EdgeCases[caseNum][0]; }

  // Return an array indicating which pixel edges intersect the contour.
  unsigned char* GetEdgeUses(unsigned char eCase) { return this->EdgeUses[eCase]; }

  // Indicate whether pixel axes need processing for this case.
  unsigned char CaseIncludesAxes(unsigned char eCase) { return this->IncludesAxes[eCase]; }

  // Count edge intersections near image boundaries.
  void CountBoundaryYInts(unsigned char loc, unsigned char* edgeUses, vtkIdType* eMD)
  {
    if (loc == 2)
    { //+x boundary
      eMD[1] += edgeUses[3];
    }
  }

  // Produce the line segments for this pixel cell.
  struct GenerateLinesImpl
  {
    template <typename CellStateT>
    void operator()(CellStateT& state, const unsigned char* edges, int numLines, vtkIdType* eIds,
      vtkIdType& lineId)
    {
      using ValueType = typename CellStateT::ValueType;
      auto* offsets = state.GetOffsets();
      auto* conn = state.GetConnectivity();

      auto offsetRange = vtk::DataArrayValueRange<1>(offsets);
      auto offsetIter = offsetRange.begin() + lineId;
      auto connRange = vtk::DataArrayValueRange<1>(conn);
      auto connIter = connRange.begin() + (lineId * 2);

      for (int i = 0; i < numLines; ++i)
      {
        *offsetIter++ = static_cast<ValueType>(2 * lineId++);
        *connIter++ = eIds[*edges++];
        *connIter++ = eIds[*edges++];
      }

      // Write the last offset:
      *offsetIter = static_cast<ValueType>(2 * lineId);
    }
  };
  void GenerateLines(
    unsigned char eCase, unsigned char numLines, vtkIdType* eIds, vtkIdType& lineId)
  {
    const unsigned char* edges = this->EdgeCases[eCase] + 1;
    this->NewLines->Visit(GenerateLinesImpl{}, edges, numLines, eIds, lineId);
  }

  // Interpolate along a pixel axes edge.
  void InterpolateAxesEdge(double vtkNotUsed(value), T* vtkNotUsed(s0), int ijk0[3],
    T* vtkNotUsed(s1), int ijk1[3], vtkIdType vId)
  {
    double t = 0.5;
    float* x = this->NewPoints + 3 * vId;
    x[0] = ijk0[0] + t * (ijk1[0] - ijk0[0]);
    x[1] = ijk0[1] + t * (ijk1[1] - ijk0[1]);
    x[2] = this->K;
  }

  // Interpolate along an arbitrary edge, typically one that may be on the
  // volume boundary. This means careful computation of stuff requiring
  // neighborhood information (e.g., gradients).
  void InterpolateEdge(double value, T* s, int ijk[3], unsigned char edgeNum,
    unsigned char edgeUses[4], vtkIdType* eIds);

  // Produce the output points on the pixel axes for this pixel cell.
  void GeneratePoints(
    double value, unsigned char loc, T* sPtr, int ijk[3], unsigned char* edgeUses, vtkIdType* eIds);

  // Helper function to set up the point ids on pixel edges.
  unsigned char InitPixelIds(
    unsigned char* ePtr0, unsigned char* ePtr1, vtkIdType* eMD0, vtkIdType* eMD1, vtkIdType* eIds)
  {
    unsigned char eCase = GetEdgeCase(ePtr0, ePtr1);
    eIds[0] = eMD0[0]; // x-edges
    eIds[1] = eMD1[0];
    eIds[2] = eMD0[1]; // y-edges
    eIds[3] = eIds[2] + this->EdgeUses[eCase][2];
    return eCase;
  }

  // Helper function to advance the point ids along pixel rows.
  void AdvancePixelIds(unsigned char eCase, vtkIdType* eIds)
  {
    eIds[0] += this->EdgeUses[eCase][0]; // x-edges
    eIds[1] += this->EdgeUses[eCase][1];
    eIds[2] += this->EdgeUses[eCase][2]; // y-edges
    eIds[3] = eIds[2] + this->EdgeUses[eCase][3];
  }

  // Threading integration via SMPTools
  template <class TT>
  class Pass1
  {
  public:
    Pass1(vtkDiscreteFlyingEdges2DAlgorithm<TT>* algo, double value)
    {
      this->Algo = algo;
      this->Value = value;
    }
    vtkDiscreteFlyingEdges2DAlgorithm<TT>* Algo;
    double Value;
    void operator()(vtkIdType row, vtkIdType end)
    {
      TT* rowPtr = this->Algo->Scalars + row * this->Algo->Inc1;
      for (; row < end; ++row)
      {
        this->Algo->ProcessXEdge(this->Value, rowPtr, row);
        rowPtr += this->Algo->Inc1;
      } // for all rows in this batch
    }
  };
  template <class TT>
  class Pass2
  {
  public:
    Pass2(vtkDiscreteFlyingEdges2DAlgorithm<TT>* algo) { this->Algo = algo; }
    vtkDiscreteFlyingEdges2DAlgorithm<TT>* Algo;
    void operator()(vtkIdType row, vtkIdType end)
    {
      for (; row < end; ++row)
      {
        this->Algo->ProcessYEdges(row);
      } // for all rows in this batch
    }
  };
  template <class TT>
  class Pass4
  {
  public:
    Pass4(vtkDiscreteFlyingEdges2DAlgorithm<TT>* algo, double value)
    {
      this->Algo = algo;
      this->Value = value;
    }
    vtkDiscreteFlyingEdges2DAlgorithm<TT>* Algo;
    double Value;
    void operator()(vtkIdType row, vtkIdType end)
    {
      T* rowPtr = this->Algo->Scalars + row * this->Algo->Inc1;
      for (; row < end; ++row)
      {
        this->Algo->GenerateOutput(this->Value, rowPtr, row);
        rowPtr += this->Algo->Inc1;
      } // for all rows in this batch
    }
  };

  // Interface between VTK and templated functions
  static void ContourImage(vtkDiscreteFlyingEdges2D* self, T* scalars, vtkPoints* newPts,
    vtkDataArray* newScalars, vtkCellArray* newLines, vtkImageData* input, int* updateExt);
};

//------------------------------------------------------------------------------
// Specify the points that define each edge.
template <class T>
const unsigned char vtkDiscreteFlyingEdges2DAlgorithm<T>::VertMap[4][2] = {
  { 0, 1 },
  { 2, 3 },
  { 0, 2 },
  { 1, 3 },
};

//------------------------------------------------------------------------------
// The offsets of each vertex (in index space) from the pixel axes origin.
template <class T>
const unsigned char vtkDiscreteFlyingEdges2DAlgorithm<T>::VertOffsets[4][2] = {
  { 0, 0 },
  { 1, 0 },
  { 0, 1 },
  { 1, 1 },
};

// Initialize case array
template <class T>
const unsigned char vtkDiscreteFlyingEdges2DAlgorithm<T>::EdgeCases[16][5] = {
  { 0, 0, 0, 0, 0 },
  { 1, 0, 2, 0, 0 },
  { 1, 3, 0, 0, 0 },
  { 1, 3, 2, 0, 0 },
  { 1, 2, 1, 0, 0 },
  { 1, 0, 1, 0, 0 },
  { 2, 2, 1, 3, 0 },
  { 1, 3, 1, 0, 0 },
  { 1, 1, 3, 0, 0 },
  { 2, 0, 2, 3, 1 },
  { 1, 1, 0, 0, 0 },
  { 1, 1, 2, 0, 0 },
  { 1, 2, 3, 0, 0 },
  { 1, 0, 3, 0, 0 },
  { 1, 2, 0, 0, 0 },
  { 0, 0, 0, 0, 0 },
};

//------------------------------------------------------------------------------
// Instantiate and initialize key data members. Mostly we build some
// acceleration structures from the case table.
template <class T>
vtkDiscreteFlyingEdges2DAlgorithm<T>::vtkDiscreteFlyingEdges2DAlgorithm()
  : XCases(nullptr)
  , EdgeMetaData(nullptr)
  , Scalars(nullptr)
  , NewScalars(nullptr)
  , NewLines(nullptr)
  , NewPoints(nullptr)
{
  int j, eCase, numLines;
  const unsigned char* edgeCase;

  // Initialize
  for (eCase = 0; eCase < 16; ++eCase)
  {
    for (j = 0; j < 4; ++j)
    {
      this->EdgeUses[eCase][j] = 0;
    }
    this->IncludesAxes[eCase] = 0;
  }

  // Populate
  for (eCase = 0; eCase < 16; ++eCase)
  {
    edgeCase = this->EdgeCases[eCase];
    numLines = *edgeCase++;

    // Mark edges that are used by this case.
    for (j = 0; j < numLines * 2; ++j) // just loop over all edges
    {
      this->EdgeUses[eCase][edgeCase[j]] = 1;
    }

    this->IncludesAxes[eCase] = this->EdgeUses[eCase][0] | this->EdgeUses[eCase][2];
  }
}

//------------------------------------------------------------------------------
// Interpolate a new point along a boundary edge. Make sure to consider
// proximity to boundary when computing gradients, etc.
template <class T>
void vtkDiscreteFlyingEdges2DAlgorithm<T>::InterpolateEdge(double vtkNotUsed(value),
  T* vtkNotUsed(s), int ijk[3], unsigned char edgeNum, unsigned char edgeUses[12], vtkIdType* eIds)
{
  // if this edge is not used then get out
  if (!edgeUses[edgeNum])
  {
    return;
  }

  // build the edge information
  const unsigned char* vertMap = this->VertMap[edgeNum];
  float x0[3], x1[3];
  vtkIdType vId = eIds[edgeNum];

  const unsigned char* offsets = this->VertOffsets[vertMap[0]];
  x0[0] = ijk[0] + offsets[0];
  x0[1] = ijk[1] + offsets[1];

  offsets = this->VertOffsets[vertMap[1]];
  x1[0] = ijk[0] + offsets[0];
  x1[1] = ijk[1] + offsets[1];

  // Okay interpolate
  double t = 0.5;
  float* xPtr = this->NewPoints + 3 * vId;
  xPtr[0] = x0[0] + t * (x1[0] - x0[0]) + this->Min0;
  xPtr[1] = x0[1] + t * (x1[1] - x0[1]) + this->Min1;
  xPtr[2] = this->K;
}

//------------------------------------------------------------------------------
// Generate the output points and optionally normals, gradients and
// interpolate attributes.
template <class T>
void vtkDiscreteFlyingEdges2DAlgorithm<T>::GeneratePoints(
  double value, unsigned char loc, T* sPtr, int ijk[3], unsigned char* edgeUses, vtkIdType* eIds)
{
  // Create a slightly faster path for pixel axes interior to the image.
  int ijk1[3];
  if (edgeUses[0]) // x axes edge
  {
    ijk1[0] = ijk[0] + 1;
    ijk1[1] = ijk[1];
    this->InterpolateAxesEdge(value, sPtr, ijk, sPtr + this->Inc0, ijk1, eIds[0]);
  }
  if (edgeUses[2]) // y axes edge
  {
    ijk1[0] = ijk[0];
    ijk1[1] = ijk[1] + 1;
    this->InterpolateAxesEdge(value, sPtr, ijk, sPtr + this->Inc1, ijk1, eIds[2]);
  }

  // Otherwise do more general gyrations. These are boundary situations where
  // the pixel axes is not fully formed. These situations occur on the
  // +x,+y image boundaries. (The other cases are handled by the default:
  // case and are expected.)
  switch (loc)
  {
    case 2: //+x edge
      this->InterpolateEdge(value, sPtr, ijk, 3, edgeUses, eIds);
      break;

    case 8: //+y
      this->InterpolateEdge(value, sPtr, ijk, 1, edgeUses, eIds);
      break;

    case 10: //+x +y
      this->InterpolateEdge(value, sPtr, ijk, 1, edgeUses, eIds);
      this->InterpolateEdge(value, sPtr, ijk, 3, edgeUses, eIds);
      break;

    default: // interior, or -x,-y boundary
      return;
  }
}

//------------------------------------------------------------------------------
// PASS 1: Process a single x-row (and all of the pixel edges that compose
// the row).  Start building cell contour case table, determine the number of
// intersections, figure out where intersections along row begin and end
// (computational trimming).
template <class T>
void vtkDiscreteFlyingEdges2DAlgorithm<T>::ProcessXEdge(double value, T* inPtr, vtkIdType row)
{
  vtkIdType nxcells = this->Dims[0] - 1;
  vtkIdType minInt = nxcells, maxInt = 0;
  unsigned char edgeCase;
  vtkIdType* eMD = this->EdgeMetaData + row * 5;
  unsigned char* ePtr = this->XCases + row * nxcells;
  T s0, s1 = (*inPtr);
  T labelValue = static_cast<T>(value);

  // run along the entire x-edge computing edge cases
  std::fill_n(eMD, 5, 0);
  for (vtkIdType i = 0; i < nxcells; ++i, ++ePtr)
  {
    s0 = s1;
    s1 = static_cast<T>(*(inPtr + (i + 1) * this->Inc0));

    if (s0 != labelValue)
    {
      edgeCase = (s1 != value ? vtkDiscreteFlyingEdges2DAlgorithm::BothOutside
                              : vtkDiscreteFlyingEdges2DAlgorithm::LeftOutside);
    }
    else // s0 == labelValue
    {
      edgeCase = (s1 != value ? vtkDiscreteFlyingEdges2DAlgorithm::RightOutside
                              : vtkDiscreteFlyingEdges2DAlgorithm::BothInside);
    }

    this->SetXEdge(ePtr, edgeCase);

    // if edge intersects contour
    if (edgeCase == vtkDiscreteFlyingEdges2DAlgorithm::LeftOutside ||
      edgeCase == vtkDiscreteFlyingEdges2DAlgorithm::RightOutside)
    {
      eMD[0]++; // increment number of intersections along x-edge
      minInt = (i < minInt ? i : minInt);
      maxInt = i + 1;
    } // if contour interacts with this x-edge
  }   // for all x-cell edges along this x-edge

  // The beginning and ending of intersections along the edge is used for
  // computational trimming.
  eMD[3] = minInt; // where intersections start along x edge
  eMD[4] = maxInt; // where intersections end along x edge
}

//------------------------------------------------------------------------------
// PASS 2: Process the y-cell edges (that form the cell axes) along a single
// x-row.  Continue building cell contour case table, and determine the
// number of cell y-edge intersections. Use computational trimming to reduce
// work.
template <class T>
void vtkDiscreteFlyingEdges2DAlgorithm<T>::ProcessYEdges(vtkIdType row)
{
  // Grab the two edge cases bounding this pixel x-row.
  unsigned char *ePtr0, *ePtr1, ec0, ec1, xInts = 1;
  ePtr0 = this->XCases + row * (this->Dims[0] - 1);
  ePtr1 = ePtr0 + this->Dims[0] - 1;

  // And metadata
  vtkIdType* eMD0 = this->EdgeMetaData + row * 5;
  vtkIdType* eMD1 = this->EdgeMetaData + (row + 1) * 5;

  // Determine whether this row of x-cells needs processing. If there are no
  // x-edge intersections, and there is no y-edge (any y-edge) then the
  // row is contour free.
  if ((eMD0[0] | eMD1[0]) == 0) // any x-ints?
  {
    if (*ePtr0 == *ePtr1)
    {
      return; // there are no x- or y-ints, thus no contour, skip pixel row
    }
    else
    {
      xInts = 0; // there are y- edge ints however
    }
  }

  // Determine proximity to the boundary of the image. This information is used
  // to count edge intersections in boundary situations.
  unsigned char loc, yLoc;
  yLoc = ((row >= (this->Dims[1] - 2) ? MaxBoundary : Interior) << 2);

  // The trim y-edges may need adjustment if the contour travels between
  // the top and bottom rows of x-edges (without intersecting x-edges).
  vtkIdType xL = ((eMD0[3] < eMD1[3]) ? eMD0[3] : eMD1[3]);
  vtkIdType xR = ((eMD0[4] > eMD1[4]) ? eMD0[4] : eMD1[4]);
  if (xInts)
  {
    if (xL > 0)
    {
      ec0 = *(ePtr0 + xL);
      ec1 = *(ePtr1 + xL);
      if ((ec0 & 0x1) != (ec1 & 0x1))
      {
        xL = eMD0[3] = 0; // reset left trim
      }
    }
    if (xR < (this->Dims[0] - 1))
    {
      ec0 = *(ePtr0 + xR);
      ec1 = *(ePtr1 + xR);
      if ((ec0 & 0x2) != (ec1 & 0x2))
      {
        xR = eMD0[4] = this->Dims[0] - 1; // reset right trim
      }
    }
  }
  else // contour cuts through without intersecting x-edges, reset trim edges
  {
    xL = eMD0[3] = 0;
    xR = eMD0[4] = this->Dims[0] - 1;
  }

  // Okay run along the x-pixels and count the number of
  // y-intersections. Here we are just checking y edges that make up the
  // pixel axes. Also check the number of primitives generated.
  unsigned char *edgeUses, eCase, numLines;
  ePtr0 += xL;
  ePtr1 += xL;
  vtkIdType i;
  for (i = xL; i < xR; ++i) // run along the trimmed x-pixels
  {
    eCase = this->GetEdgeCase(ePtr0, ePtr1);
    if ((numLines = this->GetNumberOfPrimitives(eCase)) > 0)
    {
      // Okay let's increment the triangle count.
      eMD0[2] += numLines;

      // Count the number of y-points to be generated. Pass# 1 counted
      // the number of x-intersections along the x-edges. Now we count all
      // intersections on the y-pixel axes.
      edgeUses = this->GetEdgeUses(eCase);
      eMD0[1] += edgeUses[2]; // y-pixel axes edge always counted
      loc = yLoc | (i >= (this->Dims[0] - 2) ? MaxBoundary : Interior);
      if (loc != 0)
      {
        this->CountBoundaryYInts(loc, edgeUses, eMD0);
      }
    } // if cell contains contour

    // advance the two pointers along pixel row
    ePtr0++;
    ePtr1++;
  } // for all pixels along this x-edge
}

//------------------------------------------------------------------------------
// PASS 4: Process the x-row cells to generate output primitives, including
// point coordinates and line segments. This is the fourth pass of the
// algorithm.
template <class T>
void vtkDiscreteFlyingEdges2DAlgorithm<T>::GenerateOutput(double value, T* rowPtr, vtkIdType row)
{
  vtkIdType* eMD0 = this->EdgeMetaData + row * 5;
  vtkIdType* eMD1 = this->EdgeMetaData + (row + 1) * 5;
  // Return if there is nothing to do (i.e., no lines to generate)
  if (eMD0[2] == eMD1[2])
  {
    return;
  }

  // Get the trim edges and prepare to generate
  vtkIdType i;
  vtkIdType xL = ((eMD0[3] < eMD1[3]) ? eMD0[3] : eMD1[3]);
  vtkIdType xR = ((eMD0[4] > eMD1[4]) ? eMD0[4] : eMD1[4]);

  // Grab the two edge cases bounding this pixel x-row. Begin at left trim edge.
  unsigned char *ePtr0, *ePtr1;
  ePtr0 = this->XCases + row * (this->Dims[0] - 1) + xL;
  ePtr1 = ePtr0 + this->Dims[0] - 1;

  // Traverse all pixels in this row, those containing the contour are
  // further identified for processing, meaning generating points and
  // triangles. Begin by setting up point ids on pixel edges.
  vtkIdType lineId = eMD0[2];
  vtkIdType eIds[4]; // the ids of generated points
  unsigned char *edgeUses, numLines;
  unsigned char eCase = this->InitPixelIds(ePtr0, ePtr1, eMD0, eMD1, eIds);

  // Determine the proximity to the boundary of volume. This information is
  // used to generate edge intersections.
  unsigned char loc, yLoc;
  yLoc = ((row >= (this->Dims[1] - 2) ? MaxBoundary : Interior) << 2);

  // Run along pixels in x-row direction and generate output primitives. Note
  // that active pixel axes edges are interpolated to produce points and
  // possibly interpolate attribute data.
  T* sPtr;
  int ijk[3];
  ijk[1] = row;
  ijk[2] = this->K;
  for (i = xL; i < xR; ++i)
  {
    if ((numLines = this->GetNumberOfPrimitives(eCase)) > 0)
    {
      // Start by generating triangles for this case
      this->GenerateLines(eCase, numLines, eIds, lineId);

      // Now generate point(s) along pixel axes if needed. Remember to take
      // boundary into account.
      loc = yLoc | (i >= (this->Dims[0] - 2) ? MaxBoundary : Interior);
      if (this->CaseIncludesAxes(eCase) || loc != Interior)
      {
        sPtr = rowPtr + i * this->Inc0;
        ijk[0] = i;
        edgeUses = this->GetEdgeUses(eCase);
        this->GeneratePoints(value, loc, sPtr, ijk, edgeUses, eIds);
      }

      this->AdvancePixelIds(eCase, eIds);
    }

    // advance along pixel row
    ePtr0++;
    ePtr1++;
    eCase = GetEdgeCase(ePtr0, ePtr1);
  } // for all non-trimmed cells along this x-edge
}

//------------------------------------------------------------------------------
// Contouring filter specialized for images. This templated function interfaces the
// vtkDiscreteFlyingEdges2D class with the templated algorithm class. It also invokes
// the three passes of the Flying Edges algorithm.
//
template <class T>
void vtkDiscreteFlyingEdges2DAlgorithm<T>::ContourImage(vtkDiscreteFlyingEdges2D* self, T* scalars,
  vtkPoints* newPts, vtkDataArray* newScalars, vtkCellArray* newLines, vtkImageData* input,
  int* updateExt)
{
  double value, *values = self->GetValues();
  vtkIdType numContours = self->GetNumberOfContours();
  vtkIdType vidx, row, *eMD;
  vtkIdType numOutXPts, numOutYPts, numOutLines, numXPts = 0, numYPts = 0, numLines = 0;
  vtkIdType startXPts, startYPts, startLines;
  startXPts = startYPts = startLines = 0;

  // The update extent may be different than the extent of the image.
  // The only problem with using the update extent is that one or two
  // sources enlarge the update extent.  This behavior is slated to be
  // eliminated.
  vtkIdType incs[3];
  input->GetIncrements(incs);
  int* ext = input->GetExtent();

  // Figure out which 2D plane the image lies in. Capture information for
  // subsequent processing.
  vtkDiscreteFlyingEdges2DAlgorithm<T> algo;
  if (updateExt[4] == updateExt[5])
  { // z collapsed
    algo.Axis0 = 0;
    algo.Min0 = updateExt[0];
    algo.Max0 = updateExt[1];
    algo.Inc0 = incs[0];
    algo.Axis1 = 1;
    algo.Min1 = updateExt[2];
    algo.Max1 = updateExt[3];
    algo.Inc1 = incs[1];
    algo.K = updateExt[4];
    algo.Axis2 = 2;
  }
  else if (updateExt[2] == updateExt[3])
  { // y collapsed
    algo.Axis0 = 0;
    algo.Min0 = updateExt[0];
    algo.Max0 = updateExt[1];
    algo.Inc0 = incs[0];
    algo.Axis1 = 2;
    algo.Min1 = updateExt[4];
    algo.Max1 = updateExt[5];
    algo.Inc1 = incs[2];
    algo.K = updateExt[2];
    algo.Axis2 = 1;
  }
  else if (updateExt[0] == updateExt[1])
  { // x collapsed
    algo.Axis0 = 1;
    algo.Min0 = updateExt[2];
    algo.Max0 = updateExt[3];
    algo.Inc0 = incs[1];
    algo.Axis1 = 2;
    algo.Min1 = updateExt[4];
    algo.Max1 = updateExt[5];
    algo.Inc1 = incs[2];
    algo.K = updateExt[0];
    algo.Axis2 = 0;
  }
  else
  {
    vtkGenericWarningMacro("Expecting 2D data.");
    return;
  }

  // Now allocate working arrays. The XCases array tracks case# for each cell.
  algo.Dims[0] = algo.Max0 - algo.Min0 + 1;
  algo.Dims[1] = algo.Max1 - algo.Min1 + 1;
  algo.XCases = new unsigned char[(algo.Dims[0] - 1) * algo.Dims[1]];

  // Also allocate the characterization (metadata) array for the x edges.
  // This array tracks the number of intersections along each x-row, y-row;
  // as well as num line primitives, and the xMin_i and xMax_i (minimum
  // index of first intersection, maximum index of intersection for row i,
  // so-called trim edges used for computational trimming).
  algo.EdgeMetaData = new vtkIdType[algo.Dims[1] * 5];

  // Compute the starting location for scalar data.  We may be operating
  // on a part of the image.
  algo.Scalars = scalars + incs[0] * (updateExt[0] - ext[0]) + incs[1] * (updateExt[2] - ext[2]) +
    incs[2] * (updateExt[4] - ext[4]) + self->GetArrayComponent();

  // The algorithm is separated into multiple passes. The first pass
  // computes intersections on row edges, counting the number of intersected edges
  // as it progresses. It also keeps track of the generated edge cases and
  // other incidental information about intersections along rows. The second
  // pass generates polylines from the cases and intersection information.
  // In the final and third pass output points and lines are generated.

  // Loop across each contour value. This encompasses all three passes.
  for (vidx = 0; vidx < numContours; vidx++)
  {
    value = values[vidx];

    // PASS 1: Traverse all rows generating intersection points and building
    // the case table. Also accumulate information necessary for later allocation.
    // For example the number of output points is computed.
    Pass1<T> pass1(&algo, value);
    vtkSMPTools::For(0, algo.Dims[1], pass1);

    // PASS 2: Traverse all rows and process cell y edges. Continue building
    // case table from y contributions (using computational trimming to reduce
    // work) and keep track of cell y intersections.
    Pass2<T> pass2(&algo);
    vtkSMPTools::For(0, algo.Dims[1] - 1, pass2);

    // PASS 3: Now allocate and generate output. First we have to update the
    // x-Edge meta data to partition the output into separate pieces so
    // independent threads can write into separate memory partititions. Once
    // allocation is complete, process on a row by row basis and produce
    // output points, line primitives, and interpolate point attribute data
    // (if necessary).
    numOutXPts = startXPts;
    numOutYPts = startYPts;
    numOutLines = startLines;
    for (row = 0; row < algo.Dims[1]; ++row)
    {
      eMD = algo.EdgeMetaData + row * 5;
      numXPts = eMD[0];
      numYPts = eMD[1];
      numLines = eMD[2];
      eMD[0] = numOutXPts + numOutYPts;
      eMD[1] = eMD[0] + numXPts;
      eMD[2] = numOutLines;
      numOutXPts += numXPts;
      numOutYPts += numYPts;
      numOutLines += numLines;
    }

    // Output can now be allocated.
    vtkIdType totalPts = numOutXPts + numOutYPts;
    if (totalPts > 0)
    {
      newPts->GetData()->WriteVoidPointer(0, 3 * totalPts);
      algo.NewPoints = static_cast<float*>(newPts->GetVoidPointer(0));
      newLines->ResizeExact(numOutLines, 2 * numOutLines);
      algo.NewLines = newLines;
      if (newScalars)
      {
        newScalars->WriteVoidPointer(0, numOutXPts + numOutYPts);
        algo.NewScalars = static_cast<T*>(newScalars->GetVoidPointer(0));
        T TValue = static_cast<T>(value);
        std::fill_n(algo.NewScalars, totalPts, TValue);
      }

      // PASS 4: Now process each x-row and produce the output primitives.
      Pass4<T> pass4(&algo, value);
      vtkSMPTools::For(0, algo.Dims[1] - 1, pass4);
    } // if output generated

    // Handle multiple contours
    startXPts = numOutXPts;
    startYPts = numOutYPts;
    startLines = numOutLines;
  } // for all contour values

  // Clean up and return
  delete[] algo.XCases;
  delete[] algo.EdgeMetaData;
}

//------------------------------------------------------------------------------
// Here is the VTK class proper.
// Construct object with initial contour value of 0.0.
vtkDiscreteFlyingEdges2D::vtkDiscreteFlyingEdges2D()
{
  this->ContourValues = vtkContourValues::New();

  this->ComputeScalars = 1;
  this->ArrayComponent = 0;

  // by default process active point scalars
  this->SetInputArrayToProcess(
    0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, vtkDataSetAttributes::SCALARS);
}

//------------------------------------------------------------------------------
vtkDiscreteFlyingEdges2D::~vtkDiscreteFlyingEdges2D()
{
  this->ContourValues->Delete();
}

//------------------------------------------------------------------------------
// Description:
// Overload standard modified time function. If contour values are modified,
// then this object is modified as well.
vtkMTimeType vtkDiscreteFlyingEdges2D::GetMTime()
{
  vtkMTimeType mTime = this->Superclass::GetMTime();
  vtkMTimeType mTime2 = this->ContourValues->GetMTime();

  return (mTime2 > mTime ? mTime2 : mTime);
}

//------------------------------------------------------------------------------
// Contouring filter specialized for images (or slices from images)
//
int vtkDiscreteFlyingEdges2D::RequestData(vtkInformation* vtkNotUsed(request),
  vtkInformationVector** inputVector, vtkInformationVector* outputVector)
{
  // get the info objects
  vtkInformation* inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation* outInfo = outputVector->GetInformationObject(0);

  // get the input and output
  vtkImageData* input = vtkImageData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkPolyData* output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  vtkDataArray* inScalars;

  vtkDebugMacro(<< "Executing 2D Flying Edges");

  int* ext = inInfo->Get(vtkStreamingDemandDrivenPipeline::UPDATE_EXTENT());
  inScalars = this->GetInputArrayToProcess(0, inputVector);
  if (inScalars == nullptr)
  {
    vtkErrorMacro(<< "Scalars must be defined for contouring");
    return 1;
  }

  int numComps = inScalars->GetNumberOfComponents();
  if (this->ArrayComponent >= numComps)
  {
    vtkErrorMacro("Scalars have " << numComps
                                  << " components. "
                                     "ArrayComponent must be smaller than "
                                  << numComps);
    return 1;
  }

  // Create necessary objects to hold output. We will defer the
  // actual allocation to a later point.
  vtkCellArray* newLines = vtkCellArray::New();
  vtkPoints* newPts = vtkPoints::New();
  newPts->SetDataTypeToFloat();
  vtkDataArray* newScalars = nullptr;

  if (this->ComputeScalars)
  {
    newScalars = inScalars->NewInstance();
    newScalars->SetNumberOfComponents(1);
    newScalars->SetName(inScalars->GetName());
  }

  // Check data type and execute appropriate function
  void* scalars = inScalars->GetVoidPointer(0);
  switch (inScalars->GetDataType())
  {
    vtkTemplateMacro(vtkDiscreteFlyingEdges2DAlgorithm<VTK_TT>::ContourImage(
      this, (VTK_TT*)scalars, newPts, newScalars, newLines, input, ext));
  } // switch

  vtkDebugMacro(<< "Created: " << newPts->GetNumberOfPoints() << " points, "
                << newLines->GetNumberOfCells() << " lines");

  // Update ourselves.  Because we don't know up front how many lines
  // we've created, take care to reclaim memory.
  output->SetPoints(newPts);
  newPts->Delete();

  output->SetLines(newLines);
  newLines->Delete();

  if (newScalars)
  {
    int idx = output->GetPointData()->AddArray(newScalars);
    output->GetPointData()->SetActiveAttribute(idx, vtkDataSetAttributes::SCALARS);
    newScalars->Delete();
  }

  vtkImageTransform::TransformPointSet(input, output);

  return 1;
}

//------------------------------------------------------------------------------
int vtkDiscreteFlyingEdges2D::FillInputPortInformation(int, vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkImageData");
  return 1;
}

//------------------------------------------------------------------------------
void vtkDiscreteFlyingEdges2D::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  this->ContourValues->PrintSelf(os, indent.GetNextIndent());

  os << indent << "Compute Scalars: " << (this->ComputeScalars ? "On\n" : "Off\n");
  os << indent << "ArrayComponent: " << this->ArrayComponent << endl;
}
