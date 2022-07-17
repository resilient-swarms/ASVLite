/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkViewport.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkViewport.h"

#include "vtkActor2DCollection.h"
#include "vtkAssemblyPath.h"
#include "vtkMath.h"
#include "vtkProp.h"
#include "vtkPropCollection.h"
#include "vtkWindow.h"

#include <algorithm>

//------------------------------------------------------------------------------
// Create a vtkViewport with a black background, a white ambient light,
// two-sided lighting turned on, a viewport of (0,0,1,1), and backface culling
// turned off.
vtkViewport::vtkViewport()
{
  this->VTKWindow = nullptr;

  this->Background[0] = 0;
  this->Background[1] = 0;
  this->Background[2] = 0;

  this->Background2[0] = 0.2;
  this->Background2[1] = 0.2;
  this->Background2[2] = 0.2;

  this->BackgroundAlpha = 0.0;

  this->GradientBackground = false;

  this->EnvironmentalBG[0] = 0;
  this->EnvironmentalBG[1] = 0;
  this->EnvironmentalBG[2] = 0;

  this->EnvironmentalBG2[0] = 0.2;
  this->EnvironmentalBG2[1] = 0.2;
  this->EnvironmentalBG2[2] = 0.2;

  this->GradientEnvironmentalBG = false;

  this->Viewport[0] = 0;
  this->Viewport[1] = 0;
  this->Viewport[2] = 1;
  this->Viewport[3] = 1;

  this->WorldPoint[0] = 0;
  this->WorldPoint[1] = 0;
  this->WorldPoint[2] = 0;
  this->WorldPoint[3] = 0;

  this->DisplayPoint[0] = 0;
  this->DisplayPoint[1] = 0;
  this->DisplayPoint[2] = 0;

  this->ViewPoint[0] = 0;
  this->ViewPoint[1] = 0;
  this->ViewPoint[2] = 0;

  this->Aspect[0] = this->Aspect[1] = 1.0;
  this->PixelAspect[0] = this->PixelAspect[1] = 1.0;
  this->Center[0] = 0.0;
  this->Center[1] = 0.0;

  this->Size[0] = 0;
  this->Size[1] = 0;

  this->Origin[0] = 0;
  this->Origin[1] = 0;

  this->PickedProp = nullptr;
  this->PickFromProps = nullptr;
  this->PickResultProps = nullptr;
  this->PickX1 = -1;
  this->PickY1 = -1;
  this->PickX2 = -1;
  this->PickY2 = -1;
  this->PickedZ = 1.0;

  this->Props = vtkPropCollection::New();
  this->Actors2D = vtkActor2DCollection::New();
}

//------------------------------------------------------------------------------
vtkViewport::~vtkViewport()
{
  this->Actors2D->Delete();
  this->Actors2D = nullptr;

  this->RemoveAllViewProps();
  this->Props->Delete();
  this->Props = nullptr;

  if (this->VTKWindow != nullptr)
  {
    // renderer never reference counted the window.
    // loop is too hard to detect.
    // this->VTKWindow->UnRegister(this);
    this->VTKWindow = nullptr;
  }

  if (this->PickedProp != nullptr)
  {
    this->PickedProp->UnRegister(this);
  }
  if (this->PickResultProps != nullptr)
  {
    this->PickResultProps->Delete();
  }
}

//------------------------------------------------------------------------------
void vtkViewport::AddActor2D(vtkProp* p)
{
  this->AddViewProp(p);
}

//------------------------------------------------------------------------------
void vtkViewport::RemoveActor2D(vtkProp* p)
{
  this->Actors2D->RemoveItem(p);
  this->RemoveViewProp(p);
}

//------------------------------------------------------------------------------
int vtkViewport::HasViewProp(vtkProp* p)
{
  return (p && this->Props->IsItemPresent(p));
}

//------------------------------------------------------------------------------
void vtkViewport::AddViewProp(vtkProp* p)
{
  if (p && !this->HasViewProp(p))
  {
    this->Props->AddItem(p);
    p->AddConsumer(this);
  }
}

//------------------------------------------------------------------------------
void vtkViewport::RemoveViewProp(vtkProp* p)
{
  if (p && this->HasViewProp(p))
  {
    if (this->VTKWindow)
    {
      this->VTKWindow->MakeCurrent();
    }
    p->ReleaseGraphicsResources(this->VTKWindow);
    p->RemoveConsumer(this);
    this->Props->RemoveItem(p);
  }
}

//------------------------------------------------------------------------------
void vtkViewport::RemoveAllViewProps()
{
  vtkProp* aProp;
  vtkCollectionSimpleIterator pit;
  for (this->Props->InitTraversal(pit); (aProp = this->Props->GetNextProp(pit));)
  {
    aProp->ReleaseGraphicsResources(this->VTKWindow);
    aProp->RemoveConsumer(this);
  }
  this->Props->RemoveAllItems();
}

//------------------------------------------------------------------------------
// look through the props and get all the actors
vtkActor2DCollection* vtkViewport::GetActors2D()
{
  vtkProp* aProp;

  // clear the collection first
  this->Actors2D->RemoveAllItems();

  vtkCollectionSimpleIterator pit;
  for (this->Props->InitTraversal(pit); (aProp = this->Props->GetNextProp(pit));)
  {
    aProp->GetActors2D(this->Actors2D);
  }
  return this->Actors2D;
}

// Convert display coordinates to view coordinates.
void vtkViewport::DisplayToView()
{
  if (this->VTKWindow)
  {
    double vx, vy, vz;
    int sizex, sizey;

    /* get physical window dimensions */
    const int* size = this->VTKWindow->GetSize();
    if (size == nullptr)
    {
      return;
    }
    sizex = size[0];
    sizey = size[1];

    vx = 0.0;
    if (sizex != 0.0)
    {
      vx = 2.0 * (this->DisplayPoint[0] - sizex * this->Viewport[0]) /
          (sizex * (this->Viewport[2] - this->Viewport[0])) -
        1.0;
    }

    vy = 0.0;
    if (sizey != 0.0)
    {
      vy = 2.0 * (this->DisplayPoint[1] - sizey * this->Viewport[1]) /
          (sizey * (this->Viewport[3] - this->Viewport[1])) -
        1.0;
    }

    vz = this->DisplayPoint[2];

    this->SetViewPoint(vx, vy, vz);
  }
}

//------------------------------------------------------------------------------
// Convert view coordinates to display coordinates.
void vtkViewport::ViewToDisplay()
{
  if (this->VTKWindow)
  {
    double x = this->ViewPoint[0];
    double y = this->ViewPoint[1];
    double z = this->ViewPoint[2];
    this->ViewToDisplay(x, y, z);
    this->SetDisplayPoint(x, y, z);
  }
}

void vtkViewport::ViewToDisplay(double& x, double& y, double& vtkNotUsed(z))
{
  if (this->VTKWindow)
  {
    double dx, dy;
    int sizex, sizey;

    /* get physical window dimensions */
    const int* size = this->VTKWindow->GetSize();
    if (!size)
    {
      return;
    }
    sizex = size[0];
    sizey = size[1];

    dx = (x + 1.0) * (sizex * (this->Viewport[2] - this->Viewport[0])) / 2.0 +
      sizex * this->Viewport[0];
    dy = (y + 1.0) * (sizey * (this->Viewport[3] - this->Viewport[1])) / 2.0 +
      sizey * this->Viewport[1];

    x = dx;
    y = dy;
    // z = z; // this transform does not change the z
  }
}

//------------------------------------------------------------------------------
// Convert view point coordinates to world coordinates.
void vtkViewport::ViewToWorld()
{
  this->SetWorldPoint(this->ViewPoint[0], this->ViewPoint[1], this->ViewPoint[2], 1);
}

//------------------------------------------------------------------------------
// Convert world point coordinates to view coordinates.
void vtkViewport::WorldToView()
{

  this->SetViewPoint(this->WorldPoint[0], this->WorldPoint[1], this->WorldPoint[2]);
}

//------------------------------------------------------------------------------
// Return the size of the viewport in display coordinates.
int* vtkViewport::GetSize()
{
  if (this->VTKWindow)
  {
    int lowerLeft[2];
    double* vport = this->GetViewport();

    double vpu, vpv;
    vpu = vport[0];
    vpv = vport[1];
    this->NormalizedDisplayToDisplay(vpu, vpv);
    lowerLeft[0] = static_cast<int>(vpu + 0.5);
    lowerLeft[1] = static_cast<int>(vpv + 0.5);
    double vpu2, vpv2;
    vpu2 = vport[2];
    vpv2 = vport[3];
    this->NormalizedDisplayToDisplay(vpu2, vpv2);
    this->Size[0] = static_cast<int>(vpu2 + 0.5) - lowerLeft[0];
    this->Size[1] = static_cast<int>(vpv2 + 0.5) - lowerLeft[1];
  }
  else
  {
    this->Size[0] = this->Size[1] = 0;
  }

  return this->Size;
}

//------------------------------------------------------------------------------
// Return the origin of the viewport in display coordinates.
int* vtkViewport::GetOrigin()
{
  if (this->VTKWindow)
  {
    const int* winSize = this->VTKWindow->GetSize();

    // Round the origin up a pixel
    this->Origin[0] = static_cast<int>(this->Viewport[0] * static_cast<double>(winSize[0]) + 0.5);
    this->Origin[1] = static_cast<int>(this->Viewport[1] * static_cast<double>(winSize[1]) + 0.5);
  }
  else
  {
    this->Origin[0] = this->Origin[1] = 0;
  }

  return this->Origin;
}

//------------------------------------------------------------------------------
// Return the center of this Viewport in display coordinates.
double* vtkViewport::GetCenter()
{
  if (this->VTKWindow)
  {
    // get physical window dimensions
    const int* size = this->GetVTKWindow()->GetSize();
    if (size)
    {
      this->Center[0] = ((this->Viewport[2] + this->Viewport[0]) / 2.0 * size[0]);
      this->Center[1] = ((this->Viewport[3] + this->Viewport[1]) / 2.0 * size[1]);
    }
  }
  else
  {
    this->Center[0] = this->Center[1] = 0;
  }

  return this->Center;
}

//------------------------------------------------------------------------------
// Is a given display point in this Viewport's viewport.
int vtkViewport::IsInViewport(int x, int y)
{
  if (this->VTKWindow)
  {
    // get physical window dimensions
    const int* size = this->GetVTKWindow()->GetSize();
    if (size)
    {
      if ((this->Viewport[0] * size[0] <= x) && (this->Viewport[2] * size[0] >= x) &&
        (this->Viewport[1] * size[1] <= y) && (this->Viewport[3] * size[1] >= y))
      {
        return 1;
      }
    }
  }

  return 0;
}

//------------------------------------------------------------------------------
void vtkViewport::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Aspect: (" << this->Aspect[0] << ", " << this->Aspect[1] << ")\n";

  os << indent << "PixelAspect: (" << this->PixelAspect[0] << ", " << this->PixelAspect[1] << ")\n";

  os << indent << "Background: (" << this->Background[0] << ", " << this->Background[1] << ", "
     << this->Background[2] << ")\n";

  os << indent << "Background2: (" << this->Background2[0] << ", " << this->Background2[1] << ", "
     << this->Background2[2] << ")\n";

  os << indent << "BackgroundAlpha: " << this->BackgroundAlpha << "\n";

  os << indent << "GradientBackground: " << (this->GradientBackground ? "On" : "Off") << "\n";

  os << indent << "Viewport: (" << this->Viewport[0] << ", " << this->Viewport[1] << ", "
     << this->Viewport[2] << ", " << this->Viewport[3] << ")\n";

  os << indent << "Displaypoint: (" << this->DisplayPoint[0] << ", " << this->DisplayPoint[1]
     << ", " << this->DisplayPoint[2] << ")\n";

  os << indent << "Viewpoint: (" << this->ViewPoint[0] << ", " << this->ViewPoint[1] << ", "
     << this->ViewPoint[2] << ")\n";

  os << indent << "Worldpoint: (" << this->WorldPoint[0] << ", " << this->WorldPoint[1] << ", "
     << this->WorldPoint[2] << ", " << this->WorldPoint[3] << ")\n";

  os << indent << "Pick Position X1 Y1: " << this->PickX1 << " " << this->PickY1 << endl;
  os << indent << "Pick Position X2 Y2: " << this->PickX2 << " " << this->PickY2 << endl;
  os << indent << "PickedZ: " << this->PickedZ << "\n";
  os << indent << "Props:\n";
  this->Props->PrintSelf(os, indent.GetNextIndent());
  os << indent << "PickResultProps:\n";
  if (this->PickResultProps != nullptr)
  {
    this->PickResultProps->PrintSelf(os, indent.GetNextIndent());
  }
  else
  {
    os << indent << "nullptr\n";
  }
}

//------------------------------------------------------------------------------
void vtkViewport::LocalDisplayToDisplay(double& vtkNotUsed(u), double& v)
{
  if (this->VTKWindow)
  {
    /* get physical window dimensions */
    const int* size = this->VTKWindow->GetSize();
    if (size)
    {
      v = size[1] - v - 1;
    }
  }
}

//------------------------------------------------------------------------------
void vtkViewport::DisplayToLocalDisplay(double& vtkNotUsed(u), double& v)
{
  if (this->VTKWindow)
  {
    /* get physical window dimensions */
    const int* size = this->VTKWindow->GetSize();
    if (size)
    {
      v = size[1] - v - 1;
    }
  }
}

//------------------------------------------------------------------------------
void vtkViewport::DisplayToNormalizedDisplay(double& u, double& v)
{
  if (this->VTKWindow)
  {
    /* get physical window dimensions */
    const int* size = this->VTKWindow->GetSize();

    if (size && size[0] != 0 && size[1] != 0)
    {
      u = u / size[0];
      v = v / size[1];
    }
  }
}

//------------------------------------------------------------------------------
void vtkViewport::NormalizedDisplayToViewport(double& u, double& v)
{
  if (this->VTKWindow)
  {
    // get the pixel value for the viewport origin
    double vpou, vpov;
    vpou = this->Viewport[0];
    vpov = this->Viewport[1];
    this->NormalizedDisplayToDisplay(vpou, vpov);

    // get the pixel value for the coordinate
    this->NormalizedDisplayToDisplay(u, v);

    // subtract the vpo
    u = u - vpou;
    v = v - vpov;
  }
}

//------------------------------------------------------------------------------
void vtkViewport::ViewportToNormalizedViewport(double& u, double& v)
{
  if (this->VTKWindow)
  {
    /* get physical window dimensions */
    /*
      double vpsizeu, vpsizev;
      const int *size = this->VTKWindow->GetSize();
      vpsizeu = size[0]*(this->Viewport[2] - this->Viewport[0]);
      vpsizev = size[1]*(this->Viewport[3] - this->Viewport[1]);

      u = u/(vpsizeu - 1.0);
      v = v/(vpsizev - 1.0);
    */
    const int* size = this->GetSize();
    if (size && size[0] != 0 && size[1] != 0)
    {
      u = u / size[0];
      v = v / size[1];
    }
  }
}

//------------------------------------------------------------------------------
void vtkViewport::NormalizedViewportToView(double& x, double& y, double& vtkNotUsed(z))
{
  if (this->VTKWindow)
  {
    // for tiling we must consider the tiledViewport
    double* tvport = this->VTKWindow->GetTileViewport();

    // what part of the full viewport is the current tiled viewport?
    double* vport = this->GetViewport();
    double nvport[4];
    this->GetViewport(nvport);

    // clip the viewport to the tiled viewport
    if (nvport[0] < tvport[0])
    {
      nvport[0] = tvport[0];
    }
    if (nvport[1] < tvport[1])
    {
      nvport[1] = tvport[1];
    }
    if (nvport[2] > tvport[2])
    {
      nvport[2] = tvport[2];
    }
    if (nvport[3] > tvport[3])
    {
      nvport[3] = tvport[3];
    }

    x = x * (vport[2] - vport[0]) + vport[0];
    y = y * (vport[3] - vport[1]) + vport[1];

    x = (x - nvport[0]) / (nvport[2] - nvport[0]);
    y = (y - nvport[1]) / (nvport[3] - nvport[1]);

    x = (2.0 * x - 1.0);
    y = (2.0 * y - 1.0);
  }
}

//------------------------------------------------------------------------------
void vtkViewport::NormalizedDisplayToDisplay(double& u, double& v)
{
  if (this->VTKWindow)
  {
    /* get physical window dimensions */
    const int* size = this->VTKWindow->GetSize();
    if (size)
    {
      u = u * size[0];
      v = v * size[1];
    }
  }
}

//------------------------------------------------------------------------------
void vtkViewport::ViewportToNormalizedDisplay(double& u, double& v)
{
  if (this->VTKWindow)
  {
    // get the pixel value for the viewport origin
    double vpou, vpov;
    vpou = this->Viewport[0];
    vpov = this->Viewport[1];
    this->NormalizedDisplayToDisplay(vpou, vpov);

    // add the vpo
    u = u + vpou;
    v = v + vpov;

    // get the pixel value for the coordinate
    this->DisplayToNormalizedDisplay(u, v);
  }
}

//------------------------------------------------------------------------------
void vtkViewport::NormalizedViewportToViewport(double& u, double& v)
{
  if (this->VTKWindow)
  {
    /* get physical window dimensions */
    /*
      double vpsizeu, vpsizev;
      const int *size = this->VTKWindow->GetSize();
      vpsizeu = size[0]*(this->Viewport[2] - this->Viewport[0]);
      vpsizev = size[1]*(this->Viewport[3] - this->Viewport[1]);
      u = u * (vpsizeu - 1.0);
      v = v * (vpsizev - 1.0);
    */
    const int* size = this->GetSize();
    if (size)
    {
      u = u * size[0];
      v = v * size[1];
    }
  }
}

//------------------------------------------------------------------------------
void vtkViewport::ViewToNormalizedViewport(double& x, double& y, double& vtkNotUsed(z))
{
  if (this->VTKWindow)
  {
    // for tiling we must consider the tiledViewport
    double* tvport = this->VTKWindow->GetTileViewport();

    // what part of the full viewport is the current tiled viewport?
    double* vport = this->GetViewport();
    double nvport[4];
    this->GetViewport(nvport);

    // clip the viewport to the tiled viewport
    if (nvport[0] < tvport[0])
    {
      nvport[0] = tvport[0];
    }
    if (nvport[1] < tvport[1])
    {
      nvport[1] = tvport[1];
    }
    if (nvport[2] > tvport[2])
    {
      nvport[2] = tvport[2];
    }
    if (nvport[3] > tvport[3])
    {
      nvport[3] = tvport[3];
    }

    x = (x + 1.0) / 2.0;
    y = (y + 1.0) / 2.0;

    // now x and y are in the normalized viewport of the clipped viewport
    // we need to convert that to the normalized viewport of the entire
    // viewport
    x = nvport[0] + x * (nvport[2] - nvport[0]);
    y = nvport[1] + y * (nvport[3] - nvport[1]);
    x = (x - vport[0]) / (vport[2] - vport[0]);
    y = (y - vport[1]) / (vport[3] - vport[1]);
  }
}

void vtkViewport::ComputeAspect()
{
  if (this->VTKWindow)
  {
    // get the bounds of the window
    const int* size = this->VTKWindow->GetSize();
    if (!size)
    {
      return;
    }
    double* vport = this->GetViewport();

    if (!std::equal(size, size + 2, this->LastComputeAspectSize.begin()) ||
      !std::equal(vport, vport + 4, this->LastComputeAspectVPort.begin()) ||
      !std::equal(
        this->PixelAspect, this->PixelAspect + 2, this->LastComputeAspectPixelAspect.begin()))
    {
      std::copy(size, size + 2, this->LastComputeAspectSize.begin());
      std::copy(vport, vport + 4, this->LastComputeAspectVPort.begin());
      std::copy(
        this->PixelAspect, this->PixelAspect + 2, this->LastComputeAspectPixelAspect.begin());

      int lowerLeft[2], upperRight[2];
      lowerLeft[0] = static_cast<int>(vport[0] * size[0] + 0.5);
      lowerLeft[1] = static_cast<int>(vport[1] * size[1] + 0.5);
      upperRight[0] = static_cast<int>(vport[2] * size[0] + 0.5);
      upperRight[1] = static_cast<int>(vport[3] * size[1] + 0.5);
      upperRight[0]--;
      upperRight[1]--;

      double aspect[2];
      if ((upperRight[0] - lowerLeft[0] + 1) != 0 && (upperRight[1] - lowerLeft[1] + 1) != 0)
      {
        aspect[0] = static_cast<double>(upperRight[0] - lowerLeft[0] + 1) /
          static_cast<double>(upperRight[1] - lowerLeft[1] + 1) * this->PixelAspect[0];
      }
      else
      {
        // it happens if the vtkWindow is attached to the vtkViewport but
        // the vtkWindow is not initialized yet, so size[0]==0 and size[1]==0
        aspect[0] = this->PixelAspect[0];
      }
      aspect[1] = 1.0 * this->PixelAspect[1];

      this->SetAspect(aspect);
    }
  }
}

//------------------------------------------------------------------------------
vtkAssemblyPath* vtkViewport::PickPropFrom(
  double selectionX, double selectionY, vtkPropCollection* pickfrom)
{
  this->PickFromProps = pickfrom;
  return this->PickProp(selectionX, selectionY);
}

//------------------------------------------------------------------------------
vtkAssemblyPath* vtkViewport::PickPropFrom(double selectionX1, double selectionY1,
  double selectionX2, double selectionY2, vtkPropCollection* pickfrom)
{
  this->PickFromProps = pickfrom;
  return this->PickProp(selectionX1, selectionY1, selectionX2, selectionY2);
}

//------------------------------------------------------------------------------
// This complicated method determines the size of the current tile in pixels
// this is useful in computeing the actual aspcet ration of the current tile
void vtkViewport::GetTiledSize(int* usize, int* vsize)
{
  int llx, lly;
  this->GetTiledSizeAndOrigin(usize, vsize, &llx, &lly);
}

//------------------------------------------------------------------------------
void vtkViewport::GetTiledSizeAndOrigin(int* usize, int* vsize, int* lowerLeftU, int* lowerLeftV)
{
  double* vport;

  // find out if we should stereo render
  vport = this->GetViewport();

  // if there is no window assume 0 1
  double tileViewPort[4];
  if (this->GetVTKWindow())
  {
    this->GetVTKWindow()->GetTileViewport(tileViewPort);
  }
  else
  {
    tileViewPort[0] = 0;
    tileViewPort[1] = 0;
    tileViewPort[2] = 1;
    tileViewPort[3] = 1;
  }

  // find the lower left corner of the viewport, taking into account the
  // lower left boundary of this tile
  double vpu = vtkMath::ClampValue(vport[0] - tileViewPort[0], 0.0, 1.0);
  double vpv = vtkMath::ClampValue(vport[1] - tileViewPort[1], 0.0, 1.0);
  // store the result as a pixel value
  this->NormalizedDisplayToDisplay(vpu, vpv);
  *lowerLeftU = static_cast<int>(vpu + 0.5);
  *lowerLeftV = static_cast<int>(vpv + 0.5);

  // find the upper right corner of the viewport, taking into account the
  // lower left boundary of this tile
  double vpu2 = vtkMath::ClampValue(vport[2] - tileViewPort[0], 0.0, 1.0);
  double vpv2 = vtkMath::ClampValue(vport[3] - tileViewPort[1], 0.0, 1.0);
  // also watch for the upper right boundary of the tile
  if (vpu2 > (tileViewPort[2] - tileViewPort[0]))
  {
    vpu2 = tileViewPort[2] - tileViewPort[0];
  }
  if (vpv2 > (tileViewPort[3] - tileViewPort[1]))
  {
    vpv2 = tileViewPort[3] - tileViewPort[1];
  }
  this->NormalizedDisplayToDisplay(vpu2, vpv2);
  // now compute the size of the intersection of the viewport with the
  // current tile
  *usize = static_cast<int>(vpu2 + 0.5) - *lowerLeftU;
  *vsize = static_cast<int>(vpv2 + 0.5) - *lowerLeftV;
  if (*usize < 0)
  {
    *usize = 0;
  }
  if (*vsize < 0)
  {
    *vsize = 0;
  }
}
