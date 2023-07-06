/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkRenderer.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkRenderer.h"

#include "vtkAreaPicker.h"
#include "vtkAssemblyNode.h"
#include "vtkAssemblyPath.h"
#include "vtkCamera.h"
#include "vtkCommand.h"
#include "vtkCuller.h"
#include "vtkCullerCollection.h"
#include "vtkFXAAOptions.h"
#include "vtkFrustumCoverageCuller.h"
#include "vtkHardwareSelector.h"
#include "vtkInformation.h"
#include "vtkLight.h"
#include "vtkLightCollection.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkOutputWindow.h"
#include "vtkPicker.h"
#include "vtkProp3DCollection.h"
#include "vtkPropCollection.h"
#include "vtkRect.h"
#include "vtkRenderPass.h"
#include "vtkRenderTimerLog.h"
#include "vtkRenderWindow.h"
#include "vtkRendererDelegate.h"
#include "vtkSelection.h"
#include "vtkSelectionNode.h"
#include "vtkTexture.h"
#include "vtkTimerLog.h"
#include "vtkVectorOperators.h"
#include "vtkVolume.h"

#include <sstream>

vtkCxxSetObjectMacro(vtkRenderer, Information, vtkInformation);
vtkCxxSetObjectMacro(vtkRenderer, Delegate, vtkRendererDelegate);
vtkCxxSetObjectMacro(vtkRenderer, BackgroundTexture, vtkTexture);
vtkCxxSetObjectMacro(vtkRenderer, RightBackgroundTexture, vtkTexture);
vtkCxxSetObjectMacro(vtkRenderer, Pass, vtkRenderPass);
vtkCxxSetObjectMacro(vtkRenderer, FXAAOptions, vtkFXAAOptions);

//------------------------------------------------------------------------------
vtkObjectFactoryNewMacro(vtkRenderer);

// Create a vtkRenderer with a black background, a white ambient light,
// two-sided lighting turned on, a viewport of (0,0,1,1), and backface culling
// turned off.
vtkRenderer::vtkRenderer()
{
  this->PickedProp = nullptr;
  this->ActiveCamera = nullptr;

  this->Ambient[0] = 1;
  this->Ambient[1] = 1;
  this->Ambient[2] = 1;

  this->AllocatedRenderTime = 100;
  this->TimeFactor = 1.0;

  this->CreatedLight = nullptr;
  this->AutomaticLightCreation = 1;

  this->TwoSidedLighting = 1;
  this->BackingStore = 0;
  this->BackingImage = nullptr;
  this->BackingStoreSize[0] = -1;
  this->BackingStoreSize[1] = -1;
  this->LastRenderTimeInSeconds = -1.0;

  this->RenderWindow = nullptr;
  this->Lights = vtkLightCollection::New();
  this->Actors = vtkActorCollection::New();
  this->Volumes = vtkVolumeCollection::New();

  this->LightFollowCamera = 1;

  this->NumberOfPropsRendered = 0;

  this->PropArray = nullptr;
  this->PropArrayCount = 0;

  this->Layer = 0;
  this->PreserveColorBuffer = 0;
  this->PreserveDepthBuffer = 0;

  this->ComputedVisiblePropBounds[0] = VTK_DOUBLE_MAX;
  this->ComputedVisiblePropBounds[1] = -VTK_DOUBLE_MAX;
  this->ComputedVisiblePropBounds[2] = VTK_DOUBLE_MAX;
  this->ComputedVisiblePropBounds[3] = -VTK_DOUBLE_MAX;
  this->ComputedVisiblePropBounds[4] = VTK_DOUBLE_MAX;
  this->ComputedVisiblePropBounds[5] = -VTK_DOUBLE_MAX;

  this->Interactive = 1;
  this->Cullers = vtkCullerCollection::New();
  vtkFrustumCoverageCuller* cull = vtkFrustumCoverageCuller::New();
  this->Cullers->AddItem(cull);
  cull->Delete();

  // a value of 0 indicates it is uninitialized
  this->NearClippingPlaneTolerance = 0;

  this->ClippingRangeExpansion = 0.5;

  this->Erase = 1;
  this->Draw = 1;

  this->GL2PSSpecialPropCollection = nullptr;

  this->UseFXAA = false;
  this->FXAAOptions = vtkFXAAOptions::New();

  this->UseShadows = 0;

  this->UseHiddenLineRemoval = 0;

  this->UseDepthPeeling = 0;
  this->UseDepthPeelingForVolumes = false;
  this->OcclusionRatio = 0.0;
  this->MaximumNumberOfPeels = 4;
  this->LastRenderingUsedDepthPeeling = 0;

  this->Selector = nullptr;
  this->Delegate = nullptr;

  this->TexturedBackground = false;
  this->BackgroundTexture = nullptr;
  this->RightBackgroundTexture = nullptr;

  this->Pass = nullptr;

  this->Information = vtkInformation::New();
  this->Information->Register(this);
  this->Information->Delete();

  this->UseImageBasedLighting = false;
  this->EnvironmentTexture = nullptr;
  this->EnvironmentUp[0] = 0.0;
  this->EnvironmentUp[1] = 1.0;
  this->EnvironmentUp[2] = 0.0;
  this->EnvironmentRight[0] = 1.0;
  this->EnvironmentRight[1] = 0.0;
  this->EnvironmentRight[2] = 0.0;
}

vtkRenderer::~vtkRenderer()
{
  this->SetRenderWindow(nullptr);

  if (this->ActiveCamera)
  {
    this->ActiveCamera->UnRegister(this);
    this->ActiveCamera = nullptr;
  }

  if (this->CreatedLight)
  {
    this->CreatedLight->UnRegister(this);
    this->CreatedLight = nullptr;
  }

  delete[] this->BackingImage;

  this->Actors->Delete();
  this->Actors = nullptr;
  this->Volumes->Delete();
  this->Volumes = nullptr;
  this->Lights->Delete();
  this->Lights = nullptr;
  this->Cullers->Delete();
  this->Cullers = nullptr;

  if (this->FXAAOptions != nullptr)
  {
    this->FXAAOptions->Delete();
    this->FXAAOptions = nullptr;
  }

  if (this->Delegate != nullptr)
  {
    this->Delegate->UnRegister(this);
  }

  if (this->BackgroundTexture != nullptr)
  {
    this->BackgroundTexture->Delete();
  }

  if (this->RightBackgroundTexture != nullptr)
  {
    this->RightBackgroundTexture->Delete();
  }

  this->SetInformation(nullptr);

  if (this->EnvironmentTexture != nullptr)
  {
    this->EnvironmentTexture->Delete();
  }
}

void vtkRenderer::SetLeftBackgroundTexture(vtkTexture* texture)
{
  this->SetBackgroundTexture(texture);
}

vtkTexture* vtkRenderer::GetLeftBackgroundTexture()
{
  return this->GetBackgroundTexture();
}

void vtkRenderer::ReleaseGraphicsResources(vtkWindow* renWin)
{
  if (this->EnvironmentTexture != nullptr)
  {
    this->EnvironmentTexture->ReleaseGraphicsResources(renWin);
  }
  if (this->BackgroundTexture != nullptr)
  {
    this->BackgroundTexture->ReleaseGraphicsResources(renWin);
  }
  if (this->RightBackgroundTexture != nullptr)
  {
    this->RightBackgroundTexture->ReleaseGraphicsResources(renWin);
  }
  vtkProp* aProp;
  vtkCollectionSimpleIterator pit;
  this->Props->InitTraversal(pit);
  for (aProp = this->Props->GetNextProp(pit); aProp != nullptr;
       aProp = this->Props->GetNextProp(pit))
  {
    aProp->ReleaseGraphicsResources(renWin);
  }
}

// Concrete render method.
void vtkRenderer::Render()
{
  vtkRenderTimerLog* timer = this->RenderWindow->GetRenderTimer();
  VTK_SCOPED_RENDER_EVENT(
    "vtkRenderer::Render this=@" << std::hex << this << " Layer=" << std::dec << this->Layer,
    timer);

  if (this->Delegate != nullptr && this->Delegate->GetUsed())
  {
    this->Delegate->Render(this);
    return;
  }

  double t1, t2;
  int i;
  vtkProp* aProp;
  int* size;

  // If Draw is not on, ignore the render.
  if (!this->Draw)
  {
    vtkDebugMacro("Ignoring render because Draw is off.");
    return;
  }

  t1 = vtkTimerLog::GetUniversalTime();

  this->InvokeEvent(vtkCommand::StartEvent, nullptr);

  size = this->RenderWindow->GetSize();

  // if backing store is on and we have a stored image
  if (this->BackingStore && this->BackingImage && this->MTime < this->RenderTime &&
    this->ActiveCamera->GetMTime() < this->RenderTime &&
    this->RenderWindow->GetMTime() < this->RenderTime && this->BackingStoreSize[0] == size[0] &&
    this->BackingStoreSize[1] == size[1])
  {
    int mods = 0;
    vtkLight* light;

    // now we just need to check the lights and actors
    vtkCollectionSimpleIterator sit;
    for (this->Lights->InitTraversal(sit); (light = this->Lights->GetNextLight(sit));)
    {
      if (light->GetSwitch() && light->GetMTime() > this->RenderTime)
      {
        mods = 1;
        goto completed_mod_check;
      }
    }
    vtkCollectionSimpleIterator pit;
    for (this->Props->InitTraversal(pit); (aProp = this->Props->GetNextProp(pit));)
    {
      // if it's invisible, we can skip the rest
      if (aProp->GetVisibility())
      {
        if (aProp->GetRedrawMTime() > this->RenderTime)
        {
          mods = 1;
          goto completed_mod_check;
        }
      }
    }

  completed_mod_check:

    if (!mods)
    {
      int rx1, ry1, rx2, ry2;

      // backing store should be OK, lets use it
      // calc the pixel range for the renderer
      rx1 = static_cast<int>(this->Viewport[0] * (this->RenderWindow->GetSize()[0] - 1));
      ry1 = static_cast<int>(this->Viewport[1] * (this->RenderWindow->GetSize()[1] - 1));
      rx2 = static_cast<int>(this->Viewport[2] * (this->RenderWindow->GetSize()[0] - 1));
      ry2 = static_cast<int>(this->Viewport[3] * (this->RenderWindow->GetSize()[1] - 1));
      this->RenderWindow->SetPixelData(rx1, ry1, rx2, ry2, this->BackingImage, 0);
      this->InvokeEvent(vtkCommand::EndEvent, nullptr);
      return;
    }
  }

  timer->MarkStartEvent("Culling props");

  // Create the initial list of visible props
  // This will be passed through AllocateTime(), where
  // a time is allocated for each prop, and the list
  // maybe re-ordered by the cullers. Also create the
  // sublists for the props that need ray casting, and
  // the props that need to be rendered into an image.
  // Fill these in later (in AllocateTime) - get a
  // count of them there too
  if (this->Props->GetNumberOfItems() > 0)
  {
    this->PropArray = new vtkProp*[this->Props->GetNumberOfItems()];
  }
  else
  {
    this->PropArray = nullptr;
  }

  this->PropArrayCount = 0;
  vtkCollectionSimpleIterator pit;
  for (this->Props->InitTraversal(pit); (aProp = this->Props->GetNextProp(pit));)
  {
    if (aProp->GetVisibility())
    {
      this->PropArray[this->PropArrayCount++] = aProp;
    }
  }

  if (this->PropArrayCount == 0)
  {
    vtkDebugMacro(<< "There are no visible props!");
  }
  else
  {
    // Call all the culling methods to set allocated time
    // for each prop and re-order the prop list if desired

    this->AllocateTime();
  }

  timer->MarkEndEvent(); // culling

  // update camera ideal shift scale calcs
  this->ActiveCamera->UpdateIdealShiftScale(this->GetTiledAspectRatio());

  // do the render library specific stuff
  timer->MarkStartEvent("DeviceRender");
  this->DeviceRender();
  timer->MarkEndEvent();

  // If we aborted, restore old estimated times
  // Setting the allocated render time to zero also sets the
  // estimated render time to zero, so that when we add back
  // in the old value we have set it correctly.
  if (this->RenderWindow->GetAbortRender())
  {
    for (i = 0; i < this->PropArrayCount; i++)
    {
      this->PropArray[i]->RestoreEstimatedRenderTime();
    }
  }

  // Clean up the space we allocated before. If the PropArray exists,
  // they all should exist
  delete[] this->PropArray;
  this->PropArray = nullptr;

  if (this->BackingStore)
  {
    delete[] this->BackingImage;

    int rx1, ry1, rx2, ry2;

    // backing store should be OK, lets use it
    // calc the pixel range for the renderer
    rx1 = static_cast<int>(this->Viewport[0] * (size[0] - 1));
    ry1 = static_cast<int>(this->Viewport[1] * (size[1] - 1));
    rx2 = static_cast<int>(this->Viewport[2] * (size[0] - 1));
    ry2 = static_cast<int>(this->Viewport[3] * (size[1] - 1));
    this->BackingImage = this->RenderWindow->GetPixelData(rx1, ry1, rx2, ry2, 0);
    this->BackingStoreSize[0] = size[0];
    this->BackingStoreSize[1] = size[1];
  }

  // If we aborted, do not record the last render time.
  // Lets play around with determining the accuracy of the
  // EstimatedRenderTimes.  We can try to adjust for bad
  // estimates with the TimeFactor.
  if (!this->RenderWindow->GetAbortRender())
  {
    // Measure the actual RenderTime
    t2 = vtkTimerLog::GetUniversalTime();
    this->LastRenderTimeInSeconds = static_cast<double>(t2 - t1);

    if (this->LastRenderTimeInSeconds == 0.0)
    {
      this->LastRenderTimeInSeconds = 0.0001;
    }
    this->TimeFactor = this->AllocatedRenderTime / this->LastRenderTimeInSeconds;
  }
  this->InvokeEvent(vtkCommand::EndEvent, nullptr);
}

//------------------------------------------------------------------------------
void vtkRenderer::DeviceRenderOpaqueGeometry(vtkFrameBufferObjectBase* vtkNotUsed(fbo))
{
  this->UpdateOpaquePolygonalGeometry();
}

//------------------------------------------------------------------------------
// Description:
// Render translucent polygonal geometry. Default implementation just call
// UpdateTranslucentPolygonalGeometry().
// Subclasses of vtkRenderer that can deal with depth peeling must
// override this method.
void vtkRenderer::DeviceRenderTranslucentPolygonalGeometry(
  vtkFrameBufferObjectBase* vtkNotUsed(fbo))
{
  // Have to be set before a call to UpdateTranslucentPolygonalGeometry()
  // because UpdateTranslucentPolygonalGeometry() will eventually call
  // vtkOpenGLActor::Render() that uses this flag.
  this->LastRenderingUsedDepthPeeling = 0;

  this->UpdateTranslucentPolygonalGeometry();
}

//------------------------------------------------------------------------------
double vtkRenderer::GetAllocatedRenderTime()
{
  return this->AllocatedRenderTime;
}

double vtkRenderer::GetTimeFactor()
{
  return this->TimeFactor;
}

// Ask active camera to load its view matrix.
int vtkRenderer::UpdateCamera()
{
  VTK_SCOPED_RENDER_EVENT("vtkRenderer::UpdateCamera", this->RenderWindow->GetRenderTimer());

  if (!this->ActiveCamera)
  {
    vtkDebugMacro(<< "No cameras are on, creating one.");
    // the get method will automagically create a camera
    // and reset it since one hasn't been specified yet.
    // If is very unlikely that this can occur - if this
    // renderer is part of a vtkRenderWindow, the camera
    // will already have been created as part of the
    // DoStereoRender() method.
    this->GetActiveCameraAndResetIfCreated();
  }

  // update the viewing transformation
  this->ActiveCamera->Render(this);

  return 1;
}

vtkTypeBool vtkRenderer::UpdateLightsGeometryToFollowCamera()
{
  vtkCamera* camera;
  vtkLight* light;
  vtkMatrix4x4* lightMatrix;

  // only update the light's geometry if this Renderer is tracking
  // this lights.  That allows one renderer to view the lights that
  // another renderer is setting up.
  camera = this->GetActiveCameraAndResetIfCreated();
  lightMatrix = camera->GetCameraLightTransformMatrix();

  vtkCollectionSimpleIterator sit;
  for (this->Lights->InitTraversal(sit); (light = this->Lights->GetNextLight(sit));)
  {
    if (light->LightTypeIsSceneLight())
    {
      // Do nothing. Don't reset the transform matrix because applications
      // may have set a custom matrix. Only reset the transform matrix in
      // vtkLight::SetLightTypeToSceneLight()
    }
    else if (light->LightTypeIsHeadlight())
    {
      // update position and orientation of light to match camera.
      light->SetPosition(camera->GetPosition());
      light->SetFocalPoint(camera->GetFocalPoint());
    }
    else if (light->LightTypeIsCameraLight())
    {
      light->SetTransformMatrix(lightMatrix);
    }
    else
    {
      vtkErrorMacro(<< "light has unknown light type");
    }
  }
  return 1;
}

vtkTypeBool vtkRenderer::UpdateLightGeometry()
{
  VTK_SCOPED_RENDER_EVENT(
    "vtkRenderer::UpdateLightGeometry", this->GetRenderWindow()->GetRenderTimer());

  if (this->LightFollowCamera)
  {
    // only update the light's geometry if this Renderer is tracking
    // this lights.  That allows one renderer to view the lights that
    // another renderer is setting up.

    return this->UpdateLightsGeometryToFollowCamera();
  }

  return 1;
}

// Do all outer culling to set allocated time for each prop.
// Possibly re-order the actor list.
void vtkRenderer::AllocateTime()
{
  int initialized = 0;
  double renderTime;
  double totalTime;
  int i;
  vtkCuller* aCuller;
  vtkProp* aProp;

  // Give each of the cullers a chance to modify allocated rendering time
  // for the entire set of props. Each culler returns the total time given
  // by AllocatedRenderTime for all props. Each culler is required to
  // place any props that have an allocated render time of 0.0
  // at the end of the list. The PropArrayCount value that is
  // returned is the number of non-zero, visible actors.
  // Some cullers may do additional sorting of the list (by distance,
  // importance, etc).
  //
  // The first culler will initialize all the allocated render times.
  // Any subsequent culling will multiply the new render time by the
  // existing render time for an actor.

  totalTime = this->PropArrayCount;
  this->ComputeAspect();

  // It is very likely that the culler framework will call our
  // GetActiveCamera (say, to get the view frustrum planes for example).
  // This does not reset the camera anymore. If no camera has been
  // created though, we want it not only to be created but also reset
  // so that it behaves nicely for people who never bother with the camera
  // (i.e. neither call GetActiveCamera or ResetCamera). Of course,
  // it is very likely that the camera has already been created
  // (guaranteed if this renderer is being rendered as part of a
  // vtkRenderWindow).

  if (this->Cullers->GetNumberOfItems())
  {
    this->GetActiveCameraAndResetIfCreated();
  }

  vtkCollectionSimpleIterator sit;
  for (this->Cullers->InitTraversal(sit); (aCuller = this->Cullers->GetNextCuller(sit));)
  {
    totalTime = aCuller->Cull(this, this->PropArray, this->PropArrayCount, initialized);
  }

  // loop through all props and set the AllocatedRenderTime
  for (i = 0; i < this->PropArrayCount; i++)
  {
    aProp = this->PropArray[i];

    // If we don't have an outer cull method in any of the cullers,
    // then the allocated render time has not yet been initialized
    renderTime = (initialized) ? (aProp->GetRenderTimeMultiplier()) : (1.0);

    // We need to divide by total time so that the total rendering time
    // (all prop's AllocatedRenderTime added together) would be equal
    // to the renderer's AllocatedRenderTime.
    aProp->SetAllocatedRenderTime((renderTime / totalTime) * this->AllocatedRenderTime, this);
  }
}

// Ask actors to render themselves. As a side effect will cause
// visualization network to update.
int vtkRenderer::UpdateGeometry(vtkFrameBufferObjectBase* vtkNotUsed(fbo))
{
  int i;

  this->NumberOfPropsRendered = 0;

  if (this->PropArrayCount == 0)
  {
    return 0;
  }

  if (this->Selector)
  {
    // When selector is present, we are performing a selection,
    // so do the selection rendering pass instead of the normal passes.
    // Delegate the rendering of the props to the selector itself.

    // use pickfromprops ?
    if (this->PickFromProps)
    {
      vtkProp** pa;
      vtkProp* aProp;
      if (this->PickFromProps->GetNumberOfItems() > 0)
      {
        pa = new vtkProp*[this->PickFromProps->GetNumberOfItems()];
        int pac = 0;

        vtkCollectionSimpleIterator pit;
        for (this->PickFromProps->InitTraversal(pit);
             (aProp = this->PickFromProps->GetNextProp(pit));)
        {
          if (aProp->GetVisibility())
          {
            pa[pac++] = aProp;
          }
        }

        this->NumberOfPropsRendered = this->Selector->Render(this, pa, pac);
        delete[] pa;
      }
    }
    else
    {
      this->NumberOfPropsRendered =
        this->Selector->Render(this, this->PropArray, this->PropArrayCount);
    }

    this->RenderTime.Modified();
    vtkDebugMacro("Rendered " << this->NumberOfPropsRendered << " actors");
    return this->NumberOfPropsRendered;
  }

  // We can render everything because if it was
  // not visible it would not have been put in the
  // list in the first place, and if it was allocated
  // no time (culled) it would have been removed from
  // the list

  // Opaque geometry first:
  this->DeviceRenderOpaqueGeometry();

  // do the render library specific stuff about translucent polygonal geometry.
  // As it can be expensive, do a quick check if we can skip this step
  int hasTranslucentPolygonalGeometry = this->UseDepthPeelingForVolumes;
  for (i = 0; !hasTranslucentPolygonalGeometry && i < this->PropArrayCount; i++)
  {
    hasTranslucentPolygonalGeometry = this->PropArray[i]->HasTranslucentPolygonalGeometry();
  }
  if (hasTranslucentPolygonalGeometry)
  {
    this->DeviceRenderTranslucentPolygonalGeometry();
  }

  // loop through props and give them a chance to
  // render themselves as volumetric geometry.
  if (hasTranslucentPolygonalGeometry == 0 || !this->UseDepthPeelingForVolumes)
  {
    for (i = 0; i < this->PropArrayCount; i++)
    {
      this->NumberOfPropsRendered += this->PropArray[i]->RenderVolumetricGeometry(this);
    }
  }

  // loop through props and give them a chance to
  // render themselves as an overlay (or underlay)
  for (i = 0; i < this->PropArrayCount; i++)
  {
    this->NumberOfPropsRendered += this->PropArray[i]->RenderOverlay(this);
  }

  this->RenderTime.Modified();

  vtkDebugMacro(<< "Rendered " << this->NumberOfPropsRendered << " actors");

  return this->NumberOfPropsRendered;
}

//------------------------------------------------------------------------------
// Description:
// Ask all props to update and draw any translucent polygonal
// geometry. This includes both vtkActors and vtkVolumes
// Return the number of rendered props.
// It is called once with alpha blending technique. It is called multiple
// times with depth peeling technique.
int vtkRenderer::UpdateTranslucentPolygonalGeometry()
{
  int result = 0;
  // loop through props and give them a chance to
  // render themselves as translucent geometry
  for (int i = 0; i < this->PropArrayCount; i++)
  {
    int rendered = this->PropArray[i]->RenderTranslucentPolygonalGeometry(this);
    this->NumberOfPropsRendered += rendered;
    result += rendered;
  }
  return result;
}

//------------------------------------------------------------------------------
int vtkRenderer::UpdateOpaquePolygonalGeometry()
{
  int result = 0;
  for (int i = 0; i < this->PropArrayCount; i++)
  {
    result += this->PropArray[i]->RenderOpaqueGeometry(this);
  }
  this->NumberOfPropsRendered += result;
  return result;
}

//------------------------------------------------------------------------------
vtkWindow* vtkRenderer::GetVTKWindow()
{
  return this->RenderWindow;
}

//------------------------------------------------------------------------------
void vtkRenderer::SetLayer(int layer)
{
  vtkDebugMacro(<< this->GetClassName() << " (" << this << "): setting Layer to " << layer);
  if (this->Layer != layer)
  {
    this->Layer = layer;
    this->Modified();
  }
  this->SetPreserveColorBuffer(layer == 0 ? 0 : 1);
}

// Specify the camera to use for this renderer.
void vtkRenderer::SetActiveCamera(vtkCamera* cam)
{
  if (this->ActiveCamera == cam)
  {
    return;
  }

  if (this->ActiveCamera)
  {
    this->ActiveCamera->UnRegister(this);
    this->ActiveCamera = nullptr;
  }
  if (cam)
  {
    cam->Register(this);
  }

  this->ActiveCamera = cam;
  this->Modified();
  this->InvokeEvent(vtkCommand::ActiveCameraEvent, cam);
}

//------------------------------------------------------------------------------
vtkCamera* vtkRenderer::MakeCamera()
{
  vtkCamera* cam = vtkCamera::New();
  this->InvokeEvent(vtkCommand::CreateCameraEvent, cam);
  return cam;
}

//------------------------------------------------------------------------------
vtkCamera* vtkRenderer::GetActiveCamera()
{
  if (this->ActiveCamera == nullptr)
  {
    vtkCamera* cam = this->MakeCamera();
    this->SetActiveCamera(cam);
    cam->Delete();
    // The following line has been commented out as it has a lot of
    // side effects (like computing the bounds of all props, which will
    // eventually call UpdateInformation() on data objects, etc).
    // Instead, the rendering code has been updated to internally use
    // GetActiveCameraAndResetIfCreated which will reset the camera
    // if it gets created
    // this->ResetCamera();
  }

  return this->ActiveCamera;
}

//------------------------------------------------------------------------------
vtkCamera* vtkRenderer::GetActiveCameraAndResetIfCreated()
{
  if (this->ActiveCamera == nullptr)
  {
    this->GetActiveCamera();
    this->ResetCamera();
  }
  return this->ActiveCamera;
}

//------------------------------------------------------------------------------
void vtkRenderer::AddActor(vtkProp* p)
{
  this->AddViewProp(p);
}

//------------------------------------------------------------------------------
void vtkRenderer::AddVolume(vtkProp* p)
{
  this->AddViewProp(p);
}

//------------------------------------------------------------------------------
void vtkRenderer::RemoveActor(vtkProp* p)
{
  this->Actors->RemoveItem(p);
  this->RemoveViewProp(p);
}

//------------------------------------------------------------------------------
void vtkRenderer::RemoveVolume(vtkProp* p)
{
  this->Volumes->RemoveItem(p);
  this->RemoveViewProp(p);
}

// Add a light to the list of lights.
void vtkRenderer::AddLight(vtkLight* light)
{
  this->Lights->AddItem(light);
}

// look through the props and get all the actors
vtkActorCollection* vtkRenderer::GetActors()
{
  vtkProp* aProp;

  // clear the collection first
  this->Actors->RemoveAllItems();

  vtkCollectionSimpleIterator pit;
  for (this->Props->InitTraversal(pit); (aProp = this->Props->GetNextProp(pit));)
  {
    aProp->GetActors(this->Actors);
  }
  return this->Actors;
}

// look through the props and get all the volumes
vtkVolumeCollection* vtkRenderer::GetVolumes()
{
  vtkProp* aProp;

  // clear the collection first
  this->Volumes->RemoveAllItems();

  vtkCollectionSimpleIterator pit;
  for (this->Props->InitTraversal(pit); (aProp = this->Props->GetNextProp(pit));)
  {
    aProp->GetVolumes(this->Volumes);
  }
  return this->Volumes;
}

// Remove a light from the list of lights.
void vtkRenderer::RemoveLight(vtkLight* light)
{
  this->Lights->RemoveItem(light);
}

// Remove all lights from the list of lights.
void vtkRenderer::RemoveAllLights()
{
  this->Lights->RemoveAllItems();
}

// Add an culler to the list of cullers.
void vtkRenderer::AddCuller(vtkCuller* culler)
{
  this->Cullers->AddItem(culler);
}

// Remove an actor from the list of cullers.
void vtkRenderer::RemoveCuller(vtkCuller* culler)
{
  this->Cullers->RemoveItem(culler);
}

//------------------------------------------------------------------------------
void vtkRenderer::SetLightCollection(vtkLightCollection* lights)
{
  assert("pre lights_exist" && lights != nullptr);

  this->Lights->Delete(); // this->Lights is always not nullptr
  this->Lights = lights;
  this->Lights->Register(this);
  this->Modified();

  assert("post: lights_set" && lights == this->GetLights());
}

//------------------------------------------------------------------------------
vtkLight* vtkRenderer::MakeLight()
{
  return vtkLight::New();
}

void vtkRenderer::CreateLight()
{
  if (!this->AutomaticLightCreation)
  {
    return;
  }

  if (this->CreatedLight)
  {
    this->RemoveLight(this->CreatedLight);
    this->CreatedLight->UnRegister(this);
    this->CreatedLight = nullptr;
  }

  // I do not see why UnRegister is used on CreatedLight, but lets be
  // consistent.
  vtkLight* l = this->MakeLight();
  this->CreatedLight = l;
  this->CreatedLight->Register(this);
  this->AddLight(this->CreatedLight);
  l->Delete();

  this->CreatedLight->SetLightTypeToHeadlight();

  // set these values just to have a good default should LightFollowCamera
  // be turned off.
  this->CreatedLight->SetPosition(this->GetActiveCamera()->GetPosition());
  this->CreatedLight->SetFocalPoint(this->GetActiveCamera()->GetFocalPoint());
}

// Compute the bounds of the visible props
void vtkRenderer::ComputeVisiblePropBounds(double allBounds[6])
{
  vtkProp* prop;
  const double* bounds;
  int nothingVisible = 1;

  this->InvokeEvent(vtkCommand::ComputeVisiblePropBoundsEvent, this);

  allBounds[0] = allBounds[2] = allBounds[4] = VTK_DOUBLE_MAX;
  allBounds[1] = allBounds[3] = allBounds[5] = -VTK_DOUBLE_MAX;

  // loop through all props
  vtkCollectionSimpleIterator pit;
  for (this->Props->InitTraversal(pit); (prop = this->Props->GetNextProp(pit));)
  {
    // if it's invisible, or if its bounds should be ignored,
    // or has no geometry, we can skip the rest
    if (prop->GetVisibility() && prop->GetUseBounds())
    {
      bounds = prop->GetBounds();
      // make sure we haven't got bogus bounds
      if (bounds != nullptr && vtkMath::AreBoundsInitialized(bounds))
      {
        nothingVisible = 0;

        if (bounds[0] < allBounds[0])
        {
          allBounds[0] = bounds[0];
        }
        if (bounds[1] > allBounds[1])
        {
          allBounds[1] = bounds[1];
        }
        if (bounds[2] < allBounds[2])
        {
          allBounds[2] = bounds[2];
        }
        if (bounds[3] > allBounds[3])
        {
          allBounds[3] = bounds[3];
        }
        if (bounds[4] < allBounds[4])
        {
          allBounds[4] = bounds[4];
        }
        if (bounds[5] > allBounds[5])
        {
          allBounds[5] = bounds[5];
        }
      } // not bogus
    }
  }

  if (nothingVisible)
  {
    vtkMath::UninitializeBounds(allBounds);
    vtkDebugMacro(<< "Can't compute bounds, no 3D props are visible");
    return;
  }
}

double* vtkRenderer::ComputeVisiblePropBounds()
{
  this->ComputeVisiblePropBounds(this->ComputedVisiblePropBounds);
  return this->ComputedVisiblePropBounds;
}

// Automatically set up the camera based on the visible actors.
// The camera will reposition itself to view the center point of the actors,
// and move along its initial view plane normal (i.e., vector defined from
// camera position to focal point) so that all of the actors can be seen.
void vtkRenderer::ResetCamera()
{
  double allBounds[6];

  this->ComputeVisiblePropBounds(allBounds);

  if (!vtkMath::AreBoundsInitialized(allBounds))
  {
    vtkDebugMacro(<< "Cannot reset camera!");
  }
  else
  {
    this->ResetCamera(allBounds);
  }

  // Here to let parallel/distributed compositing intercept
  // and do the right thing.
  this->InvokeEvent(vtkCommand::ResetCameraEvent, this);
}

// Automatically set the clipping range of the camera based on the
// visible actors
void vtkRenderer::ResetCameraClippingRange()
{
  double allBounds[6];

  this->ComputeVisiblePropBounds(allBounds);

  if (!vtkMath::AreBoundsInitialized(allBounds))
  {
    vtkDebugMacro(<< "Cannot reset camera clipping range!");
  }
  else
  {
    this->ResetCameraClippingRange(allBounds);
  }

  // Here to let parallel/distributed compositing intercept
  // and do the right thing.
  this->InvokeEvent(vtkCommand::ResetCameraClippingRangeEvent, this);
}

// Automatically set up the camera based on a specified bounding box
// (xmin,xmax, ymin,ymax, zmin,zmax). Camera will reposition itself so
// that its focal point is the center of the bounding box, and adjust its
// distance and position to preserve its initial view plane normal
// (i.e., vector defined from camera position to focal point). Note: if
// the view plane is parallel to the view up axis, the view up axis will
// be reset to one of the three coordinate axes.
void vtkRenderer::ResetCamera(const double bounds[6])
{
  double center[3];
  double distance;
  double vn[3], *vup;

  this->GetActiveCamera();
  if (this->ActiveCamera != nullptr)
  {
    this->ActiveCamera->GetViewPlaneNormal(vn);
  }
  else
  {
    vtkErrorMacro(<< "Trying to reset non-existent camera");
    return;
  }

  // Reset the perspective zoom factors, otherwise subsequent zooms will cause
  // the view angle to become very small and cause bad depth sorting.
  this->ActiveCamera->SetViewAngle(30.0);

  double expandedBounds[6] = { bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5] };
  this->ExpandBounds(expandedBounds, this->ActiveCamera->GetModelTransformMatrix());

  center[0] = (expandedBounds[0] + expandedBounds[1]) / 2.0;
  center[1] = (expandedBounds[2] + expandedBounds[3]) / 2.0;
  center[2] = (expandedBounds[4] + expandedBounds[5]) / 2.0;

  double w1 = expandedBounds[1] - expandedBounds[0];
  double w2 = expandedBounds[3] - expandedBounds[2];
  double w3 = expandedBounds[5] - expandedBounds[4];
  w1 *= w1;
  w2 *= w2;
  w3 *= w3;
  double radius = w1 + w2 + w3;

  // If we have just a single point, pick a radius of 1.0
  radius = (radius == 0) ? (1.0) : (radius);

  // compute the radius of the enclosing sphere
  radius = sqrt(radius) * 0.5;

  // default so that the bounding sphere fits within the view fustrum

  // compute the distance from the intersection of the view frustum with the
  // bounding sphere. Basically in 2D draw a circle representing the bounding
  // sphere in 2D then draw a horizontal line going out from the center of
  // the circle. That is the camera view. Then draw a line from the camera
  // position to the point where it intersects the circle. (it will be tangent
  // to the circle at this point, this is important, only go to the tangent
  // point, do not draw all the way to the view plane). Then draw the radius
  // from the tangent point to the center of the circle. You will note that
  // this forms a right triangle with one side being the radius, another being
  // the target distance for the camera, then just find the target dist using
  // a sin.
  double angle = vtkMath::RadiansFromDegrees(this->ActiveCamera->GetViewAngle());
  double parallelScale = radius;

  this->ComputeAspect();
  double aspect[2];
  this->GetAspect(aspect);

  if (aspect[0] >= 1.0) // horizontal window, deal with vertical angle|scale
  {
    if (this->ActiveCamera->GetUseHorizontalViewAngle())
    {
      angle = 2.0 * atan(tan(angle * 0.5) / aspect[0]);
    }
  }
  else // vertical window, deal with horizontal angle|scale
  {
    if (!this->ActiveCamera->GetUseHorizontalViewAngle())
    {
      angle = 2.0 * atan(tan(angle * 0.5) * aspect[0]);
    }

    parallelScale = parallelScale / aspect[0];
  }

  distance = radius / sin(angle * 0.5);

  // check view-up vector against view plane normal
  vup = this->ActiveCamera->GetViewUp();
  if (fabs(vtkMath::Dot(vup, vn)) > 0.999)
  {
    vtkWarningMacro(<< "Resetting view-up since view plane normal is parallel");
    this->ActiveCamera->SetViewUp(-vup[2], vup[0], vup[1]);
  }

  // update the camera
  this->ActiveCamera->SetFocalPoint(center[0], center[1], center[2]);
  this->ActiveCamera->SetPosition(
    center[0] + distance * vn[0], center[1] + distance * vn[1], center[2] + distance * vn[2]);

  this->ResetCameraClippingRange(expandedBounds);

  // setup default parallel scale
  this->ActiveCamera->SetParallelScale(parallelScale);
}

// Alternative version of ResetCamera(bounds[6]);
void vtkRenderer::ResetCamera(
  double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  double bounds[6];

  bounds[0] = xmin;
  bounds[1] = xmax;
  bounds[2] = ymin;
  bounds[3] = ymax;
  bounds[4] = zmin;
  bounds[5] = zmax;

  this->ResetCamera(bounds);
}

// Reset the camera clipping range to include this entire bounding box
void vtkRenderer::ResetCameraClippingRange(const double bounds[6])
{
  double vn[3], position[3], a, b, c, d;
  double range[2], dist;
  int i, j, k;

  // Don't reset the clipping range when we don't have any 3D visible props
  if (!vtkMath::AreBoundsInitialized(bounds))
  {
    return;
  }

  this->GetActiveCameraAndResetIfCreated();
  if (this->ActiveCamera == nullptr)
  {
    vtkErrorMacro(<< "Trying to reset clipping range of non-existent camera");
    return;
  }

  double expandedBounds[6] = { bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5] };
  if (!this->ActiveCamera->GetUseOffAxisProjection())
  {
    this->ActiveCamera->GetViewPlaneNormal(vn);
    this->ActiveCamera->GetPosition(position);
    this->ExpandBounds(expandedBounds, this->ActiveCamera->GetModelTransformMatrix());
  }
  else
  {
    this->ActiveCamera->GetEyePosition(position);
    this->ActiveCamera->GetEyePlaneNormal(vn);
    this->ExpandBounds(expandedBounds, this->ActiveCamera->GetModelViewTransformMatrix());
  }

  a = -vn[0];
  b = -vn[1];
  c = -vn[2];
  d = -(a * position[0] + b * position[1] + c * position[2]);

  // Set the max near clipping plane and the min far clipping plane
  range[0] = a * expandedBounds[0] + b * expandedBounds[2] + c * expandedBounds[4] + d;
  range[1] = 1e-18;

  // Find the closest / farthest bounding box vertex
  for (k = 0; k < 2; k++)
  {
    for (j = 0; j < 2; j++)
    {
      for (i = 0; i < 2; i++)
      {
        dist = a * expandedBounds[i] + b * expandedBounds[2 + j] + c * expandedBounds[4 + k] + d;
        range[0] = (dist < range[0]) ? (dist) : (range[0]);
        range[1] = (dist > range[1]) ? (dist) : (range[1]);
      }
    }
  }

  // do not let far - near be less than 0.1 of the window height
  // this is for cases such as 2D images which may have zero range
  double minGap = 0.0;
  if (this->ActiveCamera->GetParallelProjection())
  {
    minGap = 0.1 * this->ActiveCamera->GetParallelScale();
  }
  else
  {
    double angle = vtkMath::RadiansFromDegrees(this->ActiveCamera->GetViewAngle());
    minGap = 0.2 * tan(angle / 2.0) * range[1];
  }
  if (range[1] - range[0] < minGap)
  {
    minGap = minGap - range[1] + range[0];
    range[1] += minGap / 2.0;
    range[0] -= minGap / 2.0;
  }

  // Do not let the range behind the camera throw off the calculation.
  if (range[0] < 0.0)
  {
    range[0] = 0.0;
  }

  // Give ourselves a little breathing room
  range[0] = 0.99 * range[0] - (range[1] - range[0]) * this->ClippingRangeExpansion;
  range[1] = 1.01 * range[1] + (range[1] - range[0]) * this->ClippingRangeExpansion;

  // Make sure near is not bigger than far
  range[0] = (range[0] >= range[1]) ? (0.01 * range[1]) : (range[0]);

  // Make sure near is at least some fraction of far - this prevents near
  // from being behind the camera or too close in front. How close is too
  // close depends on the resolution of the depth buffer
  if (!this->NearClippingPlaneTolerance)
  {
    this->NearClippingPlaneTolerance = 0.01;
    if (this->RenderWindow)
    {
      int ZBufferDepth = this->RenderWindow->GetDepthBufferSize();
      if (ZBufferDepth > 16)
      {
        this->NearClippingPlaneTolerance = 0.001;
      }
    }
  }

  // make sure the front clipping range is not too far from the far clippnig
  // range, this is to make sure that the zbuffer resolution is effectively
  // used
  if (range[0] < this->NearClippingPlaneTolerance * range[1])
  {
    range[0] = this->NearClippingPlaneTolerance * range[1];
  }

  this->ActiveCamera->SetClippingRange(range);
}

// Alternative version of ResetCameraClippingRange(bounds[6]);
void vtkRenderer::ResetCameraClippingRange(
  double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  double bounds[6];

  bounds[0] = xmin;
  bounds[1] = xmax;
  bounds[2] = ymin;
  bounds[3] = ymax;
  bounds[4] = zmin;
  bounds[5] = zmax;

  this->ResetCameraClippingRange(bounds);
}

// Automatically set up the camera based on the visible actors.
// Use a screen space bounding box to zoom closer to the data.
void vtkRenderer::ResetCameraScreenSpace()
{
  double allBounds[6];

  this->ComputeVisiblePropBounds(allBounds);

  if (!vtkMath::AreBoundsInitialized(allBounds))
  {
    vtkDebugMacro(<< "Cannot reset camera!");
  }
  else
  {
    this->ResetCameraScreenSpace(allBounds);
  }

  // Here to let parallel/distributed compositing intercept
  // and do the right thing.
  this->InvokeEvent(vtkCommand::ResetCameraEvent, this);
}

// Alternative version of ResetCameraScreenSpace(bounds[6]);
void vtkRenderer::ResetCameraScreenSpace(
  double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  double bounds[6];

  bounds[0] = xmin;
  bounds[1] = xmax;
  bounds[2] = ymin;
  bounds[3] = ymax;
  bounds[4] = zmin;
  bounds[5] = zmax;

  this->ResetCameraScreenSpace(bounds);
}

// Use a screen space bounding box to zoom closer to the data.
void vtkRenderer::ResetCameraScreenSpace(const double bounds[6])
{
  // Make sure all bounds are visible to project into screen space
  this->ResetCamera(bounds);

  double expandedBounds[6] = { bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5] };
  this->ExpandBounds(expandedBounds, this->ActiveCamera->GetModelTransformMatrix());

  // 1) Compute the screen space bounding box
  double xmin = VTK_DOUBLE_MAX;
  double ymin = VTK_DOUBLE_MAX;
  double xmax = VTK_DOUBLE_MIN;
  double ymax = VTK_DOUBLE_MIN;
  double currentPointDisplay[3];
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 2; ++j)
    {
      for (int k = 0; k < 2; ++k)
      {
        double currentPoint[4] = { expandedBounds[i], expandedBounds[j + 2], expandedBounds[k + 4],
          1.0 };

        this->SetWorldPoint(currentPoint);
        this->WorldToDisplay();
        this->GetDisplayPoint(currentPointDisplay);

        xmin = std::min(currentPointDisplay[0], xmin);
        xmax = std::max(currentPointDisplay[0], xmax);
        ymin = std::min(currentPointDisplay[1], ymin);
        ymax = std::max(currentPointDisplay[1], ymax);
      }
    }
  }

  // Project the focal point in screen space
  double fp[4];
  this->ActiveCamera->GetFocalPoint(fp);
  fp[3] = 1.0;
  double fpDisplay[3];
  this->SetWorldPoint(fp);
  this->WorldToDisplay();
  this->GetDisplayPoint(fpDisplay);

  // The focal point must be at the center of the box
  // So construct a box with fpDisplay at the center
  int xCenterFocalPoint = static_cast<int>(fpDisplay[0]);
  int yCenterFocalPoint = static_cast<int>(fpDisplay[1]);

  int xCenterBox = static_cast<int>((xmin + xmax) / 2);
  int yCenterBox = static_cast<int>((ymin + ymax) / 2);

  int xDiff = 2 * (xCenterFocalPoint - xCenterBox);
  int yDiff = 2 * (yCenterFocalPoint - yCenterBox);

  int xMaxOffset = std::max(xDiff, 0);
  int xMinOffset = std::min(xDiff, 0);
  int yMaxOffset = std::max(yDiff, 0);
  int yMinOffset = std::min(yDiff, 0);

  xmin += xMinOffset;
  xmax += xMaxOffset;
  ymin += yMinOffset;
  ymax += yMaxOffset;
  // Now the focal point is at the center of the box

  const vtkRecti box(xmin, ymin, xmax - xmin, ymax - ymin);
  // We let a 10% offset around the zoomed data
  this->ZoomToBoxUsingViewAngle(box, 0.9);
}

// Display to world using vtkVector3d
vtkVector3d vtkRenderer::DisplayToWorld(const vtkVector3d& display)
{
  this->SetDisplayPoint(display[0], display[1], display[2]);
  this->DisplayToView();
  this->ViewToWorld();

  vtkVector<double, 4> world4;
  this->GetWorldPoint(world4.GetData());
  double invw = 1.0 * world4[3];
  world4 = world4 * invw;
  return vtkVector3d(world4.GetData());
}

void vtkRenderer::ZoomToBoxUsingViewAngle(const vtkRecti& box, const double offsetRatio)
{
  const int* size = this->GetSize();
  double zf1 = size[0] / static_cast<double>(box.GetWidth());
  double zf2 = size[1] / static_cast<double>(box.GetHeight());
  double zoomFactor = std::min(zf1, zf2);

  // OffsetRatio will let a free space between the zoomed data
  // And the edges of the window
  this->GetActiveCamera()->Zoom(zoomFactor * offsetRatio);
}

// Specify the rendering window in which to draw. This is automatically set
// when the renderer is created by MakeRenderer.  The user probably
// shouldn't ever need to call this method.
// no reference counting!
void vtkRenderer::SetRenderWindow(vtkRenderWindow* renwin)
{
  if (renwin != this->RenderWindow)
  {
    this->ReleaseGraphicsResources(this->RenderWindow);
    this->VTKWindow = renwin;
    this->RenderWindow = renwin;
  }
}

// Given a pixel location, return the Z value
double vtkRenderer::GetZ(int x, int y)
{
  float* zPtr;
  double z;

  zPtr = this->RenderWindow->GetZbufferData(x, y, x, y);
  if (zPtr)
  {
    z = *zPtr;
    delete[] zPtr;
  }
  else
  {
    z = 1.0;
  }
  return z;
}

// Convert view point coordinates to world coordinates.
void vtkRenderer::ViewToWorld()
{
  double result[4];
  result[0] = this->ViewPoint[0];
  result[1] = this->ViewPoint[1];
  result[2] = this->ViewPoint[2];
  result[3] = 1.0;
  this->ViewToWorld(result[0], result[1], result[2]);
  this->SetWorldPoint(result);
}

void vtkRenderer::ViewToWorld(double& x, double& y, double& z)
{
  double mat[16];
  double result[4];

  if (this->ActiveCamera == nullptr)
  {
    vtkErrorMacro("ViewToWorld: no active camera, cannot compute view to world, returning 0,0,0");
    x = y = z = 0.0;
    return;
  }

  // get the perspective transformation from the active camera
  vtkMatrix4x4* matrix =
    this->ActiveCamera->GetCompositeProjectionTransformMatrix(this->GetTiledAspectRatio(), 0, 1);

  // use the inverse matrix
  vtkMatrix4x4::Invert(*matrix->Element, mat);

  // Transform point to world coordinates
  result[0] = x;
  result[1] = y;
  result[2] = z;
  result[3] = 1.0;

  vtkMatrix4x4::MultiplyPoint(mat, result, result);

  // Get the transformed vector & set WorldPoint
  // while we are at it try to keep w at one
  if (result[3])
  {
    x = result[0] / result[3];
    y = result[1] / result[3];
    z = result[2] / result[3];
  }
}

// Convert world point coordinates to view coordinates.
void vtkRenderer::WorldToView()
{
  double result[3];
  result[0] = this->WorldPoint[0];
  result[1] = this->WorldPoint[1];
  result[2] = this->WorldPoint[2];
  this->WorldToView(result[0], result[1], result[2]);
  this->SetViewPoint(result[0], result[1], result[2]);
}

// Convert world point coordinates to view coordinates.
void vtkRenderer::WorldToView(double& x, double& y, double& z)
{
  double view[4];

  // get the perspective transformation from the active camera
  if (!this->ActiveCamera)
  {
    vtkErrorMacro("WorldToView: no active camera, cannot compute world to view, returning 0,0,0");
    x = y = z = 0.0;
    return;
  }
  const auto& mat = this->GetCompositeProjectionTransformationMatrix();

  view[0] = x * mat[0] + y * mat[1] + z * mat[2] + mat[3];
  view[1] = x * mat[4] + y * mat[5] + z * mat[6] + mat[7];
  view[2] = x * mat[8] + y * mat[9] + z * mat[10] + mat[11];
  view[3] = x * mat[12] + y * mat[13] + z * mat[14] + mat[15];

  if (view[3] != 0.0)
  {
    x = view[0] / view[3];
    y = view[1] / view[3];
    z = view[2] / view[3];
  }
}

void vtkRenderer::WorldToPose(double& x, double& y, double& z)
{
  double view[4];

  // get the perspective transformation from the active camera
  if (!this->ActiveCamera)
  {
    vtkErrorMacro("WorldToPose: no active camera, cannot compute world to pose, returning 0,0,0");
    x = y = z = 0.0;
    return;
  }
  const auto& mat = this->GetViewTransformMatrix();

  view[0] = x * mat[0] + y * mat[1] + z * mat[2] + mat[3];
  view[1] = x * mat[4] + y * mat[5] + z * mat[6] + mat[7];
  view[2] = x * mat[8] + y * mat[9] + z * mat[10] + mat[11];
  view[3] = x * mat[12] + y * mat[13] + z * mat[14] + mat[15];

  if (view[3] != 0.0)
  {
    x = view[0] / view[3];
    y = view[1] / view[3];
    z = view[2] / view[3];
  }
}

void vtkRenderer::PoseToView(double& x, double& y, double& z)
{
  double view[4];

  // get the perspective transformation from the active camera
  if (!this->ActiveCamera)
  {
    vtkErrorMacro("PoseToView: no active camera, cannot compute pose to view, returning 0,0,0");
    x = y = z = 0.0;
    return;
  }
  const auto& mat = this->GetProjectionTransformationMatrix();

  view[0] = x * mat[0] + y * mat[1] + z * mat[2] + mat[3];
  view[1] = x * mat[4] + y * mat[5] + z * mat[6] + mat[7];
  view[2] = x * mat[8] + y * mat[9] + z * mat[10] + mat[11];
  view[3] = x * mat[12] + y * mat[13] + z * mat[14] + mat[15];

  if (view[3] != 0.0)
  {
    x = view[0] / view[3];
    y = view[1] / view[3];
    z = view[2] / view[3];
  }
}

void vtkRenderer::PoseToWorld(double& x, double& y, double& z)
{
  double mat[16];
  double result[4];

  if (this->ActiveCamera == nullptr)
  {
    vtkErrorMacro("PoseToWorld: no active camera, cannot compute pose to world, returning 0,0,0");
    x = y = z = 0.0;
    return;
  }

  // get the perspective transformation from the active camera
  const auto& matrix = this->GetViewTransformMatrix();

  // use the inverse matrix
  vtkMatrix4x4::Invert(matrix.data(), mat);

  // Transform point to world coordinates
  result[0] = x;
  result[1] = y;
  result[2] = z;
  result[3] = 1.0;

  vtkMatrix4x4::MultiplyPoint(mat, result, result);

  // Get the transformed vector & set WorldPoint
  // while we are at it try to keep w at one
  if (result[3])
  {
    x = result[0] / result[3];
    y = result[1] / result[3];
    z = result[2] / result[3];
  }
}

void vtkRenderer::ViewToPose(double& x, double& y, double& z)
{
  double mat[16];
  double result[4];

  if (this->ActiveCamera == nullptr)
  {
    vtkErrorMacro("ViewToPose: no active camera, cannot compute view to pose, returning 0,0,0");
    x = y = z = 0.0;
    return;
  }

  const auto& matrix = this->GetProjectionTransformationMatrix();
  vtkMatrix4x4::Invert(matrix.data(), mat);
  // Transform point to world coordinates
  result[0] = x;
  result[1] = y;
  result[2] = z;
  result[3] = 1.0;

  vtkMatrix4x4::MultiplyPoint(mat, result, result);

  // Get the transformed vector & set WorldPoint
  // while we are at it try to keep w at one
  if (result[3])
  {
    x = result[0] / result[3];
    y = result[1] / result[3];
    z = result[2] / result[3];
  }
}

void vtkRenderer::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Near Clipping Plane Tolerance: " << this->NearClippingPlaneTolerance << "\n";

  os << indent << "ClippingRangeExpansion: " << this->ClippingRangeExpansion << "\n";

  os << indent << "Ambient: (" << this->Ambient[0] << ", " << this->Ambient[1] << ", "
     << this->Ambient[2] << ")\n";

  os << indent << "Backing Store: " << (this->BackingStore ? "On\n" : "Off\n");
  os << indent << "Display Point: (" << this->DisplayPoint[0] << ", " << this->DisplayPoint[1]
     << ", " << this->DisplayPoint[2] << ")\n";
  os << indent << "Lights:\n";
  this->Lights->PrintSelf(os, indent.GetNextIndent());

  os << indent << "Light Follow Camera: " << (this->LightFollowCamera ? "On\n" : "Off\n");

  os << indent << "View Point: (" << this->ViewPoint[0] << ", " << this->ViewPoint[1] << ", "
     << this->ViewPoint[2] << ")\n";

  os << indent << "Two Sided Lighting: " << (this->TwoSidedLighting ? "On\n" : "Off\n");

  os << indent << "Automatic Light Creation: " << (this->AutomaticLightCreation ? "On\n" : "Off\n");

  os << indent << "Layer = " << this->Layer << "\n";
  os << indent << "PreserveDepthBuffer: " << (this->PreserveDepthBuffer ? "On" : "Off") << "\n";
  os << indent << "PreserveColorBuffer: " << (this->PreserveColorBuffer ? "On" : "Off") << "\n";
  os << indent << "Interactive = " << (this->Interactive ? "On" : "Off") << "\n";

  os << indent << "Allocated Render Time: " << this->AllocatedRenderTime << "\n";

  os << indent << "Last Time To Render (Seconds): " << this->LastRenderTimeInSeconds << endl;
  os << indent << "TimeFactor: " << this->TimeFactor << endl;

  os << indent << "Erase: " << (this->Erase ? "On\n" : "Off\n");

  os << indent << "Draw: " << (this->Draw ? "On\n" : "Off\n");

  os << indent << "UseDepthPeeling: " << (this->UseDepthPeeling ? "On" : "Off") << "\n";

  os << indent << "OcclusionRation: " << this->OcclusionRatio << "\n";

  os << indent << "MaximumNumberOfPeels: " << this->MaximumNumberOfPeels << "\n";

  os << indent
     << "LastRenderingUsedDepthPeeling: " << (this->LastRenderingUsedDepthPeeling ? "On" : "Off")
     << "\n";

  // I don't want to print this since it is used just internally
  // os << indent << this->NumberOfPropsRendered;

  os << indent << "Delegate:";
  if (this->Delegate != nullptr)
  {
    os << "exists" << endl;
  }
  else
  {
    os << "null" << endl;
  }
  os << indent << "Selector: " << this->Selector << endl;

  os << indent << "TexturedBackground: " << (this->TexturedBackground ? "On" : "Off") << "\n";

  os << indent << "BackgroundTexture:";
  if (this->BackgroundTexture != nullptr)
  {
    os << "exists" << endl;
  }
  else
  {
    os << "null" << endl;
  }

  os << indent << "RightBackgroundTexture:";
  if (this->RightBackgroundTexture != nullptr)
  {
    os << "exists" << endl;
  }
  else
  {
    os << "null" << endl;
  }

  os << indent << "Pass:";
  if (this->Pass != nullptr)
  {
    os << "exists" << endl;
  }
  else
  {
    os << "null" << endl;
  }
}

int vtkRenderer::VisibleActorCount()
{
  vtkProp* aProp;
  int count = 0;

  // loop through Props
  vtkCollectionSimpleIterator pit;
  for (this->Props->InitTraversal(pit); (aProp = this->Props->GetNextProp(pit));)
  {
    if (aProp->GetVisibility())
    {
      count++;
    }
  }
  return count;
}

int vtkRenderer::VisibleVolumeCount()
{
  int count = 0;
  vtkProp* aProp;

  // loop through volumes
  vtkCollectionSimpleIterator pit;
  for (this->Props->InitTraversal(pit); (aProp = this->Props->GetNextProp(pit));)
  {
    if (aProp->GetVisibility())
    {
      count++;
    }
  }
  return count;
}

vtkMTimeType vtkRenderer::GetMTime()
{
  vtkMTimeType mTime = this->vtkViewport::GetMTime();
  vtkMTimeType time;

  if (this->ActiveCamera != nullptr)
  {
    time = this->ActiveCamera->GetMTime();
    mTime = (time > mTime ? time : mTime);
  }
  if (this->CreatedLight != nullptr)
  {
    time = this->CreatedLight->GetMTime();
    mTime = (time > mTime ? time : mTime);
  }

  return mTime;
}

vtkAssemblyPath* vtkRenderer::PickProp(
  double selectionX1, double selectionY1, double selectionX2, double selectionY2)
{
  // Get the pick id of the object that was picked
  if (this->PickedProp != nullptr)
  {
    this->PickedProp->UnRegister(this);
    this->PickedProp = nullptr;
  }
  if (this->PickResultProps != nullptr)
  {
    this->PickResultProps->Delete();
    this->PickResultProps = nullptr;
  }

  this->PickX1 = (selectionX1 < selectionX2) ? selectionX1 : selectionX2;
  this->PickY1 = (selectionY1 < selectionY2) ? selectionY1 : selectionY2;
  this->PickX2 = (selectionX1 > selectionX2) ? selectionX1 : selectionX2;
  this->PickY2 = (selectionY1 > selectionY2) ? selectionY1 : selectionY2;

  // Do not let pick area go outside the viewport
  int lowerLeft[2];
  int usize, vsize;
  this->GetTiledSizeAndOrigin(&usize, &vsize, lowerLeft, lowerLeft + 1);
  if (this->PickX1 < lowerLeft[0])
  {
    this->PickX1 = lowerLeft[0];
  }
  if (this->PickY1 < lowerLeft[1])
  {
    this->PickY1 = lowerLeft[1];
  }
  if (this->PickX2 >= lowerLeft[0] + usize)
  {
    this->PickX2 = lowerLeft[0] + usize - 1;
  }
  if (this->PickY2 >= lowerLeft[1] + vsize)
  {
    this->PickY2 = lowerLeft[1] + vsize - 1;
  }

  // if degenerate then return nullptr
  if (this->PickX1 > this->PickX2 || this->PickY1 > this->PickY2)
  {
    return nullptr;
  }

  // use a hardware selector since we have it
  vtkNew<vtkHardwareSelector> hsel;
  hsel->SetActorPassOnly(true);
  hsel->SetCaptureZValues(true);
  hsel->SetRenderer(this);
  hsel->SetArea(this->PickX1, this->PickY1, this->PickX2, this->PickY2);
  vtkSmartPointer<vtkSelection> sel;
  sel.TakeReference(hsel->Select());

  if (sel && sel->GetNode(0))
  {
    // find the node with the closest zvalue and
    // store the list of picked props
    vtkProp* closestProp = nullptr;
    double closestDepth = 2.0;
    this->PickResultProps = vtkPropCollection::New();
    unsigned int numPicked = sel->GetNumberOfNodes();
    for (unsigned int pIdx = 0; pIdx < numPicked; pIdx++)
    {
      vtkSelectionNode* selnode = sel->GetNode(pIdx);
      vtkProp* aProp =
        vtkProp::SafeDownCast(selnode->GetProperties()->Get(vtkSelectionNode::PROP()));
      if (aProp)
      {
        this->PickResultProps->AddItem(aProp);
        double adepth = selnode->GetProperties()->Get(vtkSelectionNode::ZBUFFER_VALUE());
        if (adepth < closestDepth)
        {
          closestProp = aProp;
          closestDepth = adepth;
        }
      }
    }
    if (closestProp == nullptr)
    {
      return nullptr;
    }
    closestProp->InitPathTraversal();
    this->PickedProp = closestProp->GetNextPath();
    this->PickedProp->Register(this);
    this->PickedZ = closestDepth;
  }

  // Return the pick!
  return this->PickedProp; // returns an assembly path
}

//------------------------------------------------------------------------------
void vtkRenderer::SetEnvironmentTexture(vtkTexture* texture, bool vtkNotUsed(isSRGB))
{
  vtkSetObjectBodyMacro(EnvironmentTexture, vtkTexture, texture);
}

//------------------------------------------------------------------------------
void vtkRenderer::ExpandBounds(double bounds[6], vtkMatrix4x4* matrix)
{
  if (!bounds)
  {
    vtkErrorMacro(<< "ERROR: Invalid bounds\n");
    return;
  }

  if (!matrix)
  {
    vtkErrorMacro("<<ERROR: Invalid matrix \n");
    return;
  }

  // Expand the bounding box by model view transform matrix.
  double pt[8][4] = { { bounds[0], bounds[2], bounds[5], 1.0 },
    { bounds[1], bounds[2], bounds[5], 1.0 }, { bounds[1], bounds[2], bounds[4], 1.0 },
    { bounds[0], bounds[2], bounds[4], 1.0 }, { bounds[0], bounds[3], bounds[5], 1.0 },
    { bounds[1], bounds[3], bounds[5], 1.0 }, { bounds[1], bounds[3], bounds[4], 1.0 },
    { bounds[0], bounds[3], bounds[4], 1.0 } };

  // \note: Assuming that matrix doesn not have projective component. Hence not
  // dividing by the homogeneous coordinate after multiplication
  for (int i = 0; i < 8; ++i)
  {
    matrix->MultiplyPoint(pt[i], pt[i]);
  }

  // min = mpx = pt[0]
  double min[4], max[4];
  for (int i = 0; i < 4; ++i)
  {
    min[i] = pt[0][i];
    max[i] = pt[0][i];
  }

  for (int i = 1; i < 8; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      if (min[j] > pt[i][j])
        min[j] = pt[i][j];
      if (max[j] < pt[i][j])
        max[j] = pt[i][j];
    }
  }

  // Copy values back to bounds.
  bounds[0] = min[0];
  bounds[2] = min[1];
  bounds[4] = min[2];

  bounds[1] = max[0];
  bounds[3] = max[1];
  bounds[5] = max[2];
}

int vtkRenderer::Transparent()
{
  return this->PreserveColorBuffer;
}

double vtkRenderer::GetTiledAspectRatio()
{
  int usize, vsize;
  this->GetTiledSize(&usize, &vsize);

  // some renderer subclasses may have more complicated computations for the
  // aspect ratio. SO take that into account by computing the difference
  // between our simple aspect ratio and what the actual renderer is
  // reporting.
  double aspect[2];
  this->ComputeAspect();
  this->GetAspect(aspect);
  double aspect2[2];
  this->vtkViewport::ComputeAspect();
  this->vtkViewport::GetAspect(aspect2);
  double aspectModification = aspect[0] * aspect2[1] / (aspect[1] * aspect2[0]);

  double finalAspect = 1.0;
  if (vsize && usize)
  {
    finalAspect = aspectModification * usize / vsize;
  }
  return finalAspect;
}

int vtkRenderer::CaptureGL2PSSpecialProp(vtkProp* prop)
{
  if (this->GL2PSSpecialPropCollection && !this->GL2PSSpecialPropCollection->IsItemPresent(prop))
  {
    this->GL2PSSpecialPropCollection->AddItem(prop);
    return 1;
  }

  return 0;
}

vtkCxxSetObjectMacro(vtkRenderer, GL2PSSpecialPropCollection, vtkPropCollection);

const std::array<double, 16>& vtkRenderer::GetViewTransformMatrix()
{
  if (this->LastViewTransformCameraModified != this->ActiveCamera->GetMTime())
  {
    vtkMatrix4x4::DeepCopy(
      this->ViewTransformMatrix.data(), this->ActiveCamera->GetViewTransformMatrix());

    this->LastViewTransformCameraModified = this->ActiveCamera->GetMTime();
  }
  return this->ViewTransformMatrix;
}

const std::array<double, 16>& vtkRenderer::GetCompositeProjectionTransformationMatrix()
{
  const double tiledAspectRatio = this->GetTiledAspectRatio();
  if (tiledAspectRatio != this->LastCompositeProjectionTransformationMatrixTiledAspectRatio ||
    this->LastCompositeProjectionTransformationMatrixCameraModified !=
      this->ActiveCamera->GetMTime())
  {
    vtkMatrix4x4::DeepCopy(this->CompositeProjectionTransformationMatrix.data(),
      this->ActiveCamera->GetCompositeProjectionTransformMatrix(tiledAspectRatio, 0, 1));

    this->LastCompositeProjectionTransformationMatrixTiledAspectRatio = tiledAspectRatio;
    this->LastCompositeProjectionTransformationMatrixCameraModified =
      this->ActiveCamera->GetMTime();
  }
  return this->CompositeProjectionTransformationMatrix;
}

const std::array<double, 16>& vtkRenderer::GetProjectionTransformationMatrix()
{
  const double tiledAspectRatio = this->GetTiledAspectRatio();
  if (tiledAspectRatio != this->LastProjectionTransformationMatrixTiledAspectRatio ||
    this->LastProjectionTransformationMatrixCameraModified != this->ActiveCamera->GetMTime())
  {
    vtkMatrix4x4::DeepCopy(this->ProjectionTransformationMatrix.data(),
      this->ActiveCamera->GetProjectionTransformMatrix(tiledAspectRatio, 0, 1));

    this->LastProjectionTransformationMatrixTiledAspectRatio = tiledAspectRatio;
    this->LastProjectionTransformationMatrixCameraModified = this->ActiveCamera->GetMTime();
  }
  return this->ProjectionTransformationMatrix;
}
