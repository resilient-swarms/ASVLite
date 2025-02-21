/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkInteractorStyle.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkInteractorStyle.h"

#include "vtkActor.h"
#include "vtkActor2D.h"
#include "vtkAssemblyNode.h"
#include "vtkAssemblyPath.h"
#include "vtkCallbackCommand.h"
#include "vtkCellPicker.h"
#include "vtkCommand.h"
#include "vtkEventForwarderCommand.h"
#include "vtkMath.h"
#include "vtkObjectFactory.h"
#include "vtkOutlineSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkProperty2D.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkTDxInteractorStyleCamera.h"

vtkStandardNewMacro(vtkInteractorStyle);
vtkCxxSetObjectMacro(vtkInteractorStyle, TDxStyle, vtkTDxInteractorStyle);

//------------------------------------------------------------------------------
vtkInteractorStyle::vtkInteractorStyle()
{
  this->State = VTKIS_NONE;
  this->AnimState = VTKIS_ANIM_OFF;

  this->HandleObservers = 1;
  this->UseTimers = 0;
  this->TimerId = 1;

  this->AutoAdjustCameraClippingRange = 1;

  this->Interactor = nullptr;

  this->EventCallbackCommand->SetCallback(vtkInteractorStyle::ProcessEvents);

  // These widgets are not activated with a key

  this->KeyPressActivation = 0;

  this->Outline = vtkOutlineSource::New();
  this->OutlineActor = nullptr;
  this->OutlineMapper = vtkPolyDataMapper::New();

  if (this->OutlineMapper && this->Outline)
  {
    this->OutlineMapper->SetInputConnection(this->Outline->GetOutputPort());
  }

  this->PickedRenderer = nullptr;
  this->CurrentProp = nullptr;
  this->PropPicked = 0;

  this->PickColor[0] = 1.0;
  this->PickColor[1] = 0.0;
  this->PickColor[2] = 0.0;
  this->PickedActor2D = nullptr;

  this->MouseWheelMotionFactor = 1.0;

  this->TimerDuration = 10;
  this->EventForwarder = vtkEventForwarderCommand::New();

  this->TDxStyle = vtkTDxInteractorStyleCamera::New();
}

//------------------------------------------------------------------------------
vtkInteractorStyle::~vtkInteractorStyle()
{
  // Remove observers

  this->SetInteractor(nullptr);

  // Remove any highlight

  this->HighlightProp(nullptr);

  if (this->OutlineActor)
  {
    this->OutlineActor->Delete();
  }

  if (this->OutlineMapper)
  {
    this->OutlineMapper->Delete();
  }

  this->Outline->Delete();
  this->Outline = nullptr;

  this->SetCurrentRenderer(nullptr);
  this->EventForwarder->Delete();

  if (this->TDxStyle != nullptr)
  {
    this->TDxStyle->Delete();
  }
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::SetEnabled(int enabling)
{
  if (!this->Interactor)
  {
    vtkErrorMacro(<< "The interactor must be set prior to enabling/disabling widget");
    return;
  }

  if (enabling) //----------------------------------------------------------
  {
    vtkDebugMacro(<< "Enabling widget");

    if (this->Enabled) // already enabled, just return
    {
      return;
    }

    this->Enabled = 1;
    this->InvokeEvent(vtkCommand::EnableEvent, nullptr);
  }

  else // disabling-------------------------------------------------------------
  {
    vtkDebugMacro(<< "Disabling widget");

    if (!this->Enabled) // already disabled, just return
    {
      return;
    }

    this->Enabled = 0;
    this->HighlightProp(nullptr);
    this->InvokeEvent(vtkCommand::DisableEvent, nullptr);
  }
}

//------------------------------------------------------------------------------
// NOTE!!! This does not do any reference counting!!!
// This is to avoid some ugly reference counting loops
// and the benefit of being able to hold only an entire
// renderwindow from an interactor style doesn't seem worth the
// mess.   Instead the vtkInteractorStyle sets up a DeleteEvent callback, so
// that it can tell when the vtkRenderWindowInteractor is going away.

void vtkInteractorStyle::SetInteractor(vtkRenderWindowInteractor* i)
{
  if (i == this->Interactor)
  {
    return;
  }

  // if we already have an Interactor then stop observing it
  if (this->Interactor)
  {
    this->Interactor->RemoveObserver(this->EventCallbackCommand);
  }
  this->Interactor = i;

  // add observers for each of the events handled in ProcessEvents
  if (i)
  {
    i->AddObserver(vtkCommand::EnterEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::LeaveEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::MouseMoveEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::LeftButtonPressEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::LeftButtonReleaseEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(
      vtkCommand::LeftButtonDoubleClickEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::MiddleButtonPressEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(
      vtkCommand::MiddleButtonReleaseEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(
      vtkCommand::MiddleButtonDoubleClickEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::RightButtonPressEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::RightButtonReleaseEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(
      vtkCommand::RightButtonDoubleClickEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::MouseWheelForwardEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::MouseWheelBackwardEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::MouseWheelLeftEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::MouseWheelRightEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::ExposeEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::ConfigureEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::TimerEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::KeyPressEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::KeyReleaseEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::CharEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::DeleteEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::TDxMotionEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::TDxButtonPressEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::TDxButtonReleaseEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::StartSwipeEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::SwipeEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::EndSwipeEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::StartPinchEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::PinchEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::EndPinchEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::StartRotateEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::RotateEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::EndRotateEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::StartPanEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::PanEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::EndPanEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::TapEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::LongTapEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::FourthButtonPressEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(
      vtkCommand::FourthButtonReleaseEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::FifthButtonPressEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::FifthButtonReleaseEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::Move3DEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::Button3DEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::ViewerMovement3DEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::Select3DEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::Clip3DEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::NextPose3DEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::PositionProp3DEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::Pick3DEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::Menu3DEvent, this->EventCallbackCommand, this->Priority);

    i->AddObserver(vtkCommand::DropFilesEvent, this->EventCallbackCommand, this->Priority);
    i->AddObserver(vtkCommand::UpdateDropLocationEvent, this->EventCallbackCommand, this->Priority);
  }

  this->EventForwarder->SetTarget(this->Interactor);
  if (this->Interactor)
  {
    this->AddObserver(vtkCommand::StartInteractionEvent, this->EventForwarder);
    this->AddObserver(vtkCommand::InteractionEvent, this->EventForwarder);
    this->AddObserver(vtkCommand::EndInteractionEvent, this->EventForwarder);
  }
  else
  {
    this->RemoveObserver(this->EventForwarder);
  }
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::FindPokedRenderer(int x, int y)
{
  this->SetCurrentRenderer(this->Interactor->FindPokedRenderer(x, y));
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::HighlightProp(vtkProp* prop)
{
  this->CurrentProp = prop;

  if (prop != nullptr)
  {
    vtkActor2D* actor2D;
    vtkProp3D* prop3D;
    if ((prop3D = vtkProp3D::SafeDownCast(prop)) != nullptr)
    {
      this->HighlightProp3D(prop3D);
    }
    else if ((actor2D = vtkActor2D::SafeDownCast(prop)) != nullptr)
    {
      this->HighlightActor2D(actor2D);
    }
  }
  else
  { // unhighlight everything, both 2D & 3D
    this->HighlightProp3D(nullptr);
    this->HighlightActor2D(nullptr);
  }

  if (this->Interactor)
  {
    this->Interactor->Render();
  }
}

//------------------------------------------------------------------------------
// When pick action successfully selects a vtkProp3Dactor, this method
// highlights the vtkProp3D appropriately. Currently this is done by placing a
// bounding box around the vtkProp3D.
void vtkInteractorStyle::HighlightProp3D(vtkProp3D* prop3D)
{
  // no prop picked now
  if (!prop3D)
  {
    // was there previously?
    if (this->PickedRenderer != nullptr && this->OutlineActor)
    {
      this->PickedRenderer->RemoveActor(this->OutlineActor);
      this->PickedRenderer = nullptr;
    }
  }
  // prop picked now
  else
  {
    if (!this->OutlineActor)
    {
      // have to defer creation to get right type
      this->OutlineActor = vtkActor::New();
      this->OutlineActor->PickableOff();
      this->OutlineActor->DragableOff();
      this->OutlineActor->SetMapper(this->OutlineMapper);
      this->OutlineActor->GetProperty()->SetColor(this->PickColor);
      this->OutlineActor->GetProperty()->SetAmbient(1.0);
      this->OutlineActor->GetProperty()->SetDiffuse(0.0);
    }

    // check if picked in different renderer to previous pick
    if (this->CurrentRenderer != this->PickedRenderer)
    {
      if (this->PickedRenderer != nullptr && this->OutlineActor)
      {
        this->PickedRenderer->RemoveActor(this->OutlineActor);
      }
      if (this->CurrentRenderer != nullptr)
      {
        this->CurrentRenderer->AddActor(this->OutlineActor);
      }
      else
      {
        vtkWarningMacro(<< "no current renderer on the interactor style.");
      }
      this->PickedRenderer = this->CurrentRenderer;
    }
    this->Outline->SetBounds(prop3D->GetBounds());
  }
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::HighlightActor2D(vtkActor2D* actor2D)
{
  // If nothing has changed, just return
  if (actor2D == this->PickedActor2D)
  {
    return;
  }

  if (actor2D)
  {
    double tmpColor[3];
    actor2D->GetProperty()->GetColor(tmpColor);

    if (this->PickedActor2D)
    {
      actor2D->GetProperty()->SetColor(this->PickedActor2D->GetProperty()->GetColor());
      this->PickedActor2D->GetProperty()->SetColor(this->PickColor);
    }
    else
    {
      actor2D->GetProperty()->SetColor(this->PickColor);
    }

    this->PickColor[0] = tmpColor[0];
    this->PickColor[1] = tmpColor[1];
    this->PickColor[2] = tmpColor[2];
  }
  else
  {
    if (this->PickedActor2D)
    {
      double tmpColor[3];
      this->PickedActor2D->GetProperty()->GetColor(tmpColor);
      this->PickedActor2D->GetProperty()->SetColor(this->PickColor);
      this->PickColor[0] = tmpColor[0];
      this->PickColor[1] = tmpColor[1];
      this->PickColor[2] = tmpColor[2];
    }
  }

  this->PickedActor2D = actor2D;
}

//------------------------------------------------------------------------------
// Implementation of motion state control methods
//------------------------------------------------------------------------------
void vtkInteractorStyle::StartState(int newstate)
{
  this->State = newstate;
  if (this->AnimState == VTKIS_ANIM_OFF)
  {
    vtkRenderWindowInteractor* rwi = this->Interactor;
    rwi->GetRenderWindow()->SetDesiredUpdateRate(rwi->GetDesiredUpdateRate());
    this->InvokeEvent(vtkCommand::StartInteractionEvent, nullptr);
    if (this->UseTimers && !(this->TimerId = rwi->CreateRepeatingTimer(this->TimerDuration)))
    {
      // FIXME: This comment doesn't match the logic of the code. There is one
      // test failing with it like it is, but more failing if the comparison is
      // inverted.
      // vtkTestingInteractor cannot create timers
      if (std::string(rwi->GetClassName()) != "vtkTestingInteractor")
      {
        vtkErrorMacro(<< "Timer start failed");
      }
      this->State = VTKIS_NONE;
    }
  }
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StopState()
{
  this->State = VTKIS_NONE;
  if (this->AnimState == VTKIS_ANIM_OFF)
  {
    vtkRenderWindowInteractor* rwi = this->Interactor;
    vtkRenderWindow* renwin = rwi->GetRenderWindow();
    renwin->SetDesiredUpdateRate(rwi->GetStillUpdateRate());
    if (this->UseTimers &&
      // FIXME: This comment doesn't match the logic of the code. There is
      // one test failing with it like it is, but more failing if the
      // comparison is inverted.
      // vtkTestingInteractor cannot create timers
      std::string(rwi->GetClassName()) != "vtkTestingInteractor" &&
      !rwi->DestroyTimer(this->TimerId))
    {
      vtkErrorMacro(<< "Timer stop failed");
    }
    this->InvokeEvent(vtkCommand::EndInteractionEvent, nullptr);
    rwi->Render();
  }
}

//------------------------------------------------------------------------------
// JCP animation control
void vtkInteractorStyle::StartAnimate()
{
  vtkRenderWindowInteractor* rwi = this->Interactor;
  this->AnimState = VTKIS_ANIM_ON;
  if (this->State == VTKIS_NONE)
  {
    rwi->GetRenderWindow()->SetDesiredUpdateRate(rwi->GetDesiredUpdateRate());
    if (this->UseTimers && !(this->TimerId = rwi->CreateRepeatingTimer(this->TimerDuration)))
    {
      vtkErrorMacro(<< "Timer start failed");
    }
  }
  rwi->Render();
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StopAnimate()
{
  vtkRenderWindowInteractor* rwi = this->Interactor;
  this->AnimState = VTKIS_ANIM_OFF;
  if (this->State == VTKIS_NONE)
  {
    rwi->GetRenderWindow()->SetDesiredUpdateRate(rwi->GetStillUpdateRate());
    if (this->UseTimers && !rwi->DestroyTimer(this->TimerId))
    {
      vtkErrorMacro(<< "Timer stop failed");
    }
  }
}

// JCP Animation control
//------------------------------------------------------------------------------
void vtkInteractorStyle::StartRotate()
{
  if (this->State != VTKIS_NONE)
  {
    return;
  }
  this->StartState(VTKIS_ROTATE);
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::EndRotate()
{
  if (this->State != VTKIS_ROTATE)
  {
    return;
  }
  this->StopState();
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StartZoom()
{
  if (this->State != VTKIS_NONE)
  {
    return;
  }
  this->StartState(VTKIS_ZOOM);
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::EndZoom()
{
  if (this->State != VTKIS_ZOOM)
  {
    return;
  }
  this->StopState();
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StartPan()
{
  if (this->State != VTKIS_NONE)
  {
    return;
  }
  this->StartState(VTKIS_PAN);
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::EndPan()
{
  if (this->State != VTKIS_PAN)
  {
    return;
  }
  this->StopState();
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StartSpin()
{
  if (this->State != VTKIS_NONE)
  {
    return;
  }
  this->StartState(VTKIS_SPIN);
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::EndSpin()
{
  if (this->State != VTKIS_SPIN)
  {
    return;
  }
  this->StopState();
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StartDolly()
{
  if (this->State != VTKIS_NONE)
  {
    return;
  }
  this->StartState(VTKIS_DOLLY);
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::EndDolly()
{
  if (this->State != VTKIS_DOLLY)
  {
    return;
  }
  this->StopState();
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StartUniformScale()
{
  if (this->State != VTKIS_NONE)
  {
    return;
  }
  this->StartState(VTKIS_USCALE);
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::EndUniformScale()
{
  if (this->State != VTKIS_USCALE)
  {
    return;
  }
  this->StopState();
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StartTimer()
{
  if (this->State != VTKIS_NONE)
  {
    return;
  }
  this->StartState(VTKIS_TIMER);
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::EndTimer()
{
  if (this->State != VTKIS_TIMER)
  {
    return;
  }
  this->StopState();
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StartTwoPointer()
{
  if (this->State != VTKIS_NONE)
  {
    return;
  }
  this->StartState(VTKIS_TWO_POINTER);
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::EndTwoPointer()
{
  if (this->State != VTKIS_TWO_POINTER)
  {
    return;
  }
  this->StopState();
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StartGesture()
{
  if (this->State != VTKIS_NONE)
  {
    return;
  }
  this->StartState(VTKIS_GESTURE);
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::EndGesture()
{
  if (this->State != VTKIS_GESTURE)
  {
    return;
  }
  this->StopState();
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::StartEnvRotate()
{
  if (this->State != VTKIS_NONE)
  {
    return;
  }
  this->StartState(VTKIS_ENV_ROTATE);
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::EndEnvRotate()
{
  if (this->State != VTKIS_ENV_ROTATE)
  {
    return;
  }
  this->StopState();
}

//------------------------------------------------------------------------------
// By overriding the Rotate, Rotate members we can
// use this timer routine for Joystick or Trackball - quite tidy
//------------------------------------------------------------------------------
void vtkInteractorStyle::OnTimer()
{
  vtkRenderWindowInteractor* rwi = this->Interactor;

  switch (this->State)
  {
    case VTKIS_NONE:
      if (this->AnimState == VTKIS_ANIM_ON)
      {
        if (this->UseTimers)
        {
          rwi->DestroyTimer(this->TimerId);
        }
        rwi->Render();
        if (this->UseTimers)
        {
          this->TimerId = rwi->CreateRepeatingTimer(this->TimerDuration);
        }
      }
      break;

    case VTKIS_ROTATE:
      this->Rotate();
      break;

    case VTKIS_PAN:
      this->Pan();
      break;

    case VTKIS_SPIN:
      this->Spin();
      break;

    case VTKIS_DOLLY:
      this->Dolly();
      break;

    case VTKIS_ZOOM:
      this->Zoom();
      break;

    case VTKIS_USCALE:
      this->UniformScale();
      break;

    case VTKIS_ENV_ROTATE:
      this->EnvironmentRotate();
      break;

    case VTKIS_TIMER:
      rwi->Render();
      break;

    default:
      break;
  }
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::OnChar()
{
  vtkRenderWindowInteractor* rwi = this->Interactor;

  switch (rwi->GetKeyCode())
  {
    case 'm':
    case 'M':
      if (this->AnimState == VTKIS_ANIM_OFF)
      {
        this->StartAnimate();
      }
      else
      {
        this->StopAnimate();
      }
      break;

    case 'Q':
    case 'q':
    case 'e':
    case 'E':
      rwi->ExitCallback();
      break;

    case 'f':
    case 'F':
    {
      if (this->CurrentRenderer != nullptr)
      {
        this->AnimState = VTKIS_ANIM_ON;
        vtkAssemblyPath* path = nullptr;
        this->FindPokedRenderer(rwi->GetEventPosition()[0], rwi->GetEventPosition()[1]);
        rwi->GetPicker()->Pick(
          rwi->GetEventPosition()[0], rwi->GetEventPosition()[1], 0.0, this->CurrentRenderer);
        vtkAbstractPropPicker* picker;
        if ((picker = vtkAbstractPropPicker::SafeDownCast(rwi->GetPicker())))
        {
          path = picker->GetPath();
        }
        if (path != nullptr)
        {
          rwi->FlyTo(this->CurrentRenderer, picker->GetPickPosition());
        }
        this->AnimState = VTKIS_ANIM_OFF;
      }
      else
      {
        vtkWarningMacro(<< "no current renderer on the interactor style.");
      }
    }
    break;

    case 'u':
    case 'U':
      rwi->UserCallback();
      break;

    case 'r':
    case 'R':
      this->FindPokedRenderer(rwi->GetEventPosition()[0], rwi->GetEventPosition()[1]);
      if (this->CurrentRenderer != nullptr)
      {
        this->CurrentRenderer->ResetCamera();
      }
      else
      {
        vtkWarningMacro(<< "no current renderer on the interactor style.");
      }
      rwi->Render();
      break;

    case 'w':
    case 'W':
    {
      vtkActorCollection* ac;
      vtkActor *anActor, *aPart;
      vtkAssemblyPath* path;
      this->FindPokedRenderer(rwi->GetEventPosition()[0], rwi->GetEventPosition()[1]);
      if (this->CurrentRenderer != nullptr)
      {
        ac = this->CurrentRenderer->GetActors();
        vtkCollectionSimpleIterator ait;
        for (ac->InitTraversal(ait); (anActor = ac->GetNextActor(ait));)
        {
          for (anActor->InitPathTraversal(); (path = anActor->GetNextPath());)
          {
            aPart = static_cast<vtkActor*>(path->GetLastNode()->GetViewProp());
            aPart->GetProperty()->SetRepresentationToWireframe();
          }
        }
      }
      else
      {
        vtkWarningMacro(<< "no current renderer on the interactor style.");
      }
      rwi->Render();
    }
    break;

    case 's':
    case 'S':
    {
      vtkActorCollection* ac;
      vtkActor *anActor, *aPart;
      vtkAssemblyPath* path;
      this->FindPokedRenderer(rwi->GetEventPosition()[0], rwi->GetEventPosition()[1]);
      if (this->CurrentRenderer != nullptr)
      {
        ac = this->CurrentRenderer->GetActors();
        vtkCollectionSimpleIterator ait;
        for (ac->InitTraversal(ait); (anActor = ac->GetNextActor(ait));)
        {
          for (anActor->InitPathTraversal(); (path = anActor->GetNextPath());)
          {
            aPart = static_cast<vtkActor*>(path->GetLastNode()->GetViewProp());
            aPart->GetProperty()->SetRepresentationToSurface();
          }
        }
      }
      else
      {
        vtkWarningMacro(<< "no current renderer on the interactor style.");
      }
      rwi->Render();
    }
    break;

    case '3':
      if (rwi->GetRenderWindow()->GetStereoRender())
      {
        rwi->GetRenderWindow()->StereoRenderOff();
      }
      else
      {
        rwi->GetRenderWindow()->StereoRenderOn();
      }
      rwi->Render();
      break;

    case 'p':
    case 'P':
      if (this->CurrentRenderer != nullptr)
      {
        if (this->State == VTKIS_NONE)
        {
          vtkAssemblyPath* path = nullptr;
          int* eventPos = rwi->GetEventPosition();
          this->FindPokedRenderer(eventPos[0], eventPos[1]);
          rwi->StartPickCallback();
          vtkAbstractPropPicker* picker = vtkAbstractPropPicker::SafeDownCast(rwi->GetPicker());
          if (picker != nullptr)
          {
            picker->Pick(eventPos[0], eventPos[1], 0.0, this->CurrentRenderer);
            path = picker->GetPath();
          }
          if (path == nullptr)
          {
            this->HighlightProp(nullptr);
            this->PropPicked = 0;
          }
          else
          {
            this->HighlightProp(path->GetFirstNode()->GetViewProp());
            this->PropPicked = 1;
          }
          rwi->EndPickCallback();
        }
      }
      else
      {
        vtkWarningMacro(<< "no current renderer on the interactor style.");
      }
      break;
  }
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Auto Adjust Camera Clipping Range "
     << (this->AutoAdjustCameraClippingRange ? "On\n" : "Off\n");

  os << indent << "Pick Color: (" << this->PickColor[0] << ", " << this->PickColor[1] << ", "
     << this->PickColor[2] << ")\n";

  os << indent << "CurrentRenderer: " << this->CurrentRenderer << "\n";
  if (this->PickedRenderer)
  {
    os << indent << "Picked Renderer: " << this->PickedRenderer << "\n";
  }
  else
  {
    os << indent << "Picked Renderer: (none)\n";
  }
  if (this->CurrentProp)
  {
    os << indent << "Current Prop: " << this->CurrentProp << "\n";
  }
  else
  {
    os << indent << "Current Actor: (none)\n";
  }

  os << indent << "Interactor: " << this->Interactor << "\n";
  os << indent << "Prop Picked: " << (this->PropPicked ? "Yes\n" : "No\n");

  os << indent << "State: " << this->State << endl;
  os << indent << "UseTimers: " << this->UseTimers << endl;
  os << indent << "HandleObservers: " << this->HandleObservers << endl;
  os << indent << "MouseWheelMotionFactor: " << this->MouseWheelMotionFactor << endl;

  os << indent << "Timer Duration: " << this->TimerDuration << endl;

  os << indent << "TDxStyle: ";
  if (this->TDxStyle == nullptr)
  {
    os << "(none)" << endl;
  }
  else
  {
    this->TDxStyle->PrintSelf(os, indent.GetNextIndent());
  }
}

//------------------------------------------------------------------------------
void vtkInteractorStyle::DelegateTDxEvent(unsigned long event, void* calldata)
{
  if (this->TDxStyle != nullptr)
  {
    this->TDxStyle->ProcessEvent(this->CurrentRenderer, event, calldata);
  }
}

#define vtkISEventDataMacro(eventname)                                                             \
  case vtkCommand::eventname##Event:                                                               \
    if (self->HandleObservers && self->HasObserver(vtkCommand::eventname##Event))                  \
    {                                                                                              \
      aborted = (self->InvokeEvent(vtkCommand::eventname##Event, calldata) == 1);                  \
    }                                                                                              \
    if (!aborted)                                                                                  \
    {                                                                                              \
      self->On##eventname(static_cast<vtkEventData*>(calldata));                                   \
    }                                                                                              \
    break

//------------------------------------------------------------------------------
void vtkInteractorStyle::ProcessEvents(
  vtkObject* vtkNotUsed(object), unsigned long event, void* clientdata, void* calldata)
{
  vtkInteractorStyle* self = reinterpret_cast<vtkInteractorStyle*>(clientdata);

  bool aborted = false;

  switch (event)
  {
    case vtkCommand::ExposeEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::ExposeEvent))
      {
        self->InvokeEvent(vtkCommand::ExposeEvent, nullptr);
      }
      else
      {
        self->OnExpose();
      }
      break;

    case vtkCommand::ConfigureEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::ConfigureEvent))
      {
        self->InvokeEvent(vtkCommand::ConfigureEvent, nullptr);
      }
      else
      {
        self->OnConfigure();
      }
      break;

    case vtkCommand::EnterEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::EnterEvent))
      {
        self->InvokeEvent(vtkCommand::EnterEvent, nullptr);
      }
      else
      {
        self->OnEnter();
      }
      break;

    case vtkCommand::LeaveEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::LeaveEvent))
      {
        self->InvokeEvent(vtkCommand::LeaveEvent, nullptr);
      }
      else
      {
        self->OnLeave();
      }
      break;

    case vtkCommand::TimerEvent:
    {
      // The calldata should be a timer id, but because of legacy we check
      // and make sure that it is non-nullptr.
      int timerId = (calldata ? *(reinterpret_cast<int*>(calldata)) : 1);
      if (self->HandleObservers && self->HasObserver(vtkCommand::TimerEvent))
      {
        self->InvokeEvent(vtkCommand::TimerEvent, &timerId);
      }
      else
      {
        self->OnTimer();
      }
    }
    break;

    case vtkCommand::MouseMoveEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::MouseMoveEvent))
      {
        self->InvokeEvent(vtkCommand::MouseMoveEvent, nullptr);
      }
      else
      {
        self->OnMouseMove();
      }
      break;

    case vtkCommand::LeftButtonPressEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::LeftButtonPressEvent))
      {
        self->InvokeEvent(vtkCommand::LeftButtonPressEvent, nullptr);
      }
      else
      {
        self->OnLeftButtonDown();
      }
      break;

    case vtkCommand::LeftButtonReleaseEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::LeftButtonReleaseEvent))
      {
        self->InvokeEvent(vtkCommand::LeftButtonReleaseEvent, nullptr);
      }
      else
      {
        self->OnLeftButtonUp();
      }
      break;

    case vtkCommand::MiddleButtonPressEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::MiddleButtonPressEvent))
      {
        self->InvokeEvent(vtkCommand::MiddleButtonPressEvent, nullptr);
      }
      else
      {
        self->OnMiddleButtonDown();
      }
      break;

    case vtkCommand::MiddleButtonReleaseEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::MiddleButtonReleaseEvent))
      {
        self->InvokeEvent(vtkCommand::MiddleButtonReleaseEvent, nullptr);
      }
      else
      {
        self->OnMiddleButtonUp();
      }
      break;

    case vtkCommand::RightButtonPressEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::RightButtonPressEvent))
      {
        self->InvokeEvent(vtkCommand::RightButtonPressEvent, nullptr);
      }
      else
      {
        self->OnRightButtonDown();
      }
      break;

    case vtkCommand::RightButtonReleaseEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::RightButtonReleaseEvent))
      {
        self->InvokeEvent(vtkCommand::RightButtonReleaseEvent, nullptr);
      }
      else
      {
        self->OnRightButtonUp();
      }
      break;

    case vtkCommand::LeftButtonDoubleClickEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::LeftButtonDoubleClickEvent))
      {
        self->InvokeEvent(vtkCommand::vtkCommand::LeftButtonDoubleClickEvent, nullptr);
      }
      else
      {
        self->OnLeftButtonDoubleClick();
      }
      break;

    case vtkCommand::MiddleButtonDoubleClickEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::MiddleButtonDoubleClickEvent))
      {
        self->InvokeEvent(vtkCommand::vtkCommand::MiddleButtonDoubleClickEvent, nullptr);
      }
      else
      {
        self->OnMiddleButtonDoubleClick();
      }
      break;

    case vtkCommand::RightButtonDoubleClickEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::RightButtonDoubleClickEvent))
      {
        self->InvokeEvent(vtkCommand::vtkCommand::RightButtonDoubleClickEvent, nullptr);
      }
      else
      {
        self->OnRightButtonDoubleClick();
      }
      break;

    case vtkCommand::MouseWheelForwardEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::MouseWheelForwardEvent))
      {
        self->InvokeEvent(vtkCommand::MouseWheelForwardEvent, nullptr);
      }
      else
      {
        self->OnMouseWheelForward();
      }
      break;

    case vtkCommand::MouseWheelBackwardEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::MouseWheelBackwardEvent))
      {
        self->InvokeEvent(vtkCommand::MouseWheelBackwardEvent, nullptr);
      }
      else
      {
        self->OnMouseWheelBackward();
      }
      break;

    case vtkCommand::MouseWheelLeftEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::MouseWheelLeftEvent))
      {
        self->InvokeEvent(vtkCommand::MouseWheelLeftEvent, nullptr);
      }
      else
      {
        self->OnMouseWheelLeft();
      }
      break;

    case vtkCommand::MouseWheelRightEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::MouseWheelRightEvent))
      {
        self->InvokeEvent(vtkCommand::MouseWheelRightEvent, nullptr);
      }
      else
      {
        self->OnMouseWheelRight();
      }
      break;

    case vtkCommand::KeyPressEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::KeyPressEvent))
      {
        self->InvokeEvent(vtkCommand::KeyPressEvent, nullptr);
      }
      else
      {
        self->OnKeyDown();
        self->OnKeyPress();
      }
      break;

    case vtkCommand::KeyReleaseEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::KeyReleaseEvent))
      {
        self->InvokeEvent(vtkCommand::KeyReleaseEvent, nullptr);
      }
      else
      {
        self->OnKeyUp();
        self->OnKeyRelease();
      }
      break;

    case vtkCommand::CharEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::CharEvent))
      {
        self->InvokeEvent(vtkCommand::CharEvent, nullptr);
      }
      else
      {
        self->OnChar();
      }
      break;

    case vtkCommand::DeleteEvent:
      self->SetInteractor(nullptr);
      break;

    case vtkCommand::TDxMotionEvent:
    case vtkCommand::TDxButtonPressEvent:
    case vtkCommand::TDxButtonReleaseEvent:
      self->DelegateTDxEvent(event, calldata);
      break;

    case vtkCommand::StartSwipeEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::StartSwipeEvent))
      {
        self->InvokeEvent(vtkCommand::StartSwipeEvent, nullptr);
      }
      else
      {
        self->OnStartSwipe();
      }
      break;
    case vtkCommand::SwipeEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::SwipeEvent))
      {
        self->InvokeEvent(vtkCommand::SwipeEvent, nullptr);
      }
      else
      {
        self->OnSwipe();
      }
      break;
    case vtkCommand::EndSwipeEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::EndSwipeEvent))
      {
        self->InvokeEvent(vtkCommand::EndSwipeEvent, nullptr);
      }
      else
      {
        self->OnEndSwipe();
      }
      break;

    case vtkCommand::StartPinchEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::StartPinchEvent))
      {
        self->InvokeEvent(vtkCommand::StartPinchEvent, nullptr);
      }
      else
      {
        self->OnStartPinch();
      }
      break;
    case vtkCommand::PinchEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::PinchEvent))
      {
        self->InvokeEvent(vtkCommand::PinchEvent, nullptr);
      }
      else
      {
        self->OnPinch();
      }
      break;
    case vtkCommand::EndPinchEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::EndPinchEvent))
      {
        self->InvokeEvent(vtkCommand::EndPinchEvent, nullptr);
      }
      else
      {
        self->OnEndPinch();
      }
      break;

    case vtkCommand::StartPanEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::StartPanEvent))
      {
        self->InvokeEvent(vtkCommand::StartPanEvent, nullptr);
      }
      else
      {
        self->OnStartPan();
      }
      break;
    case vtkCommand::PanEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::PanEvent))
      {
        self->InvokeEvent(vtkCommand::PanEvent, nullptr);
      }
      else
      {
        self->OnPan();
      }
      break;
    case vtkCommand::EndPanEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::EndPanEvent))
      {
        self->InvokeEvent(vtkCommand::EndPanEvent, nullptr);
      }
      else
      {
        self->OnEndPan();
      }
      break;

    case vtkCommand::StartRotateEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::StartRotateEvent))
      {
        self->InvokeEvent(vtkCommand::StartRotateEvent, nullptr);
      }
      else
      {
        self->OnStartRotate();
      }
      break;
    case vtkCommand::RotateEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::RotateEvent))
      {
        self->InvokeEvent(vtkCommand::RotateEvent, nullptr);
      }
      else
      {
        self->OnRotate();
      }
      break;
    case vtkCommand::EndRotateEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::EndRotateEvent))
      {
        self->InvokeEvent(vtkCommand::EndRotateEvent, nullptr);
      }
      else
      {
        self->OnEndRotate();
      }
      break;

    case vtkCommand::TapEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::TapEvent))
      {
        self->InvokeEvent(vtkCommand::TapEvent, nullptr);
      }
      else
      {
        self->OnTap();
      }
      break;

    case vtkCommand::LongTapEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::LongTapEvent))
      {
        self->InvokeEvent(vtkCommand::LongTapEvent, nullptr);
      }
      else
      {
        self->OnLongTap();
      }
      break;

    case vtkCommand::FourthButtonPressEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::FourthButtonPressEvent))
      {
        self->InvokeEvent(vtkCommand::FourthButtonPressEvent, nullptr);
      }
      else
      {
        self->OnFourthButtonDown();
      }
      break;

    case vtkCommand::FourthButtonReleaseEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::FourthButtonReleaseEvent))
      {
        self->InvokeEvent(vtkCommand::FourthButtonReleaseEvent, nullptr);
      }
      else
      {
        self->OnFourthButtonUp();
      }
      break;

    case vtkCommand::FifthButtonPressEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::FifthButtonPressEvent))
      {
        self->InvokeEvent(vtkCommand::FifthButtonPressEvent, nullptr);
      }
      else
      {
        self->OnFifthButtonDown();
      }
      break;

    case vtkCommand::FifthButtonReleaseEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::FifthButtonReleaseEvent))
      {
        self->InvokeEvent(vtkCommand::FifthButtonReleaseEvent, nullptr);
      }
      else
      {
        self->OnFifthButtonUp();
      }
      break;

      vtkISEventDataMacro(Move3D);
      vtkISEventDataMacro(Button3D);
      vtkISEventDataMacro(Menu3D);
      vtkISEventDataMacro(Select3D);
      vtkISEventDataMacro(NextPose3D);
      vtkISEventDataMacro(ViewerMovement3D);
      vtkISEventDataMacro(Pick3D);
      vtkISEventDataMacro(PositionProp3D);
      vtkISEventDataMacro(Clip3D);

    case vtkCommand::DropFilesEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::DropFilesEvent))
      {
        aborted = (self->InvokeEvent(vtkCommand::DropFilesEvent, calldata) == 1);
      }
      if (!aborted)
      {
        self->OnDropFiles(static_cast<vtkStringArray*>(calldata));
      }
      break;

    case vtkCommand::UpdateDropLocationEvent:
      if (self->HandleObservers && self->HasObserver(vtkCommand::UpdateDropLocationEvent))
      {
        aborted = (self->InvokeEvent(vtkCommand::UpdateDropLocationEvent, calldata) == 1);
      }
      if (!aborted)
      {
        self->OnDropLocation(static_cast<double*>(calldata));
      }
      break;
  }
}
