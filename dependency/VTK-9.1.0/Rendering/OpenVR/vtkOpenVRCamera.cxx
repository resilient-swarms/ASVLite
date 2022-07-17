/*=========================================================================

  Program:   Visualization Toolkit

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkOpenVRCamera.h"

#include "vtkMatrix3x3.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGLError.h"
#include "vtkOpenGLState.h"
#include "vtkOpenVRRenderWindow.h"
#include "vtkPerspectiveTransform.h"
#include "vtkRenderer.h"
#include "vtkTimerLog.h"

#include <cmath>

vtkStandardNewMacro(vtkOpenVRCamera);

//------------------------------------------------------------------------------
vtkOpenVRCamera::vtkOpenVRCamera()
{
  this->LeftEyeProjection = nullptr;
  this->RightEyeProjection = nullptr;

  this->LeftEyeTCDCMatrix = vtkMatrix4x4::New();
  this->RightEyeTCDCMatrix = vtkMatrix4x4::New();

  // approximate for Vive
  // we use the projection matrix directly from the vive
  // so this is just to help make view <--> display
  // adjustments reasonable, not correct, just reasonable
  this->SetViewAngle(110.0);
}

//------------------------------------------------------------------------------
vtkOpenVRCamera::~vtkOpenVRCamera()
{
  if (this->LeftEyeProjection)
  {
    this->LeftEyeProjection->Delete();
    this->RightEyeProjection->Delete();
    this->LeftEyeProjection = nullptr;
    this->RightEyeProjection = nullptr;
  }

  this->LeftEyeTCDCMatrix->Delete();
  this->RightEyeTCDCMatrix->Delete();
}

//------------------------------------------------------------------------------
void vtkOpenVRCamera::GetHMDEyePoses(vtkRenderer* ren)
{
  vtkOpenVRRenderWindow* win = vtkOpenVRRenderWindow::SafeDownCast(ren->GetRenderWindow());

  vr::IVRSystem* hMD = win->GetHMD();

  // left handed coordinate system so have to -1*Z
  vr::HmdMatrix34_t matEye = hMD->GetEyeToHeadTransform(vr::Eye_Left);
  this->LeftEyePose[0] = matEye.m[0][3];
  this->LeftEyePose[1] = matEye.m[1][3];
  this->LeftEyePose[2] = -matEye.m[2][3];

  matEye = hMD->GetEyeToHeadTransform(vr::Eye_Right);
  this->RightEyePose[0] = matEye.m[0][3];
  this->RightEyePose[1] = matEye.m[1][3];
  this->RightEyePose[2] = -matEye.m[2][3];
}

//------------------------------------------------------------------------------
void vtkOpenVRCamera::GetHMDEyeProjections(vtkRenderer* ren)
{
  vtkOpenVRRenderWindow* win = vtkOpenVRRenderWindow::SafeDownCast(ren->GetRenderWindow());

  vr::IVRSystem* hMD = win->GetHMD();

  double znear = this->ClippingRange[0];
  double zfar = this->ClippingRange[1];

  float fxmin, fxmax, fymin, fymax;
  double xmin, xmax, ymin, ymax;

  // note docs are probably wrong in OpenVR arg list for this func
  hMD->GetProjectionRaw(vr::Eye_Left, &fxmin, &fxmax, &fymin, &fymax);
  xmin = fxmin * znear;
  xmax = fxmax * znear;
  ymin = fymin * znear;
  ymax = fymax * znear;

  this->LeftEyeProjection->Zero();
  this->LeftEyeProjection->SetElement(0, 0, 2 * znear / (xmax - xmin));
  this->LeftEyeProjection->SetElement(1, 1, 2 * znear / (ymax - ymin));
  this->LeftEyeProjection->SetElement(2, 0, (xmin + xmax) / (xmax - xmin));
  this->LeftEyeProjection->SetElement(2, 1, (ymin + ymax) / (ymax - ymin));
  this->LeftEyeProjection->SetElement(2, 2, -(znear + zfar) / (zfar - znear));
  this->LeftEyeProjection->SetElement(2, 3, -1);
  this->LeftEyeProjection->SetElement(3, 2, -2 * znear * zfar / (zfar - znear));

  hMD->GetProjectionRaw(vr::Eye_Right, &fxmin, &fxmax, &fymin, &fymax);
  xmin = fxmin * znear;
  xmax = fxmax * znear;
  ymin = fymin * znear;
  ymax = fymax * znear;

  this->RightEyeProjection->Zero();
  this->RightEyeProjection->SetElement(0, 0, 2 * znear / (xmax - xmin));
  this->RightEyeProjection->SetElement(1, 1, 2 * znear / (ymax - ymin));
  this->RightEyeProjection->SetElement(2, 0, (xmin + xmax) / (xmax - xmin));
  this->RightEyeProjection->SetElement(2, 1, (ymin + ymax) / (ymax - ymin));
  this->RightEyeProjection->SetElement(2, 2, -(znear + zfar) / (zfar - znear));
  this->RightEyeProjection->SetElement(2, 3, -1);
  this->RightEyeProjection->SetElement(3, 2, -2 * znear * zfar / (zfar - znear));
}

void vtkOpenVRCamera::ApplyEyePose(vtkVRRenderWindow* win, bool left, double factor)
{
  double physicalScale = win->GetPhysicalScale();

  double* dop = this->GetDirectionOfProjection();
  double* vup = this->GetViewUp();
  double vright[3];
  vtkMath::Cross(dop, vup, vright);

  double* offset = (left ? this->LeftEyePose : this->RightEyePose);
  double newOffset[3];
  newOffset[0] =
    factor * (offset[0] * vright[0] + offset[1] * vup[0] - offset[2] * dop[0]) * physicalScale;
  newOffset[1] =
    factor * (offset[0] * vright[1] + offset[1] * vup[1] - offset[2] * dop[1]) * physicalScale;
  newOffset[2] =
    factor * (offset[0] * vright[2] + offset[1] * vup[2] - offset[2] * dop[2]) * physicalScale;
  double* pos = this->GetPosition();
  this->SetPosition(pos[0] + newOffset[0], pos[1] + newOffset[1], pos[2] + newOffset[2]);
  double* fp = this->GetFocalPoint();
  this->SetFocalPoint(fp[0] + newOffset[0], fp[1] + newOffset[1], fp[2] + newOffset[2]);
}

//------------------------------------------------------------------------------
void vtkOpenVRCamera::Render(vtkRenderer* ren)
{
  vtkOpenGLClearErrorMacro();

  vtkVRRenderWindow* win = vtkVRRenderWindow::SafeDownCast(ren->GetRenderWindow());
  vtkOpenGLState* ostate = win->GetState();

  int renSize[2];
  win->GetRenderBufferSize(renSize[0], renSize[1]);

  // get the eye pose and projection matricies once
  if (!this->LeftEyeProjection)
  {
    this->LeftEyeProjection = vtkMatrix4x4::New();
    this->RightEyeProjection = vtkMatrix4x4::New();
    this->GetHMDEyePoses(ren);
  }

  // if were on a stereo renderer draw to special parts of screen
  if (this->LeftEye)
  {
    // Left Eye
    if (win->GetMultiSamples() && !ren->GetSelector())
    {
      ostate->vtkglEnable(GL_MULTISAMPLE);
    }

    // adjust for left eye position
    if (!ren->GetSelector())
    {
      this->ApplyEyePose(win, true, 1.0);
    }
  }
  else
  {
    // right eye
    if (win->GetMultiSamples() && !ren->GetSelector())
    {
      ostate->vtkglEnable(GL_MULTISAMPLE);
    }

    if (!ren->GetSelector())
    {
      // adjust for left eye position
      this->ApplyEyePose(win, true, -1.0);
      // adjust for right eye position
      this->ApplyEyePose(win, false, 1.0);
    }
  }

  ostate->vtkglViewport(0, 0, renSize[0], renSize[1]);
  ostate->vtkglScissor(0, 0, renSize[0], renSize[1]);
  ren->Clear();
  if ((ren->GetRenderWindow())->GetErase() && ren->GetErase())
  {
    ren->Clear();
  }

  vtkOpenGLCheckErrorMacro("failed after Render");
}

//------------------------------------------------------------------------------
void vtkOpenVRCamera::GetKeyMatrices(vtkRenderer* ren, vtkMatrix4x4*& wcvc, vtkMatrix3x3*& normMat,
  vtkMatrix4x4*& vcdc, vtkMatrix4x4*& wcdc)
{
  if (ren->GetSelector())
  {
    return this->Superclass::GetKeyMatrices(ren, wcvc, normMat, vcdc, wcdc);
  }

  // has the camera changed?
  if (ren != this->LastRenderer || this->MTime > this->KeyMatrixTime ||
    ren->GetMTime() > this->KeyMatrixTime)
  {
    vtkMatrix4x4* w2v = this->GetModelViewTransformMatrix();
    this->WCVCMatrix->DeepCopy(w2v);

    if (this->LeftEye)
    {
      this->GetHMDEyeProjections(ren);

      // only compute normal matrix once
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          this->NormalMatrix->SetElement(i, j, w2v->GetElement(i, j));
        }
      }
      this->NormalMatrix->Invert();
    }

    this->WCVCMatrix->Transpose();

    if (this->LeftEye)
    {
      vtkOpenVRRenderWindow* win = vtkOpenVRRenderWindow::SafeDownCast(ren->GetRenderWindow());

      vtkMatrix4x4::Multiply4x4(this->WCVCMatrix, this->LeftEyeProjection, this->WCDCMatrix);

      // build the tracking to device coordinate matrix
      this->PoseTransform->Identity();
      double trans[3];
      win->GetPhysicalTranslation(trans);
      this->PoseTransform->Translate(-trans[0], -trans[1], -trans[2]);
      double scale = win->GetPhysicalScale();
      this->PoseTransform->Scale(scale, scale, scale);

      // deal with Vive to World rotations
      double* vup = win->GetPhysicalViewUp();
      double* dop = win->GetPhysicalViewDirection();
      double vr[3];
      vtkMath::Cross(dop, vup, vr);
      double rot[16] = { vr[0], vup[0], -dop[0], 0.0, vr[1], vup[1], -dop[1], 0.0, vr[2], vup[2],
        -dop[2], 0.0, 0.0, 0.0, 0.0, 1.0 };

      this->PoseTransform->Concatenate(rot);

      this->LeftEyeTCDCMatrix->DeepCopy(this->PoseTransform->GetMatrix());
      this->LeftEyeTCDCMatrix->Transpose();

      vtkMatrix4x4::Multiply4x4(this->LeftEyeTCDCMatrix, this->WCDCMatrix, this->LeftEyeTCDCMatrix);
    }
    else
    {
      vtkMatrix4x4::Multiply4x4(this->WCVCMatrix, this->RightEyeProjection, this->WCDCMatrix);

      this->RightEyeTCDCMatrix->DeepCopy(this->PoseTransform->GetMatrix());
      this->RightEyeTCDCMatrix->Transpose();

      vtkMatrix4x4::Multiply4x4(
        this->RightEyeTCDCMatrix, this->WCDCMatrix, this->RightEyeTCDCMatrix);
    }

    this->KeyMatrixTime.Modified();
    this->LastRenderer = ren;
  }

  wcdc = this->WCDCMatrix;
  wcvc = this->WCVCMatrix;
  normMat = this->NormalMatrix;

  if (this->LeftEye)
  {
    vcdc = this->LeftEyeProjection;
  }
  else
  {
    vcdc = this->RightEyeProjection;
  }
}

//------------------------------------------------------------------------------
void vtkOpenVRCamera::GetTrackingToDCMatrix(vtkMatrix4x4*& tcdc)
{
  if (this->LeftEye)
  {
    tcdc = this->LeftEyeTCDCMatrix;
  }
  else
  {
    tcdc = this->RightEyeTCDCMatrix;
  }
}

//------------------------------------------------------------------------------
void vtkOpenVRCamera::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "LeftEyePose : (" << this->LeftEyePose[0] << ", " << this->LeftEyePose[1] << ", "
     << this->LeftEyePose[2] << ")\n";
  os << indent << "RightEyePose : (" << this->RightEyePose[0] << ", " << this->RightEyePose[1]
     << ", " << this->RightEyePose[2] << ")\n";

  this->LeftEyeTCDCMatrix->PrintSelf(os, indent);
  this->RightEyeTCDCMatrix->PrintSelf(os, indent);

  if (this->LeftEyeProjection != nullptr)
  {
    this->LeftEyeProjection->PrintSelf(os, indent);
  }

  if (this->RightEyeProjection != nullptr)
  {
    this->RightEyeProjection->PrintSelf(os, indent);
  }

  this->PoseTransform->PrintSelf(os, indent);
}
