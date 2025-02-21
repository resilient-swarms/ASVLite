/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkSurfaceLICMapper.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkSurfaceLICMapper.h"

#include "vtkSurfaceLICInterface.h"

#include "vtkObjectFactory.h"
#include "vtkOpenGLError.h"
#include "vtkOpenGLFramebufferObject.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkOpenGLState.h"
#include "vtkOpenGLVertexBufferObject.h"
#include "vtkOpenGLVertexBufferObjectGroup.h"
#include "vtkPainterCommunicator.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkRenderer.h"
#include "vtkShaderProgram.h"

// use parallel timer for benchmarks and scaling
// if not defined vtkTimerLOG is used.
// #define vtkSurfaceLICMapperTIME
#if !defined(vtkSurfaceLICMapperTIME)
#include "vtkTimerLog.h"
#endif
#define vtkSurfaceLICMapperDEBUG 0

//------------------------------------------------------------------------------
vtkObjectFactoryNewMacro(vtkSurfaceLICMapper);

//------------------------------------------------------------------------------
vtkSurfaceLICMapper::vtkSurfaceLICMapper()
{
  this->SetInputArrayToProcess(
    0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, vtkDataSetAttributes::VECTORS);

  this->LICInterface = vtkSurfaceLICInterface::New();
}

//------------------------------------------------------------------------------
vtkSurfaceLICMapper::~vtkSurfaceLICMapper()
{
#if vtkSurfaceLICMapperDEBUG >= 1
  cerr << "=====vtkSurfaceLICMapper::~vtkSurfaceLICMapper" << endl;
#endif

  this->LICInterface->Delete();
  this->LICInterface = nullptr;
}

void vtkSurfaceLICMapper::ShallowCopy(vtkAbstractMapper* mapper)
{
  vtkSurfaceLICMapper* m = vtkSurfaceLICMapper::SafeDownCast(mapper);
  this->LICInterface->ShallowCopy(m->GetLICInterface());

  this->SetInputArrayToProcess(0, m->GetInputArrayInformation(0));
  this->SetScalarVisibility(m->GetScalarVisibility());

  // Now do superclass
  this->vtkOpenGLPolyDataMapper::ShallowCopy(mapper);
}

//------------------------------------------------------------------------------
void vtkSurfaceLICMapper::ReleaseGraphicsResources(vtkWindow* win)
{
  this->LICInterface->ReleaseGraphicsResources(win);
  this->Superclass::ReleaseGraphicsResources(win);
}

void vtkSurfaceLICMapper::ReplaceShaderValues(
  std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* actor)
{
  std::string VSSource = shaders[vtkShader::Vertex]->GetSource();
  std::string FSSource = shaders[vtkShader::Fragment]->GetSource();

  // add some code to handle the LIC vectors and mask
  vtkShaderProgram::Substitute(VSSource, "//VTK::TCoord::Dec",
    "in vec3 vecsMC;\n"
    "out vec3 tcoordVCVSOutput;\n");

  vtkShaderProgram::Substitute(VSSource, "//VTK::TCoord::Impl", "tcoordVCVSOutput = vecsMC;");

  vtkShaderProgram::Substitute(FSSource, "//VTK::TCoord::Dec",
    // 0/1, when 1 V is projected to surface for |V| computation.
    "uniform int uMaskOnSurface;\n"
    "in vec3 tcoordVCVSOutput;\n"
    "//VTK::TCoord::Dec");

  // No need to create uniform normalMatrix as it will be done in superclass
  // if the data contains normals
  if (this->VBOs->GetNumberOfComponents("normalMC") != 3)
  {
    vtkShaderProgram::Substitute(FSSource, "//VTK::TCoord::Dec", "uniform mat3 normalMatrix;");
  }

  if (this->PrimitiveInfo[this->LastBoundBO].LastLightComplexity > 0)
  {
    vtkShaderProgram::Substitute(FSSource, "//VTK::TCoord::Impl",
      // projected vectors
      "  vec3 tcoordLIC = normalMatrix * tcoordVCVSOutput;\n"
      "  vec3 normN = normalize(normalVCVSOutput);\n"
      "  float k = dot(tcoordLIC, normN);\n"
      "  tcoordLIC = (tcoordLIC - k*normN);\n"
      "  gl_FragData[1] = vec4(tcoordLIC.x, tcoordLIC.y, 0.0 , gl_FragCoord.z);\n"
      //   "  gl_FragData[1] = vec4(tcoordVC.xyz, gl_FragCoord.z);\n"
      // vectors for fragment masking
      "  if (uMaskOnSurface == 0)\n"
      "    {\n"
      "    gl_FragData[2] = vec4(tcoordVCVSOutput, gl_FragCoord.z);\n"
      "    }\n"
      "  else\n"
      "    {\n"
      "    gl_FragData[2] = vec4(tcoordLIC.x, tcoordLIC.y, 0.0 , gl_FragCoord.z);\n"
      "    }\n"
      //   "  gl_FragData[2] = vec4(19.0, 19.0, tcoordVC.x, gl_FragCoord.z);\n"
      ,
      false);
  }

  shaders[vtkShader::Vertex]->SetSource(VSSource);
  shaders[vtkShader::Fragment]->SetSource(FSSource);

  this->Superclass::ReplaceShaderValues(shaders, ren, actor);
}

void vtkSurfaceLICMapper::SetMapperShaderParameters(
  vtkOpenGLHelper& cellBO, vtkRenderer* ren, vtkActor* actor)
{
  this->Superclass::SetMapperShaderParameters(cellBO, ren, actor);
  cellBO.Program->SetUniformi("uMaskOnSurface", this->LICInterface->GetMaskOnSurface());
}

//------------------------------------------------------------------------------
void vtkSurfaceLICMapper::RenderPiece(vtkRenderer* renderer, vtkActor* actor)
{
#ifdef vtkSurfaceLICMapperTIME
  this->StartTimerEvent("vtkSurfaceLICMapper::RenderInternal");
#else
  vtkSmartPointer<vtkTimerLog> timer = vtkSmartPointer<vtkTimerLog>::New();
  timer->StartTimer();
#endif

  vtkOpenGLClearErrorMacro();

  this->LICInterface->ValidateContext(renderer);

  this->LICInterface->UpdateCommunicator(renderer, actor, this->GetInput());

  vtkPainterCommunicator* comm = this->LICInterface->GetCommunicator();

  if (comm->GetIsNull())
  {
    // other rank's may have some visible data but we
    // have none and should not participate further
    return;
  }

  this->CurrentInput = this->GetInput();
  vtkDataArray* vectors = this->GetInputArrayToProcess(0, this->CurrentInput);
  this->LICInterface->SetHasVectors(vectors != nullptr ? true : false);

  if (!this->LICInterface->CanRenderSurfaceLIC(actor))
  {
    // we've determined that there's no work for us, or that the
    // requisite opengl extensions are not available. pass control on
    // to delegate renderer and return.
    this->Superclass::RenderPiece(renderer, actor);
#ifdef vtkSurfaceLICMapperTIME
    this->EndTimerEvent("vtkSurfaceLICMapper::RenderInternal");
#endif
    return;
  }

  // Before start rendering LIC, capture some essential state so we can restore
  // it.
  vtkOpenGLRenderWindow* rw = vtkOpenGLRenderWindow::SafeDownCast(renderer->GetRenderWindow());
  vtkOpenGLState* ostate = rw->GetState();
  vtkOpenGLState::ScopedglEnableDisable bsaver(ostate, GL_BLEND);
  vtkOpenGLState::ScopedglEnableDisable cfsaver(ostate, GL_CULL_FACE);

  vtkNew<vtkOpenGLFramebufferObject> fbo;
  fbo->SetContext(rw);
  ostate->PushFramebufferBindings();

  // allocate rendering resources, initialize or update
  // textures and shaders.
  this->LICInterface->InitializeResources();

  // draw the geometry
  this->LICInterface->PrepareForGeometry();

  this->UpdateCameraShiftScale(renderer, actor);
  this->RenderPieceStart(renderer, actor);
  this->RenderPieceDraw(renderer, actor);
  this->RenderPieceFinish(renderer, actor);
  this->LICInterface->CompletedGeometry();

  // Disable cull face to make sure geometry won't be culled again
  ostate->vtkglDisable(GL_CULL_FACE);

  // --------------------------------------------- compoiste vectors for parallel LIC
  this->LICInterface->GatherVectors();

  // ------------------------------------------- LIC on screen
  this->LICInterface->ApplyLIC();

  // ------------------------------------------- combine scalar colors + LIC
  this->LICInterface->CombineColorsAndLIC();

  // ----------------------------------------------- depth test and copy to screen
  this->LICInterface->CopyToScreen();

  ostate->PopFramebufferBindings();

  // clear opengl error flags and be absolutely certain that nothing failed.
  vtkOpenGLCheckErrorMacro("failed during surface lic painter");

#ifdef vtkSurfaceLICMapperTIME
  this->EndTimerEvent("vtkSurfaceLICMapper::RenderInternal");
#else
  timer->StopTimer();
#endif
}

//------------------------------------------------------------------------------
void vtkSurfaceLICMapper::BuildBufferObjects(vtkRenderer* ren, vtkActor* act)
{
  if (this->LICInterface->GetHasVectors())
  {
    vtkDataArray* vectors = this->GetInputArrayToProcess(0, this->CurrentInput);
    this->VBOs->CacheDataArray("vecsMC", vectors, ren, VTK_FLOAT);
  }

  this->Superclass::BuildBufferObjects(ren, act);
}

//------------------------------------------------------------------------------
void vtkSurfaceLICMapper::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}
