/*=========================================================================

Program:   Visualization Toolkit
Module:    vtkDepthPeelingPass.cxx

Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.
See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkDepthPeelingPass.h"
#include "vtkInformation.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGLActor.h"
#include "vtkOpenGLError.h"
#include "vtkOpenGLFramebufferObject.h"
#include "vtkOpenGLQuadHelper.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkOpenGLRenderer.h"
#include "vtkOpenGLShaderCache.h"
#include "vtkOpenGLState.h"
#include "vtkProp.h"
#include "vtkRenderState.h"
#include "vtkRenderer.h"
#include "vtkShaderProgram.h"
#include "vtkTextureObject.h"
#include <cassert>
#include <list>

#include "vtkRenderStepsPass.h"

#include "vtkOpenGLHelper.h"

// the 2D blending shaders we use
#include "vtkDepthPeelingPassFinalFS.h"
#include "vtkDepthPeelingPassIntermediateFS.h"

vtkStandardNewMacro(vtkDepthPeelingPass);
vtkCxxSetObjectMacro(vtkDepthPeelingPass, TranslucentPass, vtkRenderPass);

//------------------------------------------------------------------------------
vtkDepthPeelingPass::vtkDepthPeelingPass()
  : Framebuffer(nullptr)
{
  this->TranslucentPass = nullptr;

  this->OcclusionRatio = 0.0;
  this->MaximumNumberOfPeels = 4;

  this->IntermediateBlend = nullptr;
  this->FinalBlend = nullptr;

  this->OpaqueRGBATexture = nullptr;
  this->OpaqueZTexture = nullptr;
  this->OwnOpaqueZTexture = false;
  this->OwnOpaqueRGBATexture = false;

  this->TranslucentZTexture[0] = vtkTextureObject::New();
  this->TranslucentZTexture[1] = vtkTextureObject::New();
  this->DepthFormat = vtkTextureObject::Float32;

  for (int i = 0; i < 3; i++)
  {
    this->TranslucentRGBATexture[i] = vtkTextureObject::New();
  }

  this->ViewportX = 0;
  this->ViewportY = 0;
  this->ViewportWidth = 100;
  this->ViewportHeight = 100;
}

//------------------------------------------------------------------------------
vtkDepthPeelingPass::~vtkDepthPeelingPass()
{
  if (this->TranslucentPass != nullptr)
  {
    this->TranslucentPass->Delete();
  }
  if (this->OpaqueZTexture)
  {
    this->OpaqueZTexture->UnRegister(this);
    this->OpaqueZTexture = nullptr;
  }
  if (this->TranslucentZTexture[0])
  {
    this->TranslucentZTexture[0]->UnRegister(this);
    this->TranslucentZTexture[0] = nullptr;
  }
  if (this->TranslucentZTexture[1])
  {
    this->TranslucentZTexture[1]->UnRegister(this);
    this->TranslucentZTexture[1] = nullptr;
  }
  if (this->OpaqueRGBATexture)
  {
    this->OpaqueRGBATexture->UnRegister(this);
    this->OpaqueRGBATexture = nullptr;
  }
  for (int i = 0; i < 3; i++)
  {
    if (this->TranslucentRGBATexture[i])
    {
      this->TranslucentRGBATexture[i]->UnRegister(this);
      this->TranslucentRGBATexture[i] = nullptr;
    }
  }
  if (this->Framebuffer)
  {
    this->Framebuffer->UnRegister(this);
    this->Framebuffer = nullptr;
  }
}

//------------------------------------------------------------------------------
// Description:
// Destructor. Delete SourceCode if any.
void vtkDepthPeelingPass::ReleaseGraphicsResources(vtkWindow* w)
{
  assert("pre: w_exists" && w != nullptr);

  if (this->FinalBlend != nullptr)
  {
    delete this->FinalBlend;
    this->FinalBlend = nullptr;
  }
  if (this->IntermediateBlend != nullptr)
  {
    delete this->IntermediateBlend;
    this->IntermediateBlend = nullptr;
  }
  if (this->TranslucentPass)
  {
    this->TranslucentPass->ReleaseGraphicsResources(w);
  }
  if (this->OpaqueZTexture)
  {
    this->OpaqueZTexture->ReleaseGraphicsResources(w);
  }
  if (this->TranslucentZTexture[0])
  {
    this->TranslucentZTexture[0]->ReleaseGraphicsResources(w);
  }
  if (this->TranslucentZTexture[1])
  {
    this->TranslucentZTexture[1]->ReleaseGraphicsResources(w);
  }
  if (this->OpaqueRGBATexture)
  {
    this->OpaqueRGBATexture->ReleaseGraphicsResources(w);
  }
  for (int i = 0; i < 3; i++)
  {
    if (this->TranslucentRGBATexture[i])
    {
      this->TranslucentRGBATexture[i]->ReleaseGraphicsResources(w);
    }
  }
  if (this->Framebuffer)
  {
    this->Framebuffer->ReleaseGraphicsResources(w);
    this->Framebuffer->UnRegister(this);
    this->Framebuffer = nullptr;
  }
}

void vtkDepthPeelingPass::SetOpaqueZTexture(vtkTextureObject* to)
{
  if (this->OpaqueZTexture == to)
  {
    return;
  }
  if (this->OpaqueZTexture)
  {
    this->OpaqueZTexture->Delete();
  }
  this->OpaqueZTexture = to;
  if (to)
  {
    to->Register(this);
  }
  this->OwnOpaqueZTexture = false;
  this->Modified();
}

void vtkDepthPeelingPass::SetOpaqueRGBATexture(vtkTextureObject* to)
{
  if (this->OpaqueRGBATexture == to)
  {
    return;
  }
  if (this->OpaqueRGBATexture)
  {
    this->OpaqueRGBATexture->Delete();
  }
  this->OpaqueRGBATexture = to;
  if (to)
  {
    to->Register(this);
  }
  this->OwnOpaqueRGBATexture = false;
  this->Modified();
}

//------------------------------------------------------------------------------
void vtkDepthPeelingPass::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "OcclusionRatio: " << this->OcclusionRatio << endl;

  os << indent << "MaximumNumberOfPeels: " << this->MaximumNumberOfPeels << endl;

  os << indent << "TranslucentPass:";
  if (this->TranslucentPass != nullptr)
  {
    this->TranslucentPass->PrintSelf(os, indent);
  }
  else
  {
    os << "(none)" << endl;
  }
}

void vtkDepthPeelingPassCreateTexture(vtkTextureObject* to, vtkOpenGLRenderWindow* context,
  int width, int height, int numComponents, bool isDepth, int depthFormat)
{
  to->SetContext(context);
  if (isDepth == true)
  {
    to->AllocateDepth(width, height, depthFormat);
  }
  else
  {
    to->Allocate2D(width, height, numComponents, VTK_UNSIGNED_CHAR);
  }

  to->SetMinificationFilter(vtkTextureObject::Nearest);
  to->SetMagnificationFilter(vtkTextureObject::Nearest);
  to->SetWrapS(vtkTextureObject::ClampToEdge);
  to->SetWrapT(vtkTextureObject::ClampToEdge);
}

void vtkDepthPeelingPass::BlendIntermediatePeels(vtkOpenGLRenderWindow* renWin, bool done)
{
  // take the TranslucentRGBA texture and blend it with the current frame buffer
  if (!this->IntermediateBlend)
  {
    this->IntermediateBlend =
      new vtkOpenGLQuadHelper(renWin, nullptr, vtkDepthPeelingPassIntermediateFS, "");
  }
  else
  {
    renWin->GetShaderCache()->ReadyShaderProgram(this->IntermediateBlend->Program);
  }
  this->IntermediateBlend->Program->SetUniformi("translucentRGBATexture",
    this->TranslucentRGBATexture[(this->ColorDrawCount - 2) % 3]->GetTextureUnit());
  this->IntermediateBlend->Program->SetUniformi("currentRGBATexture",
    this->TranslucentRGBATexture[(this->ColorDrawCount - 1) % 3]->GetTextureUnit());
  this->IntermediateBlend->Program->SetUniformi("lastpass", done ? 1 : 0);

  this->State->vtkglDisable(GL_DEPTH_TEST);

  this->Framebuffer->AddColorAttachment(0, this->TranslucentRGBATexture[this->ColorDrawCount % 3]);
  this->ColorDrawCount++;

  this->IntermediateBlend->Render();
}

void vtkDepthPeelingPass::BlendFinalPeel(vtkOpenGLRenderWindow* renWin)
{
  if (!this->FinalBlend)
  {
    this->FinalBlend = new vtkOpenGLQuadHelper(renWin, nullptr, vtkDepthPeelingPassFinalFS, "");
  }
  else
  {
    renWin->GetShaderCache()->ReadyShaderProgram(this->FinalBlend->Program);
  }

  if (this->FinalBlend->Program)
  {
    this->FinalBlend->Program->SetUniformi("translucentRGBATexture",
      this->TranslucentRGBATexture[(this->ColorDrawCount - 1) % 3]->GetTextureUnit());

    // Store the current active texture
    vtkOpenGLState::ScopedglActiveTexture activeTexture(this->State);

    this->OpaqueRGBATexture->Activate();
    this->FinalBlend->Program->SetUniformi(
      "opaqueRGBATexture", this->OpaqueRGBATexture->GetTextureUnit());

    this->OpaqueZTexture->Activate();
    this->FinalBlend->Program->SetUniformi(
      "opaqueZTexture", this->OpaqueZTexture->GetTextureUnit());

    this->Framebuffer->AddColorAttachment(
      0, this->TranslucentRGBATexture[this->ColorDrawCount % 3]);
    this->ColorDrawCount++;

    // blend in OpaqueRGBA
    this->State->vtkglEnable(GL_DEPTH_TEST);
    this->State->vtkglDepthFunc(GL_ALWAYS);

    // do we need to set the viewport
    this->FinalBlend->Render();
  }
  this->State->vtkglDepthFunc(GL_LEQUAL);
}

//------------------------------------------------------------------------------
// Description:
// Perform rendering according to a render state \p s.
// \pre s_exists: s!=0
void vtkDepthPeelingPass::Render(const vtkRenderState* s)
{
  assert("pre: s_exists" && s != nullptr);

  this->NumberOfRenderedProps = 0;

  if (this->TranslucentPass == nullptr)
  {
    vtkWarningMacro(<< "No TranslucentPass delegate set. Nothing can be rendered.");
    return;
  }

  // Any prop to render?
  bool hasTranslucentPolygonalGeometry = false;
  int i = 0;
  while (!hasTranslucentPolygonalGeometry && i < s->GetPropArrayCount())
  {
    hasTranslucentPolygonalGeometry = s->GetPropArray()[i]->HasTranslucentPolygonalGeometry() == 1;
    ++i;
  }
  if (!hasTranslucentPolygonalGeometry)
  {
    return; // nothing to render.
  }

  // check driver support
  vtkOpenGLRenderWindow* renWin =
    vtkOpenGLRenderWindow::SafeDownCast(s->GetRenderer()->GetRenderWindow());
  this->State = renWin->GetState();

  // we need alpha planes
  int rgba[4];
  renWin->GetColorBufferSizes(rgba);

  if (rgba[3] < 8)
  {
    // just use alpha blending
    this->TranslucentPass->Render(s);
    return;
  }

  // Depth peeling.
  vtkRenderer* r = s->GetRenderer();

  if (s->GetFrameBuffer() == nullptr)
  {
    // get the viewport dimensions
    r->GetTiledSizeAndOrigin(
      &this->ViewportWidth, &this->ViewportHeight, &this->ViewportX, &this->ViewportY);
  }
  else
  {
    int size[2];
    s->GetWindowSize(size);
    this->ViewportWidth = size[0];
    this->ViewportHeight = size[1];
    this->ViewportX = 0;
    this->ViewportY = 0;
  }

  // create textures we need if not done already
  if (this->TranslucentRGBATexture[0]->GetHandle() == 0)
  {
    for (i = 0; i < 3; i++)
    {
      vtkDepthPeelingPassCreateTexture(this->TranslucentRGBATexture[i], renWin, this->ViewportWidth,
        this->ViewportHeight, 4, false, 0);
    }
    vtkDepthPeelingPassCreateTexture(this->TranslucentZTexture[0], renWin, this->ViewportWidth,
      this->ViewportHeight, 1, true, this->DepthFormat);
    vtkDepthPeelingPassCreateTexture(this->TranslucentZTexture[1], renWin, this->ViewportWidth,
      this->ViewportHeight, 1, true, this->DepthFormat);
    if (!this->OpaqueZTexture)
    {
      this->OwnOpaqueZTexture = true;
      this->OpaqueZTexture = vtkTextureObject::New();
      vtkDepthPeelingPassCreateTexture(this->OpaqueZTexture, renWin, this->ViewportWidth,
        this->ViewportHeight, 1, true, this->DepthFormat);
    }
    if (!this->OpaqueRGBATexture)
    {
      this->OwnOpaqueRGBATexture = true;
      this->OpaqueRGBATexture = vtkTextureObject::New();
      vtkDepthPeelingPassCreateTexture(
        this->OpaqueRGBATexture, renWin, this->ViewportWidth, this->ViewportHeight, 4, false, 0);
    }
  }

  for (i = 0; i < 3; i++)
  {
    this->TranslucentRGBATexture[i]->Resize(this->ViewportWidth, this->ViewportHeight);
  }
  this->TranslucentZTexture[0]->Resize(this->ViewportWidth, this->ViewportHeight);
  this->TranslucentZTexture[1]->Resize(this->ViewportWidth, this->ViewportHeight);

  if (this->OwnOpaqueZTexture)
  {
    this->OpaqueZTexture->Resize(this->ViewportWidth, this->ViewportHeight);
    this->OpaqueZTexture->CopyFromFrameBuffer(this->ViewportX, this->ViewportY, this->ViewportX,
      this->ViewportY, this->ViewportWidth, this->ViewportHeight);
  }

  if (this->OwnOpaqueRGBATexture)
  {
    this->OpaqueRGBATexture->Resize(this->ViewportWidth, this->ViewportHeight);
    this->OpaqueRGBATexture->CopyFromFrameBuffer(this->ViewportX, this->ViewportY, this->ViewportX,
      this->ViewportY, this->ViewportWidth, this->ViewportHeight);
  }

  if (!this->Framebuffer)
  {
    this->Framebuffer = vtkOpenGLFramebufferObject::New();
    this->Framebuffer->SetContext(renWin);
  }
  this->State->PushFramebufferBindings();
  this->Framebuffer->Bind();
  this->Framebuffer->AddDepthAttachment(this->TranslucentZTexture[0]);
  this->Framebuffer->AddColorAttachment(0, this->TranslucentRGBATexture[0]);

  this->State->vtkglViewport(0, 0, this->ViewportWidth, this->ViewportHeight);
  bool saveScissorTestState = this->State->GetEnumState(GL_SCISSOR_TEST);
  this->State->vtkglDisable(GL_SCISSOR_TEST);

  this->State->vtkglClearDepth(static_cast<GLclampf>(0.0));
  this->State->vtkglClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  this->State->vtkglColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  this->State->vtkglClearColor(0.0, 0.0, 0.0, 0.0); // always clear to black
  this->State->vtkglClearDepth(static_cast<GLclampf>(1.0));
  // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  this->Framebuffer->AddDepthAttachment(this->TranslucentZTexture[1]);
  this->State->vtkglClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

#ifdef GL_MULTISAMPLE
  bool multiSampleStatus = this->State->GetEnumState(GL_MULTISAMPLE);
  this->State->vtkglDisable(GL_MULTISAMPLE);
#endif
  this->State->vtkglDisable(GL_BLEND);

  // Store the current active texture
  vtkOpenGLState::ScopedglActiveTexture activeTexture(this->State);

  this->TranslucentZTexture[0]->Activate();
  this->OpaqueZTexture->Activate();

  this->TranslucentRGBATexture[0]->Activate();
  this->TranslucentRGBATexture[1]->Activate();
  this->TranslucentRGBATexture[2]->Activate();

  // Setup property keys for actors:
  this->PreRender(s);

  // Enable the depth buffer (otherwise it's disabled for translucent geometry)
  assert("Render state valid." && s);
  int numProps = s->GetPropArrayCount();
  for (int j = 0; j < numProps; ++j)
  {
    vtkProp* prop = s->GetPropArray()[j];
    vtkInformation* info = prop->GetPropertyKeys();
    if (!info)
    {
      info = vtkInformation::New();
      prop->SetPropertyKeys(info);
      info->FastDelete();
    }
    info->Set(vtkOpenGLActor::GLDepthMaskOverride(), 1);
  }

  // Do render loop until complete
  unsigned int threshold =
    static_cast<unsigned int>(this->ViewportWidth * this->ViewportHeight * OcclusionRatio);

#ifndef GL_ES_VERSION_3_0
  GLuint queryId;
  glGenQueries(1, &queryId);
#endif

  bool done = false;
  GLuint nbPixels = threshold + 1;
  this->PeelCount = 0;
  this->ColorDrawCount = 0;
  this->State->vtkglDepthFunc(GL_LEQUAL);
  while (!done)
  {
    this->State->vtkglDepthMask(GL_TRUE);
    this->State->vtkglEnable(GL_DEPTH_TEST);

    this->Framebuffer->AddColorAttachment(
      0, this->TranslucentRGBATexture[this->ColorDrawCount % 3]);
    this->ColorDrawCount++;

    // clear the zbuffer and color buffers
    this->State->vtkglClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // render the translucent geometry
#ifndef GL_ES_VERSION_3_0
    glBeginQuery(GL_SAMPLES_PASSED, queryId);
#endif

    // check if we are going to exceed the max number of peels or if we
    // exceeded the pixel threshold last time
    this->PeelCount++;
    if ((this->MaximumNumberOfPeels && this->PeelCount >= this->MaximumNumberOfPeels) ||
      nbPixels <= threshold)
    {
      done = true;
      // if so we do this last render using alpha blending for all
      // the stuff that is left
      this->State->vtkglEnable(GL_BLEND);
      this->State->vtkglDepthFunc(GL_ALWAYS);
    }
    this->TranslucentPass->Render(s);
    this->State->vtkglDepthFunc(GL_LEQUAL);
    this->State->vtkglDisable(GL_BLEND);

#ifndef GL_ES_VERSION_3_0
    glEndQuery(GL_SAMPLES_PASSED);
    glGetQueryObjectuiv(queryId, GL_QUERY_RESULT, &nbPixels);
#endif
    // cerr << "Pass " << peelCount << " pixels Drawn " << nbPixels << "\n";

    // if something was drawn, blend it in
    if (nbPixels > 0)
    {
      // update translucentZ pingpong the textures
      if (this->PeelCount % 2)
      {
        this->TranslucentZTexture[0]->Deactivate();
        this->Framebuffer->AddDepthAttachment(this->TranslucentZTexture[0]);
        this->TranslucentZTexture[1]->Activate();
      }
      else
      {
        this->TranslucentZTexture[1]->Deactivate();
        this->Framebuffer->AddDepthAttachment(this->TranslucentZTexture[1]);
        this->TranslucentZTexture[0]->Activate();
      }

      // blend the last two peels together
      if (this->PeelCount > 1)
      {
        this->BlendIntermediatePeels(renWin, done);
      }
    }
    else // if we drew nothing we are done
    {
      this->ColorDrawCount--;
      done = true;
    }
  }

  //  std::cout << "Number of peels: " << peelCount << "\n";

  // do the final blend if anything was drawn
  // something is drawn only when ColorDrawCount
  // is not zero or PeelCount is > 1
  if (this->PeelCount > 1 || this->ColorDrawCount != 0)
  {
    this->BlendFinalPeel(renWin);
  }

  this->State->PopFramebufferBindings();

  // Restore the original viewport and scissor test settings
  this->State->vtkglViewport(
    this->ViewportX, this->ViewportY, this->ViewportWidth, this->ViewportHeight);
  if (saveScissorTestState)
  {
    this->State->vtkglEnable(GL_SCISSOR_TEST);
  }
  else
  {
    this->State->vtkglDisable(GL_SCISSOR_TEST);
  }

  // blit if we drew something
  if (this->PeelCount > 1 || this->ColorDrawCount != 0)
  {
    this->State->PushReadFramebufferBinding();
    this->Framebuffer->Bind(vtkOpenGLFramebufferObject::GetReadMode());

    this->State->vtkglBlitFramebuffer(0, 0, this->ViewportWidth, this->ViewportHeight,
      this->ViewportX, this->ViewportY, this->ViewportX + this->ViewportWidth,
      this->ViewportY + this->ViewportHeight, GL_COLOR_BUFFER_BIT, GL_LINEAR);

    this->State->PopReadFramebufferBinding();
  }

#ifdef GL_MULTISAMPLE
  if (multiSampleStatus)
  {
    this->State->vtkglEnable(GL_MULTISAMPLE);
  }
#endif

  // unload the textures
  this->OpaqueZTexture->Deactivate();
  this->OpaqueRGBATexture->Deactivate();
  this->TranslucentRGBATexture[0]->Deactivate();
  this->TranslucentRGBATexture[1]->Deactivate();
  this->TranslucentRGBATexture[2]->Deactivate();
  this->TranslucentZTexture[0]->Deactivate();
  this->TranslucentZTexture[1]->Deactivate();

  // restore blending
  this->State->vtkglEnable(GL_BLEND);

  this->PostRender(s);
  for (int j = 0; j < numProps; ++j)
  {
    vtkProp* prop = s->GetPropArray()[j];
    vtkInformation* info = prop->GetPropertyKeys();
    if (info)
    {
      info->Remove(vtkOpenGLActor::GLDepthMaskOverride());
    }
  }

  this->NumberOfRenderedProps = this->TranslucentPass->GetNumberOfRenderedProps();

  vtkOpenGLCheckErrorMacro("failed after Render");
}

//------------------------------------------------------------------------------
bool vtkDepthPeelingPass::PostReplaceShaderValues(
  std::string&, std::string&, std::string& fragmentShader, vtkAbstractMapper*, vtkProp*)
{
  vtkShaderProgram::Substitute(fragmentShader, "//VTK::DepthPeeling::Dec",
    "uniform vec2 vpSize;\n"
    "uniform sampler2D opaqueZTexture;\n"
    "uniform sampler2D translucentZTexture;\n");

  // Set gl_FragDepth if it isn't set already. It may have already been replaced
  // by the mapper, in which case the substitution will fail and the previously
  // set depth value will be used.
  vtkShaderProgram::Substitute(
    fragmentShader, "//VTK::Depth::Impl", "gl_FragDepth = gl_FragCoord.z;");

  // the .0000001 below is an epsilon.  It turns out that
  // graphics cards can render the same polygon two times
  // in a row with different z values. I suspect it has to
  // do with how rasterization of the polygon is broken up.
  // A different breakup across fragment shaders can result in
  // very slightly different z values for some of the pixels.
  // The end result is that with depth peeling, you can end up
  // counting/accumulating pixels of the same surface twice
  // simply due to this randomness in z values. So we introduce
  // an epsilon into the transparent test to require some
  // minimal z separation between pixels
  vtkShaderProgram::Substitute(fragmentShader, "//VTK::DepthPeeling::Impl",
    "vec2 dpTexCoord = gl_FragCoord.xy / vpSize;\n"
    "  float odepth = texture2D(opaqueZTexture, dpTexCoord).r;\n"
    "  if (gl_FragDepth >= odepth) { discard; }\n"
    "  float tdepth = texture2D(translucentZTexture, dpTexCoord).r;\n"
    "  if (gl_FragDepth <= tdepth + .0000001) { discard; }\n");

  return true;
}

//------------------------------------------------------------------------------
bool vtkDepthPeelingPass::SetShaderParameters(vtkShaderProgram* program, vtkAbstractMapper*,
  vtkProp*, vtkOpenGLVertexArrayObject* vtkNotUsed(VAO))
{
  program->SetUniformi("opaqueZTexture", this->OpaqueZTexture->GetTextureUnit());
  program->SetUniformi(
    "translucentZTexture", this->TranslucentZTexture[(this->PeelCount + 1) % 2]->GetTextureUnit());

  float vpSize[2] = { static_cast<float>(this->ViewportWidth),
    static_cast<float>(this->ViewportHeight) };
  program->SetUniform2f("vpSize", vpSize);

  return true;
}
