/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkCompositeRGBAPass.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "vtkCompositeRGBAPass.h"
#include "vtkFrameBufferObjectBase.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkOpenGLRenderer.h"
#include "vtkOpenGLState.h"
#include "vtkRenderState.h"
#include "vtkTextureObject.h"
#include <cassert>

// to be able to dump intermediate result into png files for debugging.
// only for vtkCompositeRGBAPass developers.
//#define VTK_COMPOSITE_RGBAPASS_DEBUG

#include "vtkCamera.h"
#include "vtkImageData.h"
#include "vtkImageExtractComponents.h"
#include "vtkImageImport.h"
#include "vtkImageShiftScale.h"
#include "vtkIntArray.h"
#include "vtkMultiProcessController.h"
#include "vtkPKdTree.h"
#include "vtkPNGWriter.h"
#include "vtkPixelBufferObject.h"
#include "vtkPointData.h"
#include "vtkStdString.h"
#include "vtkTimerLog.h"
#include <sstream>

#ifdef VTK_COMPOSITE_RGBAPASS_DEBUG
//#include <unistd.h>
#include <sys/syscall.h>
#include <sys/types.h> // Linux specific gettid()
#endif

#include "vtk_glew.h"

vtkStandardNewMacro(vtkCompositeRGBAPass);
vtkCxxSetObjectMacro(vtkCompositeRGBAPass, Controller, vtkMultiProcessController);
vtkCxxSetObjectMacro(vtkCompositeRGBAPass, Kdtree, vtkPKdTree);

//------------------------------------------------------------------------------
vtkCompositeRGBAPass::vtkCompositeRGBAPass()
{
  this->Controller = nullptr;
  this->Kdtree = nullptr;
  this->PBO = nullptr;
  this->RGBATexture = nullptr;
  this->RootTexture = nullptr;
  this->RawRGBABuffer = nullptr;
  this->RawRGBABufferSize = 0;
}

//------------------------------------------------------------------------------
vtkCompositeRGBAPass::~vtkCompositeRGBAPass()
{
  if (this->Controller != nullptr)
  {
    this->Controller->Delete();
  }
  if (this->Kdtree != nullptr)
  {
    this->Kdtree->Delete();
  }
  if (this->PBO != nullptr)
  {
    vtkErrorMacro(<< "PixelBufferObject should have been deleted in ReleaseGraphicsResources().");
  }
  if (this->RGBATexture != nullptr)
  {
    vtkErrorMacro(<< "RGBATexture should have been deleted in ReleaseGraphicsResources().");
  }
  if (this->RootTexture != nullptr)
  {
    vtkErrorMacro(<< "RootTexture should have been deleted in ReleaseGraphicsResources().");
  }
  delete[] this->RawRGBABuffer;
}

//------------------------------------------------------------------------------
void vtkCompositeRGBAPass::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);

  os << indent << "Controller:";
  if (this->Controller != nullptr)
  {
    this->Controller->PrintSelf(os, indent);
  }
  else
  {
    os << "(none)" << endl;
  }
  os << indent << "Kdtree:";
  if (this->Kdtree != nullptr)
  {
    this->Kdtree->PrintSelf(os, indent);
  }
  else
  {
    os << "(none)" << endl;
  }
}

//------------------------------------------------------------------------------
bool vtkCompositeRGBAPass::IsSupported(vtkOpenGLRenderWindow* context)
{
  return (context != nullptr);
}

//------------------------------------------------------------------------------
// Description:
// Perform rendering according to a render state \p s.
// \pre s_exists: s!=0
void vtkCompositeRGBAPass::Render(const vtkRenderState* s)
{
  assert("pre: s_exists" && s != nullptr);

  if (this->Controller == nullptr)
  {
    vtkErrorMacro(<< " no controller.");
    return;
  }

  int numProcs = this->Controller->GetNumberOfProcesses();

  if (numProcs == 1)
  {
    return; // nothing to do.
  }

  if (this->Kdtree == nullptr)
  {
    vtkErrorMacro(<< " no Kdtree.");
    return;
  }

  int me = this->Controller->GetLocalProcessId();

  const int VTK_COMPOSITE_RGBA_PASS_MESSAGE_GATHER = 201;

  vtkOpenGLRenderer* r = static_cast<vtkOpenGLRenderer*>(s->GetRenderer());

  vtkOpenGLRenderWindow* context = static_cast<vtkOpenGLRenderWindow*>(r->GetRenderWindow());
  vtkOpenGLState* ostate = context->GetState();

  if (!this->IsSupported(context))
  {
    vtkErrorMacro(<< "Missing required OpenGL extensions. "
                  << "Cannot perform rgba-compositing.");
    return;
  }

  int w = 0;
  int h = 0;

  vtkFrameBufferObjectBase* fbo = s->GetFrameBuffer();
  if (fbo == nullptr)
  {
    r->GetTiledSize(&w, &h);
  }
  else
  {
    int size[2];
    fbo->GetLastSize(size);
    w = size[0];
    h = size[1];
  }

  int numComps = 4;
  unsigned int numTups = w * h;

  // pbo arguments.
  unsigned int dims[2];
  vtkIdType continuousInc[3];

  dims[0] = static_cast<unsigned int>(w);
  dims[1] = static_cast<unsigned int>(h);
  continuousInc[0] = 0;
  continuousInc[1] = 0;
  continuousInc[2] = 0;

  if (this->RawRGBABuffer != nullptr && this->RawRGBABufferSize < static_cast<size_t>(w * h * 4))
  {
    delete[] this->RawRGBABuffer;
    this->RawRGBABuffer = nullptr;
  }
  if (this->RawRGBABuffer == nullptr)
  {
    this->RawRGBABufferSize = static_cast<size_t>(w * h * 4);
    this->RawRGBABuffer = new float[this->RawRGBABufferSize];
  }

  // size_t byteSize = this->RawRGBABufferSize*sizeof(unsigned char);

  if (this->PBO == nullptr)
  {
    this->PBO = vtkPixelBufferObject::New();
    this->PBO->SetContext(context);
  }
  if (this->RGBATexture == nullptr)
  {
    this->RGBATexture = vtkTextureObject::New();
    this->RGBATexture->SetContext(context);
  }

  // TO: texture object
  // PBO: pixel buffer object
  // FB: framebuffer

#ifdef VTK_COMPOSITE_RGBAPASS_DEBUG
  vtkTimerLog* timer = vtkTimerLog::New();

  vtkImageImport* importer;
  vtkImageShiftScale* converter;
  vtkPNGWriter* writer;

  cout << "me=" << me << " TID=" << syscall(SYS_gettid) << " thread=" << pthread_self() << endl;
  timer->StartTimer();
#endif

  if (me == 0)
  {
    // root
    // 1. figure out the back to front ordering
    // 2. if root is not farest, save it in a TO
    // 3. in back to front order:
    // 3a. if this is step for root, render root TO (if not farest)
    // 3b. if satellite, get image, load it into TO, render quad

#ifdef VTK_COMPOSITE_RGBAPASS_DEBUG
    // get rgba-buffer of root before any blending with satellite
    // for debugging only.

    // Framebuffer to PBO
    this->PBO->Allocate(VTK_FLOAT, numTups, numComps, vtkPixelBufferObject::PACKED_BUFFER);

    this->PBO->Bind(vtkPixelBufferObject::PACKED_BUFFER);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_FLOAT, static_cast<GLfloat*>(nullptr));
    cout << "after readpixel." << endl;
    this->PBO->Download2D(VTK_FLOAT, this->RawRGBABuffer, dims, 4, continuousInc);
    cout << "after pbodownload." << endl;
    importer = vtkImageImport::New();
    importer->CopyImportVoidPointer(this->RawRGBABuffer, static_cast<int>(byteSize));
    importer->SetDataScalarTypeToFloat();
    importer->SetNumberOfScalarComponents(4);
    importer->SetWholeExtent(0, w - 1, 0, h - 1, 0, 0);
    importer->SetDataExtentToWholeExtent();

    importer->Update();
    cout << "after importer update" << endl;
    converter = vtkImageShiftScale::New();
    converter->SetInputConnection(importer->GetOutputPort());
    converter->SetOutputScalarTypeToUnsignedChar();
    converter->SetShift(0.0);
    converter->SetScale(255.0);

    //      vtkImageExtractComponents *rgbaToRgb=vtkImageExtractComponents::New();
    //    rgbaToRgb->SetInputConnection(importer->GetOutputPort());
    //    rgbaToRgb->SetComponents(0,1,2);

    std::ostringstream ostxx;
    ostxx.setf(ios::fixed, ios::floatfield);
    ostxx.precision(5);
    timer->StopTimer();
    ostxx << "root0_" << vtkTimerLog::GetUniversalTime() << "_.png";

    vtkStdString* sssxx = new vtkStdString;
    (*sssxx) = ostxx.str();

    writer = vtkPNGWriter::New();
    writer->SetFileName(*sssxx);
    delete sssxx;
    writer->SetInputConnection(converter->GetOutputPort());
    converter->Delete();
    importer->Delete();
    //    rgbaToRgb->Delete();
    cout << "Writing " << writer->GetFileName() << endl;
    writer->Write();
    cout << "Wrote " << writer->GetFileName() << endl;
    //    sleep(30);
    writer->Delete();
#endif // #ifdef VTK_COMPOSITE_RGBAPASS_DEBUG

    // 1. figure out the back to front ordering
    vtkCamera* c = r->GetActiveCamera();
    vtkIntArray* frontToBackList = vtkIntArray::New();
    if (c->GetParallelProjection())
    {
      this->Kdtree->ViewOrderAllProcessesInDirection(
        c->GetDirectionOfProjection(), frontToBackList);
    }
    else
    {
      this->Kdtree->ViewOrderAllProcessesFromPosition(c->GetPosition(), frontToBackList);
    }

    assert("check same_size" && frontToBackList->GetNumberOfTuples() == numProcs);

#ifdef VTK_COMPOSITE_RGBAPASS_DEBUG
    int i = 0;
    while (i < numProcs)
    {
      cout << "frontToBackList[" << i << "]=" << frontToBackList->GetValue(i) << endl;
      ++i;
    }
#endif

    // framebuffers have their color premultiplied by alpha.

    // save off current state of src / dst blend functions
    {
      vtkOpenGLState::ScopedglBlendFuncSeparate bfsaver(ostate);

      ostate->vtkglColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

      // per-fragment operations
      ostate->vtkglDisable(GL_DEPTH_TEST);
      ostate->vtkglDisable(GL_BLEND);
      ostate->vtkglBlendFuncSeparate(
        GL_ONE, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

      ostate->vtkglPixelStorei(GL_UNPACK_ALIGNMENT, 1); // client to server

      // 2. if root is not farest, save it in a TO
      bool rootIsFarest = frontToBackList->GetValue(numProcs - 1) == 0;
      if (!rootIsFarest)
      {
        if (this->RootTexture == nullptr)
        {
          this->RootTexture = vtkTextureObject::New();
          this->RootTexture->SetContext(context);
        }
        this->RootTexture->Allocate2D(dims[0], dims[1], 4, VTK_UNSIGNED_CHAR);
        this->RootTexture->CopyFromFrameBuffer(0, 0, 0, 0, w, h);
      }

      // 3. in back to front order:
      // 3a. if this is step for root, render root TO (if not farest)
      // 3b. if satellite, get image, load it into TO, render quad

      int procIndex = numProcs - 1;
      bool blendingEnabled = false;
      if (rootIsFarest)
      {
        // nothing to do.
        --procIndex;
      }
      while (procIndex >= 0)
      {
        vtkTextureObject* to;
        int proc = frontToBackList->GetValue(procIndex);
        if (proc == 0)
        {
          to = this->RootTexture;
        }
        else
        {
          // receive the rgba from satellite process.
          this->Controller->Receive(this->RawRGBABuffer,
            static_cast<vtkIdType>(this->RawRGBABufferSize), proc,
            VTK_COMPOSITE_RGBA_PASS_MESSAGE_GATHER);

          // send it to a PBO
          this->PBO->Upload2D(VTK_FLOAT, this->RawRGBABuffer, dims, 4, continuousInc);
          // Send PBO to TO
          this->RGBATexture->Create2D(dims[0], dims[1], 4, this->PBO, false);
          to = this->RGBATexture;
        }
        if (!blendingEnabled && procIndex < (numProcs - 1))
        {
          ostate->vtkglEnable(GL_BLEND);
          blendingEnabled = true;
        }
        to->Activate();
        to->CopyToFrameBuffer(0, 0, w - 1, h - 1, 0, 0, w, h, nullptr, nullptr);
        to->Deactivate();
        --procIndex;
      }
      // restore blend func by going out of scope
    }

    frontToBackList->Delete();

#ifdef VTK_COMPOSITE_RGBAPASS_DEBUG
    // get rgba-buffer of root before any blending with satellite
    // for debugging only.

    // Framebuffer to PBO
    this->PBO->Allocate(VTK_FLOAT, numTups, numComps, vtkPixelBufferObject::PACKED_BUFFER);

    this->PBO->Bind(vtkPixelBufferObject::PACKED_BUFFER);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_FLOAT, static_cast<GLfloat*>(nullptr));

    this->PBO->Download2D(VTK_FLOAT, this->RawRGBABuffer, dims, 4, continuousInc);

    importer = vtkImageImport::New();
    importer->CopyImportVoidPointer(this->RawRGBABuffer, static_cast<int>(byteSize));
    importer->SetDataScalarTypeToFloat();
    importer->SetNumberOfScalarComponents(4);
    importer->SetWholeExtent(0, w - 1, 0, h - 1, 0, 0);
    importer->SetDataExtentToWholeExtent();

    importer->Update();

    converter = vtkImageShiftScale::New();
    converter->SetInputConnection(importer->GetOutputPort());
    converter->SetOutputScalarTypeToUnsignedChar();
    converter->SetShift(0.0);
    converter->SetScale(255.0);

    //      vtkImageExtractComponents *rgbaToRgb=vtkImageExtractComponents::New();
    //    rgbaToRgb->SetInputConnection(importer->GetOutputPort());
    //    rgbaToRgb->SetComponents(0,1,2);

    std::ostringstream osty;
    osty.setf(ios::fixed, ios::floatfield);
    osty.precision(5);
    timer->StopTimer();
    osty << "rootend_" << vtkTimerLog::GetUniversalTime() << "_.png";

    vtkStdString* sssy = new vtkStdString;
    (*sssy) = osty.str();

    writer = vtkPNGWriter::New();
    writer->SetFileName(*sssy);
    delete sssy;
    writer->SetInputConnection(converter->GetOutputPort());
    converter->Delete();
    importer->Delete();
    //    rgbaToRgb->Delete();
    cout << "Writing " << writer->GetFileName() << endl;
    writer->Write();
    cout << "Wrote " << writer->GetFileName() << endl;
    //    sleep(30);
    writer->Delete();
#endif

    // root node Done.
  }
  else
  {
    // satellite
    // send rgba-buffer

    // framebuffer to PBO.
    this->PBO->Allocate(VTK_FLOAT, numTups, numComps, vtkPixelBufferObject::PACKED_BUFFER);

    this->PBO->Bind(vtkPixelBufferObject::PACKED_BUFFER);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_FLOAT, static_cast<GLfloat*>(nullptr));

    // PBO to client
    ostate->vtkglPixelStorei(GL_PACK_ALIGNMENT, 1); // server to client
    this->PBO->Download2D(VTK_FLOAT, this->RawRGBABuffer, dims, 4, continuousInc);
    this->PBO->UnBind();

#ifdef VTK_COMPOSITE_RGBAPASS_DEBUG
    importer = vtkImageImport::New();
    importer->CopyImportVoidPointer(this->RawRGBABuffer, static_cast<int>(byteSize));
    importer->SetDataScalarTypeToFloat();
    importer->SetNumberOfScalarComponents(4);
    importer->SetWholeExtent(0, w - 1, 0, h - 1, 0, 0);
    importer->SetDataExtentToWholeExtent();

    importer->Update();

    converter = vtkImageShiftScale::New();
    converter->SetInputConnection(importer->GetOutputPort());
    converter->SetOutputScalarTypeToUnsignedChar();
    converter->SetShift(0.0);
    converter->SetScale(255.0);

    //      vtkImageExtractComponents *rgbaToRgb=vtkImageExtractComponents::New();
    //    rgbaToRgb->SetInputConnection(importer->GetOutputPort());
    //    rgbaToRgb->SetComponents(0,1,2);

    std::ostringstream ostxx;
    ostxx.setf(ios::fixed, ios::floatfield);
    ostxx.precision(5);
    timer->StopTimer();
    ostxx << "satellite_send_" << vtkTimerLog::GetUniversalTime() << "_.png";

    vtkStdString* sssxx = new vtkStdString;
    (*sssxx) = ostxx.str();

    writer = vtkPNGWriter::New();
    writer->SetFileName(*sssxx);
    delete sssxx;
    writer->SetInputConnection(converter->GetOutputPort());
    converter->Delete();
    importer->Delete();
    //    rgbaToRgb->Delete();
    cout << "Writing " << writer->GetFileName() << endl;
    writer->Write();
    cout << "Wrote " << writer->GetFileName() << endl;
    //    sleep(30);
    writer->Delete();
#endif

    // client to root process
    this->Controller->Send(this->RawRGBABuffer, static_cast<vtkIdType>(this->RawRGBABufferSize), 0,
      VTK_COMPOSITE_RGBA_PASS_MESSAGE_GATHER);
  }
#ifdef VTK_COMPOSITE_RGBAPASS_DEBUG
  delete state;
  timer->Delete();
#endif
}

//------------------------------------------------------------------------------
// Description:
// Release graphics resources and ask components to release their own
// resources.
// \pre w_exists: w!=0
void vtkCompositeRGBAPass::ReleaseGraphicsResources(vtkWindow* w)
{
  assert("pre: w_exists" && w != nullptr);

  (void)w;

  if (this->PBO != nullptr)
  {
    this->PBO->Delete();
    this->PBO = nullptr;
  }
  if (this->RGBATexture != nullptr)
  {
    this->RGBATexture->Delete();
    this->RGBATexture = nullptr;
  }
  if (this->RootTexture != nullptr)
  {
    this->RootTexture->Delete();
    this->RootTexture = nullptr;
  }
}
