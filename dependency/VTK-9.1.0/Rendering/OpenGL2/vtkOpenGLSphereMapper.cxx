/*=========================================================================

  Program:   Visualization Toolkit

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkOpenGLSphereMapper.h"

#include "vtkOpenGLHelper.h"

#include "vtkFloatArray.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGLActor.h"
#include "vtkOpenGLCamera.h"
#include "vtkOpenGLIndexBufferObject.h"
#include "vtkOpenGLVertexArrayObject.h"
#include "vtkOpenGLVertexBufferObject.h"
#include "vtkOpenGLVertexBufferObjectGroup.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkProperty.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkShaderProgram.h"
#include "vtkUnsignedCharArray.h"

#include "vtkPointGaussianVS.h"
#include "vtkSphereMapperGS.h"

#include "vtk_glew.h"

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkOpenGLSphereMapper);

//------------------------------------------------------------------------------
vtkOpenGLSphereMapper::vtkOpenGLSphereMapper()
{
  this->ScaleArray = nullptr;
  this->Invert = false;
  this->Radius = 0.3;
}

//------------------------------------------------------------------------------
void vtkOpenGLSphereMapper::GetShaderTemplate(
  std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* actor)
{
  this->Superclass::GetShaderTemplate(shaders, ren, actor);
  shaders[vtkShader::Vertex]->SetSource(vtkPointGaussianVS);
  shaders[vtkShader::Geometry]->SetSource(vtkSphereMapperGS);
}

void vtkOpenGLSphereMapper::ReplaceShaderValues(
  std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* actor)
{
  std::string VSSource = shaders[vtkShader::Vertex]->GetSource();
  std::string FSSource = shaders[vtkShader::Fragment]->GetSource();

  vtkShaderProgram::Substitute(VSSource, "//VTK::Camera::Dec",
    "uniform mat4 VCDCMatrix;\n"
    "uniform mat4 MCVCMatrix;");

  vtkShaderProgram::Substitute(FSSource, "//VTK::PositionVC::Dec", "in vec4 vertexVCVSOutput;");

  // we create vertexVC below, so turn off the default
  // implementation
  vtkShaderProgram::Substitute(
    FSSource, "//VTK::PositionVC::Impl", "vec4 vertexVC = vertexVCVSOutput;\n");

  // for lights kit and positional the VCDC matrix is already defined
  // so don't redefine it
  std::string replacement = "uniform float invertedDepth;\n"
                            "in float radiusVCVSOutput;\n"
                            "in vec3 centerVCVSOutput;\n"
                            "uniform mat4 VCDCMatrix;\n";
  vtkShaderProgram::Substitute(FSSource, "//VTK::Normal::Dec", replacement);

  vtkShaderProgram::Substitute(FSSource, "//VTK::Depth::Impl",
    // compute the eye position and unit direction
    "  vec3 EyePos;\n"
    "  vec3 EyeDir;\n"
    "  if (cameraParallel != 0) {\n"
    "    EyePos = vec3(vertexVC.x, vertexVC.y, vertexVC.z + 3.0*radiusVCVSOutput);\n"
    "    EyeDir = vec3(0.0,0.0,-1.0); }\n"
    "  else {\n"
    "    EyeDir = vertexVC.xyz;\n"
    "    EyePos = vec3(0.0,0.0,0.0);\n"
    "    float lengthED = length(EyeDir);\n"
    "    EyeDir = normalize(EyeDir);\n"
    // we adjust the EyePos to be closer if it is too far away
    // to prevent floating point precision noise
    "    if (lengthED > radiusVCVSOutput*3.0) {\n"
    "      EyePos = vertexVC.xyz - EyeDir*3.0*radiusVCVSOutput; }\n"
    "    }\n"

    // translate to Sphere center
    "  EyePos = EyePos - centerVCVSOutput;\n"
    // scale to radius 1.0
    "  EyePos = EyePos/radiusVCVSOutput;\n"
    // find the intersection
    "  float b = 2.0*dot(EyePos,EyeDir);\n"
    "  float c = dot(EyePos,EyePos) - 1.0;\n"
    "  float d = b*b - 4.0*c;\n"
    "  vec3 normalVCVSOutput = vec3(0.0,0.0,1.0);\n"
    "  if (d < 0.0) { discard; }\n"
    "  float t = (-b - invertedDepth*sqrt(d))*0.5;\n"

    // compute the normal, for unit sphere this is just
    // the intersection point
    "  normalVCVSOutput = normalize(EyePos + t*EyeDir);\n"
    // compute the intersection point in VC
    "  vertexVC.xyz = normalVCVSOutput*radiusVCVSOutput + centerVCVSOutput;\n"
    "  normalVCVSOutput *= invertedDepth;\n"
    // compute the pixel's depth
    // " normalVCVSOutput = vec3(0,0,1);\n"
    "  vec4 pos = VCDCMatrix * vertexVC;\n"
    "  gl_FragDepth = (pos.z / pos.w + 1.0) / 2.0;\n");

  // Strip out the normal line -- the normal is computed as part of the depth
  vtkShaderProgram::Substitute(FSSource, "//VTK::Normal::Impl", "");

  shaders[vtkShader::Vertex]->SetSource(VSSource);
  shaders[vtkShader::Fragment]->SetSource(FSSource);

  this->Superclass::ReplaceShaderValues(shaders, ren, actor);
}

//------------------------------------------------------------------------------
vtkOpenGLSphereMapper::~vtkOpenGLSphereMapper()
{
  this->SetScaleArray(nullptr);
}

//------------------------------------------------------------------------------
void vtkOpenGLSphereMapper::SetCameraShaderParameters(
  vtkOpenGLHelper& cellBO, vtkRenderer* ren, vtkActor* actor)
{
  vtkShaderProgram* program = cellBO.Program;

  vtkOpenGLCamera* cam = (vtkOpenGLCamera*)(ren->GetActiveCamera());

  vtkMatrix4x4* wcdc;
  vtkMatrix4x4* wcvc;
  vtkMatrix3x3* norms;
  vtkMatrix4x4* vcdc;
  cam->GetKeyMatrices(ren, wcvc, norms, vcdc, wcdc);
  if (program->IsUniformUsed("VCDCMatrix"))
  {
    program->SetUniformMatrix("VCDCMatrix", vcdc);
  }

  if (program->IsUniformUsed("MCVCMatrix"))
  {
    if (!actor->GetIsIdentity())
    {
      vtkMatrix4x4* mcwc;
      vtkMatrix3x3* anorms;
      ((vtkOpenGLActor*)actor)->GetKeyMatrices(mcwc, anorms);
      vtkMatrix4x4::Multiply4x4(mcwc, wcvc, this->TempMatrix4);
      program->SetUniformMatrix("MCVCMatrix", this->TempMatrix4);
    }
    else
    {
      program->SetUniformMatrix("MCVCMatrix", wcvc);
    }
  }

  if (program->IsUniformUsed("cameraParallel"))
  {
    cellBO.Program->SetUniformi("cameraParallel", cam->GetParallelProjection());
  }
}

//------------------------------------------------------------------------------
void vtkOpenGLSphereMapper::SetMapperShaderParameters(
  vtkOpenGLHelper& cellBO, vtkRenderer* ren, vtkActor* actor)
{
  if (cellBO.Program->IsUniformUsed("invertedDepth"))
  {
    cellBO.Program->SetUniformf("invertedDepth", this->Invert ? -1.0 : 1.0);
  }

  this->Superclass::SetMapperShaderParameters(cellBO, ren, actor);
}

//------------------------------------------------------------------------------
void vtkOpenGLSphereMapper::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Radius: " << this->Radius << "\n";
}

// internal function called by CreateVBO
void vtkOpenGLSphereMapper::CreateVBO(vtkPolyData* poly, vtkIdType numPts, unsigned char* colors,
  int colorComponents, vtkIdType nc, float* sizes, vtkIdType ns, vtkRenderer* ren)
{
  vtkFloatArray* offsets = vtkFloatArray::New();
  offsets->SetNumberOfComponents(1);
  offsets->SetNumberOfTuples(numPts);
  float* oPtr = static_cast<float*>(offsets->GetVoidPointer(0));

  vtkUnsignedCharArray* ucolors = vtkUnsignedCharArray::New();
  ucolors->SetNumberOfComponents(4);
  ucolors->SetNumberOfTuples(numPts);
  unsigned char* cPtr = static_cast<unsigned char*>(ucolors->GetVoidPointer(0));

  unsigned char* colorPtr;

  for (vtkIdType i = 0; i < numPts; ++i)
  {
    colorPtr = (nc == numPts ? colors + i * colorComponents : colors);
    float radius = (ns == numPts ? sizes[i] : sizes[0]);

    *(cPtr++) = colorPtr[0];
    *(cPtr++) = colorPtr[1];
    *(cPtr++) = colorPtr[2];
    *(cPtr++) = colorPtr[3];
    *(oPtr++) = radius;
  }

  this->VBOs->CacheDataArray("vertexMC", poly->GetPoints()->GetData(), ren, VTK_FLOAT);

  this->VBOs->CacheDataArray("radiusMC", offsets, ren, VTK_FLOAT);
  offsets->Delete();
  this->VBOs->CacheDataArray("scalarColor", ucolors, ren, VTK_UNSIGNED_CHAR);
  ucolors->Delete();
  VBOs->BuildAllVBOs(ren);
}

//------------------------------------------------------------------------------
void vtkOpenGLSphereMapper::BuildBufferObjects(vtkRenderer* ren, vtkActor* act)
{
  vtkPolyData* poly = this->CurrentInput;

  if (poly == nullptr) // || !poly->GetPointData()->GetNormals())
  {
    return;
  }

  // For vertex coloring, this sets this->Colors as side effect.
  // For texture map coloring, this sets ColorCoordinates
  // and ColorTextureMap as a side effect.
  // I moved this out of the conditional because it is fast.
  // Color arrays are cached. If nothing has changed,
  // then the scalars do not have to be regenerted.
  this->MapScalars(1.0);

  vtkIdType numPts = poly->GetPoints()->GetNumberOfPoints();
  unsigned char* c;
  int cc;
  vtkIdType nc;
  if (this->Colors)
  {
    c = (unsigned char*)this->Colors->GetVoidPointer(0);
    nc = numPts;
    cc = this->Colors->GetNumberOfComponents();
  }
  else
  {
    double* ac = act->GetProperty()->GetColor();
    double opac = act->GetProperty()->GetOpacity();
    c = new unsigned char[4];
    c[0] = (unsigned char)(ac[0] * 255.0);
    c[1] = (unsigned char)(ac[1] * 255.0);
    c[2] = (unsigned char)(ac[2] * 255.0);
    c[3] = (unsigned char)(opac * 255.0);
    nc = 1;
    cc = 4;
  }

  float* scales;
  vtkIdType ns = numPts;
  if (this->ScaleArray != nullptr && poly->GetPointData()->HasArray(this->ScaleArray))
  {
    scales =
      static_cast<float*>(poly->GetPointData()->GetArray(this->ScaleArray)->GetVoidPointer(0));
  }
  else
  {
    scales = &this->Radius;
    ns = 1;
  }

  // Iterate through all of the different types in the polydata, building OpenGLs
  // and IBOs as appropriate for each type.
  this->CreateVBO(poly, numPts, c, cc, nc, scales, ns, ren);

  if (!this->Colors)
  {
    delete[] c;
  }

  // create the IBO
  this->Primitives[PrimitivePoints].IBO->IndexCount = 0;
  this->Primitives[PrimitiveLines].IBO->IndexCount = 0;
  this->Primitives[PrimitiveTriStrips].IBO->IndexCount = 0;
  this->Primitives[PrimitiveTris].IBO->IndexCount = numPts;
  this->VBOBuildTime.Modified();
}

//------------------------------------------------------------------------------
void vtkOpenGLSphereMapper::Render(vtkRenderer* ren, vtkActor* act)
{
  vtkProperty* prop = act->GetProperty();
  bool is_opaque = (prop->GetOpacity() >= 1.0);

  // if we are transparent (and not backface culling) we have to draw twice
  if (!is_opaque && !prop->GetBackfaceCulling())
  {
    this->Invert = true;
    this->Superclass::Render(ren, act);
    this->Invert = false;
  }
  this->Superclass::Render(ren, act);
}

//------------------------------------------------------------------------------
void vtkOpenGLSphereMapper::RenderPieceDraw(vtkRenderer* ren, vtkActor* actor)
{
  // draw polygons
  int numVerts = this->VBOs->GetNumberOfTuples("vertexMC");
  if (numVerts)
  {
    // First we do the triangles, update the shader, set uniforms, etc.
    this->UpdateShaders(this->Primitives[PrimitiveTris], ren, actor);
    glDrawArrays(GL_POINTS, 0, static_cast<GLuint>(numVerts));
  }
}
