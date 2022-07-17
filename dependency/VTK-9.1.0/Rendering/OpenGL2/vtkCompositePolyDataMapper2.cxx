/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkCompositePolyDataMapper2.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkCompositePolyDataMapper2.h"

#include "vtk_glew.h"

#include "vtkBoundingBox.h"
#include "vtkCellData.h"
#include "vtkColorTransferFunction.h"
#include "vtkCommand.h"
#include "vtkCompositeDataDisplayAttributes.h"
#include "vtkCompositeDataPipeline.h"
#include "vtkCompositeDataSet.h"
#include "vtkCompositeDataSetRange.h"
#include "vtkDataObjectTree.h"
#include "vtkDataObjectTreeRange.h"
#include "vtkFloatArray.h"
#include "vtkHardwareSelector.h"
#include "vtkIdTypeArray.h"
#include "vtkImageData.h"
#include "vtkInformation.h"
#include "vtkLookupTable.h"
#include "vtkObjectFactory.h"
#include "vtkOpenGLCellToVTKCellMap.h"
#include "vtkOpenGLIndexBufferObject.h"
#include "vtkOpenGLRenderWindow.h"
#include "vtkOpenGLRenderer.h"
#include "vtkOpenGLShaderProperty.h"
#include "vtkOpenGLState.h"
#include "vtkOpenGLTexture.h"
#include "vtkOpenGLVertexBufferObject.h"
#include "vtkOpenGLVertexBufferObjectGroup.h"
#include "vtkPointData.h"
#include "vtkPolyData.h"
#include "vtkProperty.h"
#include "vtkScalarsToColors.h"
#include "vtkSelection.h"
#include "vtkSelectionNode.h"
#include "vtkShaderProgram.h"
#include "vtkTextureObject.h"
#include "vtkTransform.h"
#include "vtkUnsignedIntArray.h"

#include <algorithm>
#include <sstream>

#include "vtkCompositeMapperHelper2.h"

typedef std::map<vtkPolyData*, vtkCompositeMapperHelperData*>::iterator dataIter;
typedef std::map<const std::string, vtkCompositeMapperHelper2*>::iterator helpIter;

vtkStandardNewMacro(vtkCompositeMapperHelper2);

void vtkCompositeMapperHelper2::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  os << indent << "Marked: " << this->Marked << endl;
  os << indent << "Primary ID Used: " << this->PrimIDUsed << endl;
  os << indent << "Override Color Used: " << this->OverideColorUsed << endl;
}

vtkCompositeMapperHelper2::~vtkCompositeMapperHelper2()
{
  for (dataIter it = this->Data.begin(); it != this->Data.end(); ++it)
  {
    delete it->second;
  }
  this->Data.clear();
}

void vtkCompositeMapperHelper2::SetShaderValues(
  vtkShaderProgram* prog, vtkCompositeMapperHelperData* hdata, size_t primOffset)
{
  if (this->PrimIDUsed)
  {
    prog->SetUniformi("PrimitiveIDOffset", static_cast<int>(primOffset));
  }

  if (this->CurrentSelector)
  {
    if (this->CurrentSelector->GetCurrentPass() == vtkHardwareSelector::COMPOSITE_INDEX_PASS &&
      prog->IsUniformUsed("mapperIndex"))
    {
      this->CurrentSelector->RenderCompositeIndex(hdata->FlatIndex);
      prog->SetUniform3f("mapperIndex", this->CurrentSelector->GetPropColorValue());
    }
    return;
  }

  // If requested, color partial / missing arrays with NaN color.
  bool useNanColor = false;
  double nanColor[4] = { -1., -1., -1., -1 };
  if (this->Parent->GetColorMissingArraysWithNanColor() && this->GetScalarVisibility())
  {
    int cellFlag = 0;
    vtkAbstractArray* scalars = vtkAbstractMapper::GetAbstractScalars(hdata->Data, this->ScalarMode,
      this->ArrayAccessMode, this->ArrayId, this->ArrayName, cellFlag);
    if (scalars == nullptr)
    {
      vtkLookupTable* lut = vtkLookupTable::SafeDownCast(this->GetLookupTable());
      vtkColorTransferFunction* ctf =
        lut ? nullptr : vtkColorTransferFunction::SafeDownCast(this->GetLookupTable());
      if (lut)
      {
        lut->GetNanColor(nanColor);
        useNanColor = true;
      }
      else if (ctf)
      {
        ctf->GetNanColor(nanColor);
        useNanColor = true;
      }
    }
  }

  // override the opacity and color
  prog->SetUniformf("opacityUniform", hdata->Opacity);

  if (useNanColor)
  {
    float fnancolor[3] = { static_cast<float>(nanColor[0]), static_cast<float>(nanColor[1]),
      static_cast<float>(nanColor[2]) };
    prog->SetUniform3f("ambientColorUniform", fnancolor);
    prog->SetUniform3f("diffuseColorUniform", fnancolor);
  }
  else
  {
    if (this->DrawingSelection)
    {
      vtkColor3d& sColor = hdata->SelectionColor;
      float selectionColor[3] = { static_cast<float>(sColor[0]), static_cast<float>(sColor[1]),
        static_cast<float>(sColor[2]) };
      prog->SetUniform3f("ambientColorUniform", selectionColor);
      prog->SetUniform3f("diffuseColorUniform", selectionColor);
      prog->SetUniformf("opacityUniform", hdata->SelectionOpacity);
    }
    else
    {
      vtkColor3d& aColor = hdata->AmbientColor;
      float ambientColor[3] = { static_cast<float>(aColor[0]), static_cast<float>(aColor[1]),
        static_cast<float>(aColor[2]) };
      vtkColor3d& dColor = hdata->DiffuseColor;
      float diffuseColor[3] = { static_cast<float>(dColor[0]), static_cast<float>(dColor[1]),
        static_cast<float>(dColor[2]) };
      prog->SetUniform3f("ambientColorUniform", ambientColor);
      prog->SetUniform3f("diffuseColorUniform", diffuseColor);
    }
    if (this->OverideColorUsed)
    {
      prog->SetUniformi("OverridesColor", hdata->OverridesColor);
    }
  }
}

void vtkCompositeMapperHelper2::UpdateShaders(
  vtkOpenGLHelper& cellBO, vtkRenderer* ren, vtkActor* act)
{
  // in cases where LegacyShaderProperty is not nullptr, it means someone has used
  // legacy shader replacement functions, so we make sure the actor uses the same
  // shader property. NOTE: this implies that it is not possible to use both legacy
  // and new functionality on the same actor/mapper.
  if (this->Parent->LegacyShaderProperty &&
    act->GetShaderProperty() != this->Parent->LegacyShaderProperty)
  {
    act->SetShaderProperty(this->Parent->LegacyShaderProperty);
  }

  Superclass::UpdateShaders(cellBO, ren, act);
  if (cellBO.Program && this->Parent)
  {
    // allow the program to set what it wants
    this->Parent->InvokeEvent(vtkCommand::UpdateShaderEvent, cellBO.Program);
  }
}

void vtkCompositeMapperHelper2::ReplaceShaderColor(
  std::map<vtkShader::Type, vtkShader*> shaders, vtkRenderer* ren, vtkActor* actor)
{
  if (!this->CurrentSelector)
  {
    std::string FSSource = shaders[vtkShader::Fragment]->GetSource();

    vtkShaderProgram::Substitute(FSSource, "//VTK::Color::Dec",
      "uniform bool OverridesColor;\n"
      "//VTK::Color::Dec",
      false);

    vtkShaderProgram::Substitute(FSSource, "//VTK::Color::Impl",
      "//VTK::Color::Impl\n"
      "  if (OverridesColor) {\n"
      "    ambientColor = ambientColorUniform * ambientIntensity;\n"
      "    diffuseColor = diffuseColorUniform * diffuseIntensity; }\n",
      false);

    shaders[vtkShader::Fragment]->SetSource(FSSource);
  }

  this->Superclass::ReplaceShaderColor(shaders, ren, actor);
}

void vtkCompositeMapperHelper2::ClearMark()
{
  for (dataIter it = this->Data.begin(); it != this->Data.end(); ++it)
  {
    it->second->Marked = false;
  }
  this->Marked = false;
}

void vtkCompositeMapperHelper2::RemoveUnused()
{
  for (dataIter it = this->Data.begin(); it != this->Data.end();)
  {
    if (!it->second->Marked)
    {
      delete it->second;
      this->Data.erase(it++);
      this->Modified();
    }
    else
    {
      ++it;
    }
  }
}

std::vector<vtkPolyData*> vtkCompositeMapperHelper2::GetRenderedList() const
{
  std::vector<vtkPolyData*> result;
  for (const auto& pair : this->Data)
  {
    result.push_back(pair.first);
  }
  return result;
}

//------------------------------------------------------------------------------
// Returns if we can use texture maps for scalar coloring. Note this doesn't say
// we "will" use scalar coloring. It says, if we do use scalar coloring, we will
// use a texture.
// When rendering multiblock datasets, if any 2 blocks provide different
// lookup tables for the scalars, then also we cannot use textures. This case can
// be handled if required.
int vtkCompositeMapperHelper2::CanUseTextureMapForColoring(vtkDataObject*)
{
  if (!this->InterpolateScalarsBeforeMapping)
  {
    return 0; // user doesn't want us to use texture maps at all.
  }

  int cellFlag = 0;
  vtkScalarsToColors* scalarsLookupTable = nullptr;
  for (dataIter it = this->Data.begin(); it != this->Data.end(); ++it)
  {
    vtkPolyData* pd = it->second->Data;
    vtkDataArray* scalars = vtkAbstractMapper::GetScalars(
      pd, this->ScalarMode, this->ArrayAccessMode, this->ArrayId, this->ArrayName, cellFlag);

    if (scalars)
    {
      if (cellFlag)
      {
        return 0;
      }
      if ((this->ColorMode == VTK_COLOR_MODE_DEFAULT &&
            vtkArrayDownCast<vtkUnsignedCharArray>(scalars)) ||
        this->ColorMode == VTK_COLOR_MODE_DIRECT_SCALARS)
      {
        // Don't use texture if direct coloring using RGB unsigned chars is
        // requested.
        return 0;
      }

      if (scalarsLookupTable && scalars->GetLookupTable() &&
        (scalarsLookupTable != scalars->GetLookupTable()))
      {
        // Two datasets are requesting different lookup tables to color with.
        // We don't handle this case right now for composite datasets.
        return 0;
      }
      if (scalars->GetLookupTable())
      {
        scalarsLookupTable = scalars->GetLookupTable();
      }
    }
  }

  if ((scalarsLookupTable && scalarsLookupTable->GetIndexedLookup()) ||
    (!scalarsLookupTable && this->LookupTable && this->LookupTable->GetIndexedLookup()))
  {
    return 0;
  }

  return 1;
}

//------------------------------------------------------------------------------
void vtkCompositeMapperHelper2::RenderPiece(vtkRenderer* ren, vtkActor* actor)
{
  // Make sure that we have been properly initialized.
  if (ren->GetRenderWindow()->CheckAbortStatus())
  {
    return;
  }

  if (ren->GetSelector())
  {
    for (auto& it : this->Data)
    {
      this->CurrentInput = it.first;
      this->UpdateMaximumPointCellIds(ren, actor);
    }
  }
  this->CurrentInput = this->Data.begin()->first;

  this->UpdateCameraShiftScale(ren, actor);
  this->RenderPieceStart(ren, actor);
  this->RenderPieceDraw(ren, actor);
  this->RenderPieceFinish(ren, actor);
}

void vtkCompositeMapperHelper2::UpdateCameraShiftScale(vtkRenderer* ren, vtkActor* actor)
{
  if (this->PauseShiftScale)
  {
    return;
  }

  // handle camera shift scale
  if (this->ShiftScaleMethod == vtkOpenGLVertexBufferObject::NEAR_PLANE_SHIFT_SCALE ||
    this->ShiftScaleMethod == vtkOpenGLVertexBufferObject::FOCAL_POINT_SHIFT_SCALE)
  {
    // get ideal shift scale from camera
    auto posVBO = this->VBOs->GetVBO("vertexMC");
    if (posVBO)
    {
      posVBO->SetCamera(ren->GetActiveCamera());
      posVBO->SetProp3D(actor);
      posVBO->UpdateShiftScale(this->CurrentInput->GetPoints()->GetData());
      // force a rebuild if needed
      if (posVBO->GetMTime() > posVBO->GetUploadTime())
      {
        this->Modified();
      }
    }
  }
}

void vtkCompositeMapperHelper2::DrawIBO(vtkRenderer* ren, vtkActor* actor, int primType,
  vtkOpenGLHelper& CellBO, GLenum mode, int pointSize)
{
  if (CellBO.IBO->IndexCount)
  {
    vtkOpenGLRenderWindow* renWin = static_cast<vtkOpenGLRenderWindow*>(ren->GetRenderWindow());
    vtkOpenGLState* ostate = renWin->GetState();

    if (pointSize > 0)
    {
      ostate->vtkglPointSize(pointSize); // need to use shader value
    }
    // First we do the triangles, update the shader, set uniforms, etc.
    this->UpdateShaders(CellBO, ren, actor);
    vtkShaderProgram* prog = CellBO.Program;
    if (!prog)
    {
      return;
    }
    this->PrimIDUsed = prog->IsUniformUsed("PrimitiveIDOffset");
    this->OverideColorUsed = prog->IsUniformUsed("OverridesColor");
    CellBO.IBO->Bind();

    if (!this->HaveWideLines(ren, actor) && mode == GL_LINES)
    {
      ostate->vtkglLineWidth(actor->GetProperty()->GetLineWidth());
    }

    // if (this->DrawingEdgesOrVetices && !this->DrawingTubes(CellBO, actor))
    // {
    //   vtkProperty *ppty = actor->GetProperty();
    //   float diffuseColor[3] = {0.0, 0.0, 0.0};
    //   float ambientColor[3];
    //   double *acol = ppty->GetEdgeColor();
    //   ambientColor[0] = acol[0];
    //   ambientColor[1] = acol[1];
    //   ambientColor[2] = acol[2];
    //   prog->SetUniform3f("diffuseColorUniform", diffuseColor);
    //   prog->SetUniform3f("ambientColorUniform", ambientColor);
    // }

    bool selecting = (this->CurrentSelector ? true : false);
    bool tpass = actor->IsRenderingTranslucentPolygonalGeometry();

    for (dataIter it = this->Data.begin(); it != this->Data.end(); ++it)
    {
      vtkCompositeMapperHelperData* starthdata = it->second;
      bool shouldDraw = starthdata->Visibility     // must be visible
        && (!selecting || starthdata->Pickability) // and pickable when selecting
        && (((selecting || starthdata->IsOpaque || actor->GetForceOpaque()) &&
              !tpass) // opaque during opaque or when selecting
             || ((!starthdata->IsOpaque || actor->GetForceTranslucent()) && tpass &&
                  !selecting)); // translucent during translucent and never selecting
      if (shouldDraw && starthdata->NextIndex[primType] > starthdata->StartIndex[primType])
      {
        // compilers think this can exceed the bounds so we also
        // test against primType even though we should not need to
        if (primType <= vtkOpenGLPolyDataMapper::PrimitiveTriStrips)
        {
          this->SetShaderValues(
            prog, starthdata, starthdata->CellCellMap->GetPrimitiveOffsets()[primType]);
        }

        unsigned int count = this->DrawingSelection
          ? static_cast<unsigned int>(CellBO.IBO->IndexCount)
          : starthdata->NextIndex[primType] - starthdata->StartIndex[primType];

        glDrawRangeElements(mode, static_cast<GLuint>(starthdata->StartVertex),
          static_cast<GLuint>(starthdata->NextVertex > 0 ? starthdata->NextVertex - 1 : 0), count,
          GL_UNSIGNED_INT,
          reinterpret_cast<const GLvoid*>(starthdata->StartIndex[primType] * sizeof(GLuint)));
      }
    }
    CellBO.IBO->Release();
  }
}

//------------------------------------------------------------------------------
void vtkCompositeMapperHelper2::RenderPieceDraw(vtkRenderer* ren, vtkActor* actor)
{
  int representation = actor->GetProperty()->GetRepresentation();

  // render points for point picking in a special way
  // all cell types should be rendered as points
  this->CurrentSelector = ren->GetSelector();
  bool pointPicking = false;
  if (this->CurrentSelector && this->PopulateSelectionSettings &&
    this->CurrentSelector->GetFieldAssociation() == vtkDataObject::FIELD_ASSOCIATION_POINTS)
  {
    representation = VTK_POINTS;
    pointPicking = true;
  }

  this->PrimitiveIDOffset = 0;

  // draw IBOs
  for (int i = vtkOpenGLPolyDataMapper::PrimitiveStart;
       i < (this->CurrentSelector ? vtkOpenGLPolyDataMapper::PrimitiveTriStrips + 1
                                  : vtkOpenGLPolyDataMapper::PrimitiveEnd);
       i++)
  {
    this->DrawingVertices = (i > vtkOpenGLPolyDataMapper::PrimitiveTriStrips ? true : false);
    this->DrawingSelection = false;
    GLenum mode = this->GetOpenGLMode(representation, i);
    this->DrawIBO(ren, actor, i, this->Primitives[i], mode,
      pointPicking ? this->GetPointPickingPrimitiveSize(i) : 0);
  }

  if (!this->CurrentSelector)
  {
    vtkSelection* sel = this->Parent->GetSelection();

    if (sel && sel->GetNumberOfNodes() > 0)
    {
      // draw selection IBOs
      for (int i = vtkOpenGLPolyDataMapper::PrimitiveStart;
           i <= vtkOpenGLPolyDataMapper::PrimitiveTriStrips; i++)
      {
        this->DrawingSelection = true;
        GLenum mode = this->GetOpenGLMode(this->SelectionType, i);
        this->DrawIBO(ren, actor, i, this->SelectionPrimitives[i], mode, 5);
      }
    }
  }

  if (this->CurrentSelector &&
    (this->CurrentSelector->GetCurrentPass() == vtkHardwareSelector::CELL_ID_LOW24 ||
      this->CurrentSelector->GetCurrentPass() == vtkHardwareSelector::CELL_ID_HIGH24))
  {
    this->CurrentSelector->SetPropColorValue(this->PrimitiveIDOffset);
  }
}

vtkCompositeMapperHelperData* vtkCompositeMapperHelper2::AddData(
  vtkPolyData* pd, unsigned int flatIndex)
{
  dataIter found = this->Data.find(pd);
  if (found == this->Data.end())
  {
    vtkCompositeMapperHelperData* hdata = new vtkCompositeMapperHelperData();
    hdata->FlatIndex = flatIndex;
    hdata->Data = pd;
    hdata->Marked = true;
    this->Data.insert(std::make_pair(pd, hdata));
    this->Modified();
    return hdata;
  }
  found->second->FlatIndex = flatIndex;
  found->second->Marked = true;
  return found->second;
}

//------------------------------------------------------------------------------
bool vtkCompositeMapperHelper2::GetNeedToRebuildBufferObjects(
  vtkRenderer* vtkNotUsed(ren), vtkActor* act)
{
  // Same as vtkOpenGLPolyDataMapper::GetNeedToRebuildBufferObjects(), but
  // we need to check all inputs, not just this->CurrentInput
  this->TempState.Clear();
  this->TempState.Append(act->GetProperty()->GetMTime(), "actor mtime");
  for (dataIter iter = this->Data.begin(); iter != this->Data.end(); ++iter)
  {
    this->TempState.Append(iter->first ? iter->first->GetMTime() : 0, "input mtime");
  }
  this->TempState.Append(act->GetTexture() ? act->GetTexture()->GetMTime() : 0, "texture mtime");

  if (this->VBOBuildState != this->TempState || this->VBOBuildTime < this->GetMTime())
  {
    this->VBOBuildState = this->TempState;
    return true;
  }

  return false;
}

//------------------------------------------------------------------------------
void vtkCompositeMapperHelper2::BuildBufferObjects(vtkRenderer* ren, vtkActor* act)
{
  // render using the composite data attributes

  // create the cell scalar array adjusted for ogl Cells
  std::vector<unsigned char> newColors;
  std::vector<float> newNorms;

  dataIter iter;
  this->VBOs->ClearAllVBOs();

  if (this->Data.begin() == this->Data.end())
  {
    this->VBOBuildTime.Modified();
    return;
  }

  this->EdgeValues.clear();

  vtkBoundingBox bbox;
  double bounds[6];
  this->Data.begin()->second->Data->GetPoints()->GetBounds(bounds);
  bbox.SetBounds(bounds);
  vtkCompositeMapperHelperData* prevhdata = nullptr;
  for (iter = this->Data.begin(); iter != this->Data.end(); ++iter)
  {
    vtkCompositeMapperHelperData* hdata = iter->second;

    hdata->Data->GetPoints()->GetBounds(bounds);
    bbox.AddBounds(bounds);

    for (int i = 0; i < vtkOpenGLPolyDataMapper::PrimitiveEnd; i++)
    {
      hdata->StartIndex[i] = static_cast<unsigned int>(this->IndexArray[i].size());
    }

    vtkIdType voffset = 0;
    // vert cell offset starts at the end of the last block
    hdata->CellCellMap->SetStartOffset(prevhdata ? prevhdata->CellCellMap->GetFinalOffset() : 0);
    this->AppendOneBufferObject(ren, act, hdata, voffset, newColors, newNorms);
    hdata->StartVertex = static_cast<unsigned int>(voffset);
    hdata->NextVertex = hdata->StartVertex + hdata->Data->GetPoints()->GetNumberOfPoints();
    for (int i = 0; i < vtkOpenGLPolyDataMapper::PrimitiveEnd; i++)
    {
      hdata->NextIndex[i] = static_cast<unsigned int>(this->IndexArray[i].size());
    }
    prevhdata = hdata;
  }

  // clear color cache
  for (auto& c : this->ColorArrayMap)
  {
    c.second->Delete();
  }
  this->ColorArrayMap.clear();

  vtkOpenGLVertexBufferObject* posVBO = this->VBOs->GetVBO("vertexMC");
  if (posVBO)
  {
    if (this->ShiftScaleMethod == vtkOpenGLVertexBufferObject::AUTO_SHIFT_SCALE)
    {
      posVBO->SetCoordShiftAndScaleMethod(vtkOpenGLVertexBufferObject::MANUAL_SHIFT_SCALE);
      bbox.GetBounds(bounds);
      std::vector<double> shift;
      std::vector<double> scale;
      for (int i = 0; i < 3; i++)
      {
        shift.push_back(0.5 * (bounds[i * 2] + bounds[i * 2 + 1]));
        scale.push_back(
          (bounds[i * 2 + 1] - bounds[i * 2]) ? 1.0 / (bounds[i * 2 + 1] - bounds[i * 2]) : 1.0);
      }
      posVBO->SetShift(shift);
      posVBO->SetScale(scale);
    }
    else
    {
      posVBO->SetCoordShiftAndScaleMethod(
        static_cast<vtkOpenGLVertexBufferObject::ShiftScaleMethod>(this->ShiftScaleMethod));
      posVBO->SetProp3D(act);
      posVBO->SetCamera(ren->GetActiveCamera());
    }
  }

  this->VBOs->BuildAllVBOs(ren);

  // refetch as it may have been deleted
  posVBO = this->VBOs->GetVBO("vertexMC");
  if (posVBO)
  {
    // If the VBO coordinates were shifted and scaled, prepare the inverse transform
    // for application to the model->view matrix:
    if (posVBO->GetCoordShiftAndScaleEnabled())
    {
      std::vector<double> const& shift = posVBO->GetShift();
      std::vector<double> const& scale = posVBO->GetScale();
      this->VBOInverseTransform->Identity();
      this->VBOInverseTransform->Translate(shift[0], shift[1], shift[2]);
      this->VBOInverseTransform->Scale(1.0 / scale[0], 1.0 / scale[1], 1.0 / scale[2]);
      this->VBOInverseTransform->GetTranspose(this->VBOShiftScale);
    }
  }

  for (int i = vtkOpenGLPolyDataMapper::PrimitiveStart; i < vtkOpenGLPolyDataMapper::PrimitiveEnd;
       i++)
  {
    this->Primitives[i].IBO->IndexCount = this->IndexArray[i].size();
    if (this->Primitives[i].IBO->IndexCount)
    {
      this->Primitives[i].IBO->Upload(
        this->IndexArray[i], vtkOpenGLBufferObject::ElementArrayBuffer);
      this->IndexArray[i].resize(0);
    }
  }

  if (!this->EdgeValues.empty())
  {
    if (!this->EdgeTexture)
    {
      this->EdgeTexture = vtkTextureObject::New();
      this->EdgeBuffer = vtkOpenGLBufferObject::New();
      this->EdgeBuffer->SetType(vtkOpenGLBufferObject::TextureBuffer);
    }
    this->EdgeTexture->SetContext(static_cast<vtkOpenGLRenderWindow*>(ren->GetVTKWindow()));
    this->EdgeBuffer->Upload(this->EdgeValues, vtkOpenGLBufferObject::TextureBuffer);
    this->EdgeTexture->CreateTextureBuffer(
      static_cast<unsigned int>(this->EdgeValues.size()), 1, VTK_UNSIGNED_CHAR, this->EdgeBuffer);
  }

  // allocate as needed
  if (this->HaveCellScalars)
  {
    if (!this->CellScalarTexture)
    {
      this->CellScalarTexture = vtkTextureObject::New();
      this->CellScalarBuffer = vtkOpenGLBufferObject::New();
    }
    this->CellScalarTexture->SetContext(static_cast<vtkOpenGLRenderWindow*>(ren->GetVTKWindow()));
    this->CellScalarBuffer->Upload(newColors, vtkOpenGLBufferObject::TextureBuffer);
    this->CellScalarTexture->CreateTextureBuffer(static_cast<unsigned int>(newColors.size() / 4), 4,
      VTK_UNSIGNED_CHAR, this->CellScalarBuffer);
  }

  if (this->HaveCellNormals)
  {
    if (!this->CellNormalTexture)
    {
      this->CellNormalTexture = vtkTextureObject::New();
      this->CellNormalBuffer = vtkOpenGLBufferObject::New();
      this->CellNormalBuffer->SetType(vtkOpenGLBufferObject::TextureBuffer);
    }
    this->CellNormalTexture->SetContext(static_cast<vtkOpenGLRenderWindow*>(ren->GetVTKWindow()));

    // do we have float texture support ?
    int ftex = static_cast<vtkOpenGLRenderWindow*>(ren->GetRenderWindow())
                 ->GetDefaultTextureInternalFormat(VTK_FLOAT, 4, false, true, false);

    if (ftex)
    {
      this->CellNormalBuffer->Upload(newNorms, vtkOpenGLBufferObject::TextureBuffer);
      this->CellNormalTexture->CreateTextureBuffer(
        static_cast<unsigned int>(newNorms.size() / 4), 4, VTK_FLOAT, this->CellNormalBuffer);
    }
    else
    {
      // have to convert to unsigned char if no float support
      std::vector<unsigned char> ucNewNorms;
      ucNewNorms.resize(newNorms.size());
      for (size_t i = 0; i < newNorms.size(); i++)
      {
        ucNewNorms[i] = 127.0 * (newNorms[i] + 1.0);
      }
      this->CellNormalBuffer->Upload(ucNewNorms, vtkOpenGLBufferObject::TextureBuffer);
      this->CellNormalTexture->CreateTextureBuffer(static_cast<unsigned int>(newNorms.size() / 4),
        4, VTK_UNSIGNED_CHAR, this->CellNormalBuffer);
    }
  }

  this->VBOBuildTime.Modified();
}

//------------------------------------------------------------------------------
void vtkCompositeMapperHelper2::BuildSelectionIBO(vtkPolyData* vtkNotUsed(poly),
  std::vector<unsigned int> (&indices)[4], vtkIdType vtkNotUsed(offset))
{
  for (auto& helper : this->Data)
  {
    this->Superclass::BuildSelectionIBO(helper.second->Data, indices, helper.second->StartVertex);
  }
}

//------------------------------------------------------------------------------
void vtkCompositeMapperHelper2::AppendOneBufferObject(vtkRenderer* ren, vtkActor* act,
  vtkCompositeMapperHelperData* hdata, vtkIdType& voffset, std::vector<unsigned char>& newColors,
  std::vector<float>& newNorms)
{
  vtkPolyData* poly = hdata->Data;

  // if there are no points then skip this piece
  if (!poly->GetPoints() || poly->GetPoints()->GetNumberOfPoints() == 0)
  {
    return;
  }

  // Get rid of old texture color coordinates if any
  if (this->ColorCoordinates)
  {
    this->ColorCoordinates->UnRegister(this);
    this->ColorCoordinates = nullptr;
  }
  // Get rid of old texture color coordinates if any
  if (this->Colors)
  {
    this->Colors->UnRegister(this);
    this->Colors = nullptr;
  }

  // For vertex coloring, this sets this->Colors as side effect.
  // For texture map coloring, this sets ColorCoordinates
  // and ColorTextureMap as a side effect.
  // I moved this out of the conditional because it is fast.
  // Color arrays are cached. If nothing has changed,
  // then the scalars do not have to be regenerted.
  this->MapScalars(poly, 1.0);

  // If we are coloring by texture, then load the texture map.
  if (this->ColorTextureMap)
  {
    if (this->InternalColorTexture == nullptr)
    {
      this->InternalColorTexture = vtkOpenGLTexture::New();
      this->InternalColorTexture->RepeatOff();
    }
    this->InternalColorTexture->SetInputData(this->ColorTextureMap);
  }

  this->HaveCellScalars = false;
  vtkDataArray* c = this->Colors;
  if (this->ScalarVisibility)
  {
    // We must figure out how the scalars should be mapped to the polydata.
    if ((this->ScalarMode == VTK_SCALAR_MODE_USE_CELL_DATA ||
          this->ScalarMode == VTK_SCALAR_MODE_USE_CELL_FIELD_DATA ||
          this->ScalarMode == VTK_SCALAR_MODE_USE_FIELD_DATA ||
          !poly->GetPointData()->GetScalars()) &&
      this->ScalarMode != VTK_SCALAR_MODE_USE_POINT_FIELD_DATA && this->Colors)
    {
      this->HaveCellScalars = true;
      c = nullptr;
    }
  }

  this->HaveCellNormals = false;
  // Do we have cell normals?
  vtkDataArray* n = (act->GetProperty()->GetInterpolation() != VTK_FLAT)
    ? poly->GetPointData()->GetNormals()
    : nullptr;
  if (n == nullptr && poly->GetCellData()->GetNormals())
  {
    this->HaveCellNormals = true;
    n = nullptr;
  }

  int representation = act->GetProperty()->GetRepresentation();
  vtkHardwareSelector* selector = ren->GetSelector();

  if (selector && this->PopulateSelectionSettings &&
    selector->GetFieldAssociation() == vtkDataObject::FIELD_ASSOCIATION_POINTS)
  {
    representation = VTK_POINTS;
  }

  // if we have cell scalars then we have to
  // explode the data
  vtkCellArray* prims[4];
  prims[0] = poly->GetVerts();
  prims[1] = poly->GetLines();
  prims[2] = poly->GetPolys();
  prims[3] = poly->GetStrips();

  // needs to get a cell call map passed in
  this->AppendCellTextures(
    ren, act, prims, representation, newColors, newNorms, poly, hdata->CellCellMap);

  hdata->CellCellMap->BuildPrimitiveOffsetsIfNeeded(prims, representation, poly->GetPoints());

  // do we have texture maps?
  bool haveTextures =
    (this->ColorTextureMap || act->GetTexture() || act->GetProperty()->GetNumberOfTextures());

  // Set the texture if we are going to use texture
  // for coloring with a point attribute.
  // fixme ... make the existence of the coordinate array the signal.
  vtkDataArray* tcoords = nullptr;
  if (haveTextures)
  {
    if (this->InterpolateScalarsBeforeMapping && this->ColorCoordinates)
    {
      tcoords = this->ColorCoordinates;
    }
    else
    {
      tcoords = poly->GetPointData()->GetTCoords();
    }
  }

  // Check if color array is already computed for the current array.
  // This step is mandatory otherwise the test ArrayExists will fail for "scalarColor" even if
  // the array used to map the color has already been added.
  if (c)
  {
    int cellFlag = 0; // not used
    vtkAbstractArray* abstractArray = vtkCompositeMapperHelper2::GetAbstractScalars(
      poly, this->ScalarMode, this->ArrayAccessMode, this->ArrayId, this->ArrayName, cellFlag);

    auto iter = this->ColorArrayMap.find(abstractArray);
    if (iter != this->ColorArrayMap.end())
    {
      c = iter->second;
    }
    else
    {
      this->ColorArrayMap[abstractArray] = c;
      c->Register(this);
    }
  }

  vtkFloatArray* tangents = vtkFloatArray::SafeDownCast(poly->GetPointData()->GetTangents());

  // Build the VBO
  vtkIdType offsetPos = 0;
  vtkIdType offsetNorm = 0;
  vtkIdType offsetColor = 0;
  vtkIdType offsetTex = 0;
  vtkIdType offsetTangents = 0;
  vtkIdType totalOffset = 0;
  vtkIdType dummy = 0;
  bool exists =
    this->VBOs->ArrayExists("vertexMC", poly->GetPoints()->GetData(), offsetPos, totalOffset) &&
    this->VBOs->ArrayExists("normalMC", n, offsetNorm, dummy) &&
    this->VBOs->ArrayExists("scalarColor", c, offsetColor, dummy) &&
    this->VBOs->ArrayExists("tcoord", tcoords, offsetTex, dummy) &&
    this->VBOs->ArrayExists("tangentMC", tangents, offsetTangents, dummy);

  // if all used arrays have the same offset and have already been added,
  // we can reuse them and save memory
  if (exists && (offsetNorm == 0 || offsetPos == offsetNorm) &&
    (offsetColor == 0 || offsetPos == offsetColor) && (offsetTex == 0 || offsetPos == offsetTex) &&
    (offsetTangents == 0 || offsetPos == offsetTangents))
  {
    voffset = offsetPos;
  }
  else
  {
    this->VBOs->AppendDataArray("vertexMC", poly->GetPoints()->GetData(), VTK_FLOAT);
    this->VBOs->AppendDataArray("normalMC", n, VTK_FLOAT);
    this->VBOs->AppendDataArray("scalarColor", c, VTK_UNSIGNED_CHAR);
    this->VBOs->AppendDataArray("tcoord", tcoords, VTK_FLOAT);
    this->VBOs->AppendDataArray("tangentMC", tangents, VTK_FLOAT);

    voffset = totalOffset;
  }

  // now create the IBOs
  vtkOpenGLIndexBufferObject::AppendPointIndexBuffer(this->IndexArray[0], prims[0], voffset);

  vtkDataArray* ef = poly->GetPointData()->GetAttribute(vtkDataSetAttributes::EDGEFLAG);
  if (ef)
  {
    if (ef->GetNumberOfComponents() != 1)
    {
      vtkDebugMacro(<< "Currently only 1d edge flags are supported.");
      ef = nullptr;
    }
    if (ef && !ef->IsA("vtkUnsignedCharArray"))
    {
      vtkDebugMacro(<< "Currently only unsigned char edge flags are supported.");
      ef = nullptr;
    }
  }

  vtkProperty* prop = act->GetProperty();
  bool draw_surface_with_edges =
    (prop->GetEdgeVisibility() && prop->GetRepresentation() == VTK_SURFACE);

  if (representation == VTK_POINTS)
  {
    vtkOpenGLIndexBufferObject::AppendPointIndexBuffer(this->IndexArray[1], prims[1], voffset);

    vtkOpenGLIndexBufferObject::AppendPointIndexBuffer(this->IndexArray[2], prims[2], voffset);

    vtkOpenGLIndexBufferObject::AppendPointIndexBuffer(this->IndexArray[3], prims[3], voffset);
  }
  else // WIREFRAME OR SURFACE
  {
    vtkOpenGLIndexBufferObject::AppendLineIndexBuffer(this->IndexArray[1], prims[1], voffset);

    if (representation == VTK_WIREFRAME)
    {
      if (ef)
      {
        vtkOpenGLIndexBufferObject::AppendEdgeFlagIndexBuffer(
          this->IndexArray[2], prims[2], voffset, ef);
      }
      else
      {
        vtkOpenGLIndexBufferObject::AppendTriangleLineIndexBuffer(
          this->IndexArray[2], prims[2], voffset);
      }
      vtkOpenGLIndexBufferObject::AppendStripIndexBuffer(
        this->IndexArray[3], prims[3], voffset, true);
    }
    else // SURFACE
    {
      if (draw_surface_with_edges)
      {
        // have to insert dummy values for points and lines
        vtkIdType* offsets = hdata->CellCellMap->GetPrimitiveOffsets();
        this->EdgeValues.resize(offsets[2], 0);
        vtkOpenGLIndexBufferObject::AppendTriangleIndexBuffer(
          this->IndexArray[2], prims[2], poly->GetPoints(), voffset, &this->EdgeValues, ef);
      }
      else
      {
        vtkOpenGLIndexBufferObject::AppendTriangleIndexBuffer(
          this->IndexArray[2], prims[2], poly->GetPoints(), voffset, nullptr, nullptr);
      }
      vtkOpenGLIndexBufferObject::AppendStripIndexBuffer(
        this->IndexArray[3], prims[3], voffset, false);
    }
  }

  if (prop->GetVertexVisibility())
  {
    vtkOpenGLIndexBufferObject::AppendVertexIndexBuffer(
      this->IndexArray[vtkOpenGLPolyDataMapper::PrimitiveVertices], prims, voffset);
  }
}

void vtkCompositeMapperHelper2::ProcessSelectorPixelBuffers(
  vtkHardwareSelector* sel, std::vector<unsigned int>& pixeloffsets, vtkProp* prop)
{
  if (!this->PopulateSelectionSettings)
  {
    return;
  }

  if (sel->GetCurrentPass() == vtkHardwareSelector::ACTOR_PASS)
  {
    this->PickPixels.clear();
    return;
  }

  if (PickPixels.empty() && !pixeloffsets.empty())
  {
    // preprocess the image to find matching pixels and
    // store them in a map of vectors based on flat index
    // this makes the block processing far faster as we just
    // loop over the pixels for our block
    unsigned char* compositedata =
      sel->GetRawPixelBuffer(vtkHardwareSelector::COMPOSITE_INDEX_PASS);

    if (!compositedata)
    {
      return;
    }

    size_t maxFlatIndex = 0;
    for (dataIter it = this->Data.begin(); it != this->Data.end(); ++it)
    {
      maxFlatIndex = (it->second->FlatIndex > maxFlatIndex) ? it->second->FlatIndex : maxFlatIndex;
    }

    this->PickPixels.resize(maxFlatIndex + 1);

    for (auto pos : pixeloffsets)
    {
      unsigned int compval = compositedata[pos + 2];
      compval = compval << 8;
      compval |= compositedata[pos + 1];
      compval = compval << 8;
      compval |= compositedata[pos];
      if (compval <= maxFlatIndex)
      {
        this->PickPixels[compval].push_back(pos);
      }
    }
  }

  // for each block update the image
  for (dataIter it = this->Data.begin(); it != this->Data.end(); ++it)
  {
    if (!this->PickPixels[it->second->FlatIndex].empty())
    {
      this->ProcessCompositePixelBuffers(
        sel, prop, it->second, this->PickPixels[it->second->FlatIndex]);
    }
  }
}

void vtkCompositeMapperHelper2::ProcessCompositePixelBuffers(vtkHardwareSelector* sel,
  vtkProp* prop, vtkCompositeMapperHelperData* hdata, std::vector<unsigned int>& pixeloffsets)
{
  vtkPolyData* poly = hdata->Data;

  if (!poly)
  {
    return;
  }

  // which pass are we processing ?
  int currPass = sel->GetCurrentPass();

  // get some common useful values
  bool pointPicking = sel->GetFieldAssociation() == vtkDataObject::FIELD_ASSOCIATION_POINTS;
  vtkPointData* pd = poly->GetPointData();
  vtkCellData* cd = poly->GetCellData();

  // get some values
  unsigned char* rawplowdata = sel->GetRawPixelBuffer(vtkHardwareSelector::POINT_ID_LOW24);
  unsigned char* rawphighdata = sel->GetRawPixelBuffer(vtkHardwareSelector::POINT_ID_HIGH24);

  // do we need to do anything to the process pass data?
  if (currPass == vtkHardwareSelector::PROCESS_PASS)
  {
    unsigned char* processdata = sel->GetPixelBuffer(vtkHardwareSelector::PROCESS_PASS);
    vtkUnsignedIntArray* processArray = nullptr;

    if (sel->GetUseProcessIdFromData())
    {
      processArray = this->ProcessIdArrayName
        ? vtkArrayDownCast<vtkUnsignedIntArray>(pd->GetArray(this->ProcessIdArrayName))
        : nullptr;
    }

    if (processArray && processdata && rawplowdata)
    {
      for (auto pos : pixeloffsets)
      {
        unsigned int inval = 0;
        if (rawphighdata)
        {
          inval = rawphighdata[pos];
          inval = inval << 8;
        }
        inval |= rawplowdata[pos + 2];
        inval = inval << 8;
        inval |= rawplowdata[pos + 1];
        inval = inval << 8;
        inval |= rawplowdata[pos];
        // as this pass happens after both low and high point passes
        // the computed value should be higher than StartVertex
        inval -= hdata->StartVertex;
        unsigned int outval = processArray->GetValue(inval) + 1;
        processdata[pos] = outval & 0xff;
        processdata[pos + 1] = (outval & 0xff00) >> 8;
        processdata[pos + 2] = (outval & 0xff0000) >> 16;
      }
    }
  }

  // do we need to do anything to the point id data?
  if (currPass == vtkHardwareSelector::POINT_ID_LOW24)
  {
    vtkIdTypeArray* pointArrayId = this->PointIdArrayName
      ? vtkArrayDownCast<vtkIdTypeArray>(pd->GetArray(this->PointIdArrayName))
      : nullptr;

    // do we need to do anything to the point id data?
    if (rawplowdata)
    {
      unsigned char* plowdata = sel->GetPixelBuffer(vtkHardwareSelector::POINT_ID_LOW24);
      bool hasHighPointIds = sel->HasHighPointIds();

      for (auto pos : pixeloffsets)
      {
        unsigned int inval = 0;
        if (rawphighdata)
        {
          inval = rawphighdata[pos];
          inval = inval << 8;
        }
        inval |= rawplowdata[pos + 2];
        inval = inval << 8;
        inval |= rawplowdata[pos + 1];
        inval = inval << 8;
        inval |= rawplowdata[pos];
        // this pass happens before the high pass which means the value
        // could underflow etc when the high data is not around yet and high
        // data is needed.
        if (rawphighdata || !hasHighPointIds)
        {
          inval -= hdata->StartVertex;
          vtkIdType outval = inval;
          if (pointArrayId && static_cast<vtkIdType>(inval) <= pointArrayId->GetMaxId())
          {
            outval = pointArrayId->GetValue(inval);
          }
          plowdata[pos] = outval & 0xff;
          plowdata[pos + 1] = (outval & 0xff00) >> 8;
          plowdata[pos + 2] = (outval & 0xff0000) >> 16;
        }
      }
    }
  }

  if (currPass == vtkHardwareSelector::POINT_ID_HIGH24)
  {
    vtkIdTypeArray* pointArrayId = this->PointIdArrayName
      ? vtkArrayDownCast<vtkIdTypeArray>(pd->GetArray(this->PointIdArrayName))
      : nullptr;

    // do we need to do anything to the point id data?
    if (rawphighdata)
    {
      unsigned char* phighdata = sel->GetPixelBuffer(vtkHardwareSelector::POINT_ID_HIGH24);

      for (auto pos : pixeloffsets)
      {
        unsigned int inval = 0;
        inval = rawphighdata[pos];
        inval = inval << 8;
        inval |= rawplowdata[pos + 2];
        inval = inval << 8;
        inval |= rawplowdata[pos + 1];
        inval = inval << 8;
        inval |= rawplowdata[pos];
        // always happens after the low pass so we should be safe
        inval -= hdata->StartVertex;
        vtkIdType outval = inval;
        if (pointArrayId)
        {
          outval = pointArrayId->GetValue(inval);
        }
        phighdata[pos] = (outval & 0xff000000) >> 24;
        phighdata[pos + 1] = (outval & 0xff00000000) >> 32;
        phighdata[pos + 2] = (outval & 0xff0000000000) >> 40;
      }
    }
  }

  // vars for cell based indexing
  vtkCellArray* prims[4];
  prims[0] = poly->GetVerts();
  prims[1] = poly->GetLines();
  prims[2] = poly->GetPolys();
  prims[3] = poly->GetStrips();

  int representation = static_cast<vtkActor*>(prop)->GetProperty()->GetRepresentation();

  unsigned char* rawclowdata = sel->GetRawPixelBuffer(vtkHardwareSelector::CELL_ID_LOW24);
  unsigned char* rawchighdata = sel->GetRawPixelBuffer(vtkHardwareSelector::CELL_ID_HIGH24);

  // do we need to do anything to the composite pass data?
  if (currPass == vtkHardwareSelector::COMPOSITE_INDEX_PASS)
  {
    unsigned char* compositedata = sel->GetPixelBuffer(vtkHardwareSelector::COMPOSITE_INDEX_PASS);

    vtkUnsignedIntArray* compositeArray = this->CompositeIdArrayName
      ? vtkArrayDownCast<vtkUnsignedIntArray>(cd->GetArray(this->CompositeIdArrayName))
      : nullptr;

    if (compositedata && compositeArray && rawclowdata)
    {
      hdata->CellCellMap->Update(prims, representation, poly->GetPoints());

      for (auto pos : pixeloffsets)
      {
        unsigned int inval = 0;
        if (rawchighdata)
        {
          inval = rawchighdata[pos];
          inval = inval << 8;
        }
        inval |= rawclowdata[pos + 2];
        inval = inval << 8;
        inval |= rawclowdata[pos + 1];
        inval = inval << 8;
        inval |= rawclowdata[pos];

        // always gets called after the cell high and low are available
        // so it is safe
        vtkIdType vtkCellId =
          hdata->CellCellMap->ConvertOpenGLCellIdToVTKCellId(pointPicking, inval);
        unsigned int outval = compositeArray->GetValue(vtkCellId);
        compositedata[pos] = outval & 0xff;
        compositedata[pos + 1] = (outval & 0xff00) >> 8;
        compositedata[pos + 2] = (outval & 0xff0000) >> 16;
      }
    }
  }

  if (currPass == vtkHardwareSelector::CELL_ID_LOW24)
  {
    vtkIdTypeArray* cellArrayId = this->CellIdArrayName
      ? vtkArrayDownCast<vtkIdTypeArray>(cd->GetArray(this->CellIdArrayName))
      : nullptr;
    unsigned char* clowdata = sel->GetPixelBuffer(vtkHardwareSelector::CELL_ID_LOW24);
    bool hasHighCellIds = sel->HasHighCellIds();

    if (rawclowdata)
    {
      hdata->CellCellMap->Update(prims, representation, poly->GetPoints());

      for (auto pos : pixeloffsets)
      {
        unsigned int inval = 0;
        if (rawchighdata)
        {
          inval = rawchighdata[pos];
          inval = inval << 8;
        }
        inval |= rawclowdata[pos + 2];
        inval = inval << 8;
        inval |= rawclowdata[pos + 1];
        inval = inval << 8;
        inval |= rawclowdata[pos];
        // this pass happens before the high pass which means the value
        // could underflow etc when the high data is not around yet and high
        // data is needed. This underflow would happen in the ConvertToOpenGLCellId
        // code when passed too low a number
        if (rawchighdata || !hasHighCellIds)
        {
          vtkIdType outval =
            hdata->CellCellMap->ConvertOpenGLCellIdToVTKCellId(pointPicking, inval);
          if (cellArrayId && outval <= cellArrayId->GetMaxId())
          {
            outval = cellArrayId->GetValue(outval);
          }
          clowdata[pos] = outval & 0xff;
          clowdata[pos + 1] = (outval & 0xff00) >> 8;
          clowdata[pos + 2] = (outval & 0xff0000) >> 16;
        }
      }
    }
  }

  if (currPass == vtkHardwareSelector::CELL_ID_HIGH24)
  {
    vtkIdTypeArray* cellArrayId = this->CellIdArrayName
      ? vtkArrayDownCast<vtkIdTypeArray>(cd->GetArray(this->CellIdArrayName))
      : nullptr;
    unsigned char* chighdata = sel->GetPixelBuffer(vtkHardwareSelector::CELL_ID_HIGH24);

    if (rawchighdata)
    {
      hdata->CellCellMap->Update(prims, representation, poly->GetPoints());

      for (auto pos : pixeloffsets)
      {
        unsigned int inval = 0;
        inval = rawchighdata[pos];
        inval = inval << 8;
        inval |= rawclowdata[pos + 2];
        inval = inval << 8;
        inval |= rawclowdata[pos + 1];
        inval = inval << 8;
        inval |= rawclowdata[pos];
        // always called after low24 so safe
        vtkIdType outval = hdata->CellCellMap->ConvertOpenGLCellIdToVTKCellId(pointPicking, inval);
        if (cellArrayId)
        {
          outval = cellArrayId->GetValue(outval);
        }
        chighdata[pos] = (outval & 0xff000000) >> 24;
        chighdata[pos + 1] = (outval & 0xff00000000) >> 32;
        chighdata[pos + 2] = (outval & 0xff0000000000) >> 40;
      }
    }
  }
}

//===================================================================
// Now the main class methods

vtkStandardNewMacro(vtkCompositePolyDataMapper2);
//------------------------------------------------------------------------------
vtkCompositePolyDataMapper2::vtkCompositePolyDataMapper2()
{
  this->CurrentFlatIndex = 0;
  this->ColorMissingArraysWithNanColor = false;
}

//------------------------------------------------------------------------------
vtkCompositePolyDataMapper2::~vtkCompositePolyDataMapper2()
{
  helpIter miter = this->Helpers.begin();
  for (; miter != this->Helpers.end(); ++miter)
  {
    miter->second->Delete();
  }
  this->Helpers.clear();
}

//------------------------------------------------------------------------------
int vtkCompositePolyDataMapper2::FillInputPortInformation(
  int vtkNotUsed(port), vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
  info->Append(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkCompositeDataSet");
  return 1;
}

//------------------------------------------------------------------------------
vtkExecutive* vtkCompositePolyDataMapper2::CreateDefaultExecutive()
{
  return vtkCompositeDataPipeline::New();
}

//------------------------------------------------------------------------------
// Looks at each DataSet and finds the union of all the bounds
void vtkCompositePolyDataMapper2::ComputeBounds()
{
  vtkCompositeDataSet* input = vtkCompositeDataSet::SafeDownCast(this->GetInputDataObject(0, 0));

  // If we don't have hierarchical data, test to see if we have
  // plain old polydata. In this case, the bounds are simply
  // the bounds of the input polydata.
  if (!input)
  {
    this->Superclass::ComputeBounds();
    return;
  }

  if (input->GetMTime() < this->BoundsMTime && this->GetMTime() < this->BoundsMTime)
  {
    return;
  }

  // computing bounds with only visible blocks
  vtkCompositeDataDisplayAttributes::ComputeVisibleBounds(
    this->CompositeAttributes, input, this->Bounds);
  this->BoundsMTime.Modified();
}

//------------------------------------------------------------------------------
// simple tests, the mapper is tolerant of being
// called both on opaque and translucent
bool vtkCompositePolyDataMapper2::HasOpaqueGeometry()
{
  return true;
}

bool vtkCompositePolyDataMapper2::RecursiveHasTranslucentGeometry(
  vtkDataObject* dobj, unsigned int& flat_index)
{
  vtkCompositeDataDisplayAttributes* cda = this->GetCompositeDataDisplayAttributes();
  bool overrides_visibility = (cda && cda->HasBlockVisibility(dobj));
  if (overrides_visibility)
  {
    if (!cda->GetBlockVisibility(dobj))
    {
      return false;
    }
  }
  bool overrides_opacity = (cda && cda->HasBlockOpacity(dobj));
  if (overrides_opacity)
  {
    if (cda->GetBlockOpacity(dobj) < 1.0)
    {
      return true;
    }
  }

  // Advance flat-index. After this point, flat_index no longer points to this
  // block.
  flat_index++;

  auto dObjTree = vtkDataObjectTree::SafeDownCast(dobj);
  if (dObjTree)
  {
    using Opts = vtk::DataObjectTreeOptions;
    for (vtkDataObject* child : vtk::Range(dObjTree, Opts::None))
    {
      if (!child)
      {
        ++flat_index;
      }
      else
      {
        if (this->RecursiveHasTranslucentGeometry(child, flat_index))
        {
          return true;
        }
      }
    }
    return false;
  }

  vtkPolyData* pd = vtkPolyData::SafeDownCast(dobj);
  // if we think it is opaque check the scalars
  if (this->ScalarVisibility)
  {
    vtkScalarsToColors* lut = this->GetLookupTable();
    int cellFlag;
    vtkDataArray* scalars = vtkCompositePolyDataMapper2::GetScalars(
      pd, this->ScalarMode, this->ArrayAccessMode, this->ArrayId, this->ArrayName, cellFlag);
    if (lut->IsOpaque(scalars, this->ColorMode, this->ArrayComponent) == 0)
    {
      return true;
    }
  }

  return false;
}

//------------------------------------------------------------------------------
// simple tests, the mapper is tolerant of being
// called both on opaque and translucent
bool vtkCompositePolyDataMapper2::HasTranslucentPolygonalGeometry()
{
  // Make sure that we have been properly initialized.
  if (this->GetInputAlgorithm() == nullptr)
  {
    return false;
  }

  if (!this->Static)
  {
    this->InvokeEvent(vtkCommand::StartEvent, nullptr);
    this->GetInputAlgorithm()->Update();
    this->InvokeEvent(vtkCommand::EndEvent, nullptr);
  }

  if (this->GetInputDataObject(0, 0) == nullptr)
  {
    return false;
  }

  // rebuild the render values if needed
  vtkCompositeDataDisplayAttributes* cda = this->GetCompositeDataDisplayAttributes();
  vtkScalarsToColors* lut = this->ScalarVisibility ? this->GetLookupTable() : nullptr;

  this->TempState.Clear();
  this->TempState.Append(cda ? cda->GetMTime() : 0, "cda mtime");
  this->TempState.Append(lut ? lut->GetMTime() : 0, "lut mtime");
  this->TempState.Append(this->GetInputDataObject(0, 0)->GetMTime(), "input mtime");
  if (this->TranslucentState != this->TempState)
  {
    this->TranslucentState = this->TempState;
    if (lut)
    {
      // Ensure that the lookup table is built
      lut->Build();
    }

    // Push base-values on the state stack.
    unsigned int flat_index = 0;
    this->HasTranslucentGeometry =
      this->RecursiveHasTranslucentGeometry(this->GetInputDataObject(0, 0), flat_index);
  }

  return this->HasTranslucentGeometry;
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::SetBlockVisibility(unsigned int index, bool visible)
{
  if (this->CompositeAttributes)
  {
    auto dataObj =
      vtkCompositeDataDisplayAttributes::DataObjectFromIndex(index, this->GetInputDataObject(0, 0));
    if (dataObj)
    {
      this->CompositeAttributes->SetBlockVisibility(dataObj, visible);
      this->Modified();
    }
  }
}

//------------------------------------------------------------------------------
bool vtkCompositePolyDataMapper2::GetBlockVisibility(unsigned int index)
{
  if (this->CompositeAttributes)
  {
    auto dataObj =
      vtkCompositeDataDisplayAttributes::DataObjectFromIndex(index, this->GetInputDataObject(0, 0));
    if (dataObj)
    {
      return this->CompositeAttributes->GetBlockVisibility(dataObj);
    }
  }

  return true;
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::RemoveBlockVisibility(unsigned int index)
{
  if (this->CompositeAttributes)
  {
    auto dataObj =
      vtkCompositeDataDisplayAttributes::DataObjectFromIndex(index, this->GetInputDataObject(0, 0));
    if (dataObj)
    {
      this->CompositeAttributes->RemoveBlockVisibility(dataObj);
      this->Modified();
    }
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::RemoveBlockVisibilities()
{
  if (this->CompositeAttributes)
  {
    this->CompositeAttributes->RemoveBlockVisibilities();
    this->Modified();
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::SetBlockColor(unsigned int index, const double color[3])
{
  if (this->CompositeAttributes)
  {
    auto dataObj =
      vtkCompositeDataDisplayAttributes::DataObjectFromIndex(index, this->GetInputDataObject(0, 0));

    if (dataObj)
    {
      this->CompositeAttributes->SetBlockColor(dataObj, color);
      this->Modified();
    }
  }
}

//------------------------------------------------------------------------------
double* vtkCompositePolyDataMapper2::GetBlockColor(unsigned int index)
{
  static double white[3] = { 1.0, 1.0, 1.0 };

  if (this->CompositeAttributes)
  {
    unsigned int start_index = 0;
    auto dataObj = vtkCompositeDataDisplayAttributes::DataObjectFromIndex(
      index, this->GetInputDataObject(0, 0), start_index);
    if (dataObj)
    {
      this->CompositeAttributes->GetBlockColor(dataObj, this->ColorResult);
    }

    return this->ColorResult;
  }
  else
  {
    return white;
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::RemoveBlockColor(unsigned int index)
{
  if (this->CompositeAttributes)
  {
    unsigned int start_index = 0;
    auto dataObj = vtkCompositeDataDisplayAttributes::DataObjectFromIndex(
      index, this->GetInputDataObject(0, 0), start_index);
    if (dataObj)
    {
      this->CompositeAttributes->RemoveBlockColor(dataObj);
      this->Modified();
    }
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::RemoveBlockColors()
{
  if (this->CompositeAttributes)
  {
    this->CompositeAttributes->RemoveBlockColors();
    this->Modified();
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::SetBlockOpacity(unsigned int index, double opacity)
{
  if (this->CompositeAttributes)
  {
    unsigned int start_index = 0;
    auto dataObj = vtkCompositeDataDisplayAttributes::DataObjectFromIndex(
      index, this->GetInputDataObject(0, 0), start_index);
    if (dataObj)
    {
      this->CompositeAttributes->SetBlockOpacity(dataObj, opacity);
      this->Modified();
    }
  }
}

//------------------------------------------------------------------------------
double vtkCompositePolyDataMapper2::GetBlockOpacity(unsigned int index)
{
  if (this->CompositeAttributes)
  {
    unsigned int start_index = 0;
    auto dataObj = vtkCompositeDataDisplayAttributes::DataObjectFromIndex(
      index, this->GetInputDataObject(0, 0), start_index);
    if (dataObj)
    {
      return this->CompositeAttributes->GetBlockOpacity(dataObj);
    }
  }
  return 1.;
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::RemoveBlockOpacity(unsigned int index)
{
  if (this->CompositeAttributes)
  {
    unsigned int start_index = 0;
    auto dataObj = vtkCompositeDataDisplayAttributes::DataObjectFromIndex(
      index, this->GetInputDataObject(0, 0), start_index);
    if (dataObj)
    {
      this->CompositeAttributes->RemoveBlockOpacity(dataObj);
      this->Modified();
    }
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::RemoveBlockOpacities()
{
  if (this->CompositeAttributes)
  {
    this->CompositeAttributes->RemoveBlockOpacities();
    this->Modified();
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::SetCompositeDataDisplayAttributes(
  vtkCompositeDataDisplayAttributes* attributes)
{
  if (this->CompositeAttributes != attributes)
  {
    this->CompositeAttributes = attributes;
    this->Modified();
  }
}

//------------------------------------------------------------------------------
vtkCompositeDataDisplayAttributes* vtkCompositePolyDataMapper2::GetCompositeDataDisplayAttributes()
{
  return this->CompositeAttributes;
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

void vtkCompositePolyDataMapper2::CopyMapperValuesToHelper(vtkCompositeMapperHelper2* helper)
{
  // We avoid PolyDataMapper::ShallowCopy because it copies the input
  helper->vtkMapper::ShallowCopy(this);
  helper->SetPointIdArrayName(this->GetPointIdArrayName());
  helper->SetCompositeIdArrayName(this->GetCompositeIdArrayName());
  helper->SetProcessIdArrayName(this->GetProcessIdArrayName());
  helper->SetCellIdArrayName(this->GetCellIdArrayName());
  helper->SetSeamlessU(this->SeamlessU);
  helper->SetSeamlessV(this->SeamlessV);
  helper->SetStatic(1);
  helper->SetSelection(this->GetSelection());
  helper->SetVBOShiftScaleMethod(this->GetVBOShiftScaleMethod());
}

void vtkCompositePolyDataMapper2::SetVBOShiftScaleMethod(int m)
{
  if (this->ShiftScaleMethod == m)
  {
    return;
  }

  this->Superclass::SetVBOShiftScaleMethod(m);

  for (helpIter hiter = this->Helpers.begin(); hiter != this->Helpers.end(); ++hiter)
  {
    hiter->second->SetVBOShiftScaleMethod(m);
  }
}

void vtkCompositePolyDataMapper2::SetPauseShiftScale(bool pauseShiftScale)
{
  if (pauseShiftScale == this->PauseShiftScale)
  {
    return;
  }

  this->Superclass::SetPauseShiftScale(pauseShiftScale);
  for (helpIter hiter = this->Helpers.begin(); hiter != this->Helpers.end(); ++hiter)
  {
    hiter->second->SetPauseShiftScale(pauseShiftScale);
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::ReleaseGraphicsResources(vtkWindow* win)
{
  helpIter miter = this->Helpers.begin();
  for (; miter != this->Helpers.end(); ++miter)
  {
    miter->second->ReleaseGraphicsResources(win);
  }
  miter = this->Helpers.begin();
  for (; miter != this->Helpers.end(); ++miter)
  {
    miter->second->Delete();
  }
  this->Helpers.clear();
  this->Modified();
  this->Superclass::ReleaseGraphicsResources(win);
}

//------------------------------------------------------------------------------
// Description:
// Method initiates the mapping process. Generally sent by the actor
// as each frame is rendered.
void vtkCompositePolyDataMapper2::Render(vtkRenderer* ren, vtkActor* actor)
{
  this->RenderedList.clear();

  // Make sure that we have been properly initialized.
  if (ren->GetRenderWindow()->CheckAbortStatus())
  {
    return;
  }

  if (this->GetInputAlgorithm() == nullptr)
  {
    return;
  }

  if (!this->Static)
  {
    this->InvokeEvent(vtkCommand::StartEvent, nullptr);
    this->GetInputAlgorithm()->Update();
    this->InvokeEvent(vtkCommand::EndEvent, nullptr);
  }

  if (this->GetInputDataObject(0, 0) == nullptr)
  {
    vtkErrorMacro(<< "No input!");
    return;
  }

  // the first step is to gather up the polydata based on their
  // signatures (aka have normals, have scalars etc)
  if (this->HelperMTime < this->GetInputDataObject(0, 0)->GetMTime() ||
    this->HelperMTime < this->GetMTime())
  {
    // clear old helpers
    for (helpIter hiter = this->Helpers.begin(); hiter != this->Helpers.end(); ++hiter)
    {
      hiter->second->ClearMark();
    }
    this->HelperDataMap.clear();

    vtkCompositeDataSet* input = vtkCompositeDataSet::SafeDownCast(this->GetInputDataObject(0, 0));

    if (input)
    {
      vtkSmartPointer<vtkDataObjectTreeIterator> iter =
        vtkSmartPointer<vtkDataObjectTreeIterator>::New();
      iter->SetDataSet(input);
      iter->SkipEmptyNodesOn();
      iter->VisitOnlyLeavesOn();
      for (iter->InitTraversal(); !iter->IsDoneWithTraversal(); iter->GoToNextItem())
      {
        unsigned int flatIndex = iter->GetCurrentFlatIndex();
        vtkDataObject* dso = iter->GetCurrentDataObject();
        vtkPolyData* pd = vtkPolyData::SafeDownCast(dso);

        if (!pd || !pd->GetPoints())
        {
          continue;
        }
        int cellFlag = 0;
        bool hasScalars = this->ScalarVisibility &&
          (vtkAbstractMapper::GetAbstractScalars(pd, this->ScalarMode, this->ArrayAccessMode,
             this->ArrayId, this->ArrayName, cellFlag) != nullptr);

        bool hasNormals = (pd->GetPointData()->GetNormals() || pd->GetCellData()->GetNormals());

        bool hasTCoords = (pd->GetPointData()->GetTCoords() != nullptr);

        std::ostringstream toString;
        toString.str("");
        toString.clear();
        toString << 'A' << (hasScalars ? 1 : 0) << 'B' << (hasNormals ? 1 : 0) << 'C'
                 << (hasTCoords ? 1 : 0);

        vtkCompositeMapperHelper2* helper = nullptr;
        helpIter found = this->Helpers.find(toString.str());
        if (found == this->Helpers.end())
        {
          helper = this->CreateHelper();
          helper->SetParent(this);
          this->Helpers.insert(std::make_pair(toString.str(), helper));
        }
        else
        {
          helper = found->second;
        }
        this->CopyMapperValuesToHelper(helper);
        helper->SetMarked(true);
        this->HelperDataMap[pd] = helper->AddData(pd, flatIndex);
      }
    }
    else
    {
      vtkPolyData* pd = vtkPolyData::SafeDownCast(this->GetInputDataObject(0, 0));
      if (pd && pd->GetPoints())
      {
        int cellFlag = 0;
        bool hasScalars = this->ScalarVisibility &&
          (vtkAbstractMapper::GetAbstractScalars(pd, this->ScalarMode, this->ArrayAccessMode,
             this->ArrayId, this->ArrayName, cellFlag) != nullptr);

        bool hasNormals = (pd->GetPointData()->GetNormals() || pd->GetCellData()->GetNormals());

        bool hasTCoords = (pd->GetPointData()->GetTCoords() != nullptr);

        std::ostringstream toString;
        toString.str("");
        toString.clear();
        toString << 'A' << (hasScalars ? 1 : 0) << 'B' << (hasNormals ? 1 : 0) << 'C'
                 << (hasTCoords ? 1 : 0);

        vtkCompositeMapperHelper2* helper = nullptr;
        helpIter found = this->Helpers.find(toString.str());
        if (found == this->Helpers.end())
        {
          helper = this->CreateHelper();
          helper->SetParent(this);
          this->Helpers.insert(std::make_pair(toString.str(), helper));
        }
        else
        {
          helper = found->second;
        }
        this->CopyMapperValuesToHelper(helper);
        helper->SetMarked(true);
        this->HelperDataMap[pd] = helper->AddData(pd, 0);
      }
    }

    // delete unused old helpers/data
    for (helpIter hiter = this->Helpers.begin(); hiter != this->Helpers.end();)
    {
      hiter->second->RemoveUnused();
      if (!hiter->second->GetMarked())
      {
        hiter->second->ReleaseGraphicsResources(ren->GetVTKWindow());
        hiter->second->Delete();
        this->Helpers.erase(hiter++);
      }
      else
      {
        ++hiter;
      }
    }
    this->HelperMTime.Modified();
  }

  // rebuild the render values if needed
  this->TempState.Clear();
  this->TempState.Append(actor->GetProperty()->GetMTime(), "actor mtime");
  this->TempState.Append(this->GetMTime(), "this mtime");
  this->TempState.Append(this->HelperMTime, "helper mtime");
  this->TempState.Append(
    actor->GetTexture() ? actor->GetTexture()->GetMTime() : 0, "texture mtime");
  if (this->RenderValuesState != this->TempState)
  {
    this->RenderValuesState = this->TempState;
    vtkProperty* prop = actor->GetProperty();
    vtkScalarsToColors* lut = this->GetLookupTable();
    if (lut)
    {
      // Ensure that the lookup table is built
      lut->Build();
    }

    double* selColor = prop->GetSelectionColor();

    // Push base-values on the state stack.
    this->BlockState.Visibility.push(true);
    this->BlockState.Pickability.push(true);
    this->BlockState.Opacity.push(prop->GetOpacity());
    this->BlockState.AmbientColor.push(vtkColor3d(prop->GetAmbientColor()));
    this->BlockState.DiffuseColor.push(vtkColor3d(prop->GetDiffuseColor()));
    this->BlockState.SpecularColor.push(vtkColor3d(prop->GetSpecularColor()));
    this->BlockState.SelectionColor.push(vtkColor3d(selColor));
    this->BlockState.SelectionOpacity.push(selColor[3]);

    unsigned int flat_index = 0;
    this->BuildRenderValues(ren, actor, this->GetInputDataObject(0, 0), flat_index);

    this->BlockState.Visibility.pop();
    this->BlockState.Pickability.pop();
    this->BlockState.Opacity.pop();
    this->BlockState.AmbientColor.pop();
    this->BlockState.DiffuseColor.pop();
    this->BlockState.SpecularColor.pop();
    this->BlockState.SelectionColor.pop();
    this->BlockState.SelectionOpacity.pop();
  }

  this->InitializeHelpersBeforeRendering(ren, actor);

  for (helpIter hiter = this->Helpers.begin(); hiter != this->Helpers.end(); ++hiter)
  {
    vtkCompositeMapperHelper2* helper = hiter->second;
    helper->RenderPiece(ren, actor);

    // update the list of rendered polydata that vtkValuePass relies on
    std::vector<vtkPolyData*> pdl = helper->GetRenderedList();
    for (unsigned int i = 0; i < pdl.size(); ++i)
    {
      this->RenderedList.push_back(pdl[i]);
    }
  }
}

vtkCompositeMapperHelper2* vtkCompositePolyDataMapper2::CreateHelper()
{
  return vtkCompositeMapperHelper2::New();
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::BuildRenderValues(
  vtkRenderer* renderer, vtkActor* actor, vtkDataObject* dobj, unsigned int& flat_index)
{
  vtkCompositeDataDisplayAttributes* cda = this->GetCompositeDataDisplayAttributes();
  bool overrides_visibility = (cda && cda->HasBlockVisibility(dobj));
  if (overrides_visibility)
  {
    this->BlockState.Visibility.push(cda->GetBlockVisibility(dobj));
  }
  bool overrides_pickability = (cda && cda->HasBlockPickability(dobj));
  if (overrides_pickability)
  {
    this->BlockState.Pickability.push(cda->GetBlockPickability(dobj));
  }

  bool overrides_opacity = (cda && cda->HasBlockOpacity(dobj));
  if (overrides_opacity)
  {
    this->BlockState.Opacity.push(cda->GetBlockOpacity(dobj));
  }

  bool overrides_color = (cda && cda->HasBlockColor(dobj));
  if (overrides_color)
  {
    vtkColor3d color = cda->GetBlockColor(dobj);
    this->BlockState.AmbientColor.push(color);
    this->BlockState.DiffuseColor.push(color);
    this->BlockState.SpecularColor.push(color);
  }

  // Advance flat-index. After this point, flat_index no longer points to this
  // block.
  flat_index++;

  bool textureOpaque = true;
  if (actor->GetTexture() != nullptr && actor->GetTexture()->IsTranslucent())
  {
    textureOpaque = false;
  }

  auto dObjTree = vtkDataObjectTree::SafeDownCast(dobj);
  if (dObjTree)
  {
    using Opts = vtk::DataObjectTreeOptions;
    for (vtkDataObject* child : vtk::Range(dObjTree, Opts::None))
    {
      if (!child)
      {
        ++flat_index;
      }
      else
      {
        this->BuildRenderValues(renderer, actor, child, flat_index);
      }
    }
  }
  else
  {
    vtkPolyData* pd = vtkPolyData::SafeDownCast(dobj);
    dataIter dit = this->HelperDataMap.find(pd);
    if (dit != this->HelperDataMap.end())
    {
      vtkCompositeMapperHelperData* helperData = dit->second;
      helperData->Opacity = this->BlockState.Opacity.top();
      helperData->Visibility = this->BlockState.Visibility.top();
      helperData->Pickability = this->BlockState.Pickability.top();
      helperData->AmbientColor = this->BlockState.AmbientColor.top();
      helperData->DiffuseColor = this->BlockState.DiffuseColor.top();
      helperData->SelectionColor = this->BlockState.SelectionColor.top();
      helperData->SelectionOpacity = this->BlockState.SelectionOpacity.top();
      helperData->OverridesColor = (this->BlockState.AmbientColor.size() > 1);
      helperData->IsOpaque = (helperData->Opacity >= 1.0) ? textureOpaque : false;
      // if we think it is opaque check the scalars
      if (helperData->IsOpaque && this->ScalarVisibility)
      {
        vtkScalarsToColors* lut = this->GetLookupTable();
        int cellFlag;
        vtkDataArray* scalars = vtkCompositePolyDataMapper2::GetScalars(
          pd, this->ScalarMode, this->ArrayAccessMode, this->ArrayId, this->ArrayName, cellFlag);
        if (lut->IsOpaque(scalars, this->ColorMode, this->ArrayComponent) == 0)
        {
          helperData->IsOpaque = false;
        }
      }
    }
  }
  if (overrides_color)
  {
    this->BlockState.AmbientColor.pop();
    this->BlockState.DiffuseColor.pop();
    this->BlockState.SpecularColor.pop();
  }
  if (overrides_opacity)
  {
    this->BlockState.Opacity.pop();
  }
  if (overrides_pickability)
  {
    this->BlockState.Pickability.pop();
  }
  if (overrides_visibility)
  {
    this->BlockState.Visibility.pop();
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::SetInputArrayToProcess(int idx, vtkInformation* inInfo)
{
  this->Superclass::SetInputArrayToProcess(idx, inInfo);

  // set inputs to helpers
  for (auto& helper : this->Helpers)
  {
    helper.second->SetInputArrayToProcess(idx, inInfo);
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::SetInputArrayToProcess(
  int idx, int port, int connection, int fieldAssociation, int attributeType)
{
  this->Superclass::SetInputArrayToProcess(idx, port, connection, fieldAssociation, attributeType);

  // set inputs to helpers
  for (auto& helper : this->Helpers)
  {
    helper.second->SetInputArrayToProcess(idx, port, connection, fieldAssociation, attributeType);
  }
}

//------------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::SetInputArrayToProcess(
  int idx, int port, int connection, int fieldAssociation, const char* name)
{
  this->Superclass::SetInputArrayToProcess(idx, port, connection, fieldAssociation, name);

  // set inputs to helpers
  for (auto& helper : this->Helpers)
  {
    helper.second->SetInputArrayToProcess(idx, port, connection, fieldAssociation, name);
  }
}

//-----------------------------------------------------------------------------
void vtkCompositePolyDataMapper2::ProcessSelectorPixelBuffers(
  vtkHardwareSelector* sel, std::vector<unsigned int>& pixeloffsets, vtkProp* prop)
{
  // forward to helper
  for (auto& helper : this->Helpers)
  {
    helper.second->ProcessSelectorPixelBuffers(sel, pixeloffsets, prop);
  }
}

//-----------------------------------------------------------------------------
vtkMTimeType vtkCompositePolyDataMapper2::GetMTime()
{
  if (this->CompositeAttributes)
  {
    return std::max(this->Superclass::GetMTime(), this->CompositeAttributes->GetMTime());
  }
  return this->Superclass::GetMTime();
}
