#!/usr/bin/env python
import vtk
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

# we need to use composite data pipeline with multiblock datasets
alg = vtk.vtkAlgorithm()
pip = vtk.vtkCompositeDataPipeline()
alg.SetDefaultExecutivePrototype(pip)
#del pip

Ren1 = vtk.vtkRenderer()
Ren1.SetBackground(0.33, 0.35, 0.43)
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(Ren1)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)

Plot3D0 = vtk.vtkMultiBlockPLOT3DReader()
Plot3D0.SetFileName(VTK_DATA_ROOT + "/Data/combxyz.bin")
Plot3D0.SetQFileName(VTK_DATA_ROOT + "/Data/combq.bin")
Plot3D0.SetBinaryFile(1)
Plot3D0.SetMultiGrid(0)
Plot3D0.SetHasByteCount(0)
Plot3D0.SetIBlanking(0)
Plot3D0.SetTwoDimensionalGeometry(0)
Plot3D0.SetForceRead(0)
Plot3D0.SetByteOrder(0)
Plot3D0.Update()

output = Plot3D0.GetOutput().GetBlock(0)

Geometry5 = vtk.vtkStructuredGridOutlineFilter()
Geometry5.SetInputData(output)

Mapper5 = vtk.vtkPolyDataMapper()
Mapper5.SetInputConnection(Geometry5.GetOutputPort())
Mapper5.UseLookupTableScalarRangeOn()
Mapper5.SetScalarVisibility(0)
Mapper5.SetScalarModeToDefault()

Actor5 = vtk.vtkActor()
Actor5.SetMapper(Mapper5)
Actor5.GetProperty().SetRepresentationToSurface()
Actor5.GetProperty().SetInterpolationToGouraud()
Actor5.GetProperty().SetAmbient(0.15)
Actor5.GetProperty().SetDiffuse(0.85)
Actor5.GetProperty().SetSpecular(0.1)
Actor5.GetProperty().SetSpecularPower(100)
Actor5.GetProperty().SetSpecularColor(1, 1, 1)
Actor5.GetProperty().SetColor(1, 1, 1)

Ren1.AddActor(Actor5)

ExtractGrid0 = vtk.vtkExtractGrid()
ExtractGrid0.SetInputData(output)
ExtractGrid0.SetVOI(0, 14, 0, 32, 0, 24)
ExtractGrid0.SetSampleRate(1, 1, 1)
ExtractGrid0.SetIncludeBoundary(0)

ExtractGrid1 = vtk.vtkExtractGrid()
ExtractGrid1.SetInputData(output)
ExtractGrid1.SetVOI(14, 29, 0, 32, 0, 24)
ExtractGrid1.SetSampleRate(1, 1, 1)
ExtractGrid1.SetIncludeBoundary(0)

ExtractGrid2 = vtk.vtkExtractGrid()
ExtractGrid2.SetInputData(output)
ExtractGrid2.SetVOI(29, 56, 0, 32, 0, 24)
ExtractGrid2.SetSampleRate(1, 1, 1)
ExtractGrid2.SetIncludeBoundary(0)

LineSourceWidget0 = vtk.vtkLineSource()
LineSourceWidget0.SetPoint1(3.05638, -3.00497, 28.2211)
LineSourceWidget0.SetPoint2(3.05638, 3.95916, 28.2211)
LineSourceWidget0.SetResolution(20)

mbds = vtk.vtkMultiBlockDataSet()
mbds.SetNumberOfBlocks(3)

i = 0
while i < 3:
    eval("ExtractGrid" + str(i)).Update()
    exec("sg" + str(i) + " = vtk.vtkStructuredGrid()")
    eval("sg" + str(i)).ShallowCopy(eval("ExtractGrid" + str(i)).GetOutput())
    mbds.SetBlock(i, eval("sg" + str(i)))
    i += 1

Stream0 = vtk.vtkStreamTracer()
Stream0.SetInputData(mbds)
Stream0.SetSourceConnection(LineSourceWidget0.GetOutputPort())
Stream0.SetIntegrationStepUnit(2)
Stream0.SetMaximumPropagation(20)
Stream0.SetInitialIntegrationStep(0.5)
Stream0.SetIntegrationDirection(0)
Stream0.SetIntegratorType(0)
Stream0.SetMaximumNumberOfSteps(2000)
Stream0.SetTerminalSpeed(1e-12)

#del mbds

aa = vtk.vtkAssignAttribute()
aa.SetInputConnection(Stream0.GetOutputPort())
aa.Assign("Normals", "NORMALS", "POINT_DATA")

Ribbon0 = vtk.vtkRibbonFilter()
Ribbon0.SetInputConnection(aa.GetOutputPort())
Ribbon0.SetWidth(0.1)
Ribbon0.SetAngle(0)
Ribbon0.SetDefaultNormal(0, 0, 1)
Ribbon0.SetVaryWidth(0)

LookupTable1 = vtk.vtkLookupTable()
LookupTable1.SetNumberOfTableValues(256)
LookupTable1.SetHueRange(0, 0.66667)
LookupTable1.SetSaturationRange(1, 1)
LookupTable1.SetValueRange(1, 1)
LookupTable1.SetTableRange(0.197813, 0.710419)
LookupTable1.SetVectorComponent(0)
LookupTable1.Build()

Mapper10 = vtk.vtkPolyDataMapper()
Mapper10.SetInputConnection(Ribbon0.GetOutputPort())
Mapper10.UseLookupTableScalarRangeOn()
Mapper10.SetScalarVisibility(1)
Mapper10.SetScalarModeToUsePointFieldData()
Mapper10.SelectColorArray("Density")
Mapper10.SetLookupTable(LookupTable1)

Actor10 = vtk.vtkActor()
Actor10.SetMapper(Mapper10)
Actor10.GetProperty().SetRepresentationToSurface()
Actor10.GetProperty().SetInterpolationToGouraud()
Actor10.GetProperty().SetAmbient(0.15)
Actor10.GetProperty().SetDiffuse(0.85)
Actor10.GetProperty().SetSpecular(0)
Actor10.GetProperty().SetSpecularPower(1)
Actor10.GetProperty().SetSpecularColor(1, 1, 1)

Ren1.AddActor(Actor10)

iren.Initialize()

alg.SetDefaultExecutivePrototype(None)
#del alg

#iren.Start()
