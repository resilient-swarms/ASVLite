#!/usr/bin/env python

# we need to use composite data pipeline with multiblock datasets
alg = vtk.vtkAlgorithm()
pip = vtk.vtkCompositeDataPipeline()
alg.SetDefaultExecutivePrototype(pip)
del pip
# Create the RenderWindow, Renderer and both Actors
#
Ren1 = vtk.vtkRenderer()
Ren1.SetBackground(0.33,0.35,0.43)
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(Ren1)
renWin.SetSize(300,300)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)

pvTemp59 = vtk.vtkXMLRectilinearGridReader()
pvTemp59.SetFileName("" + str(VTK_DATA_ROOT) + "/Data/cth.vtr")
pvTemp59.UpdateInformation()
pvTemp59.SetCellArrayStatus("X Velocity",0)
pvTemp59.SetCellArrayStatus("Y Velocity",0)
pvTemp59.SetCellArrayStatus("Z Velocity",0)
pvTemp59.SetCellArrayStatus("Mass for Armor Plate",0)
pvTemp59.SetCellArrayStatus("Mass for Body, Nose",0)

pvTemp79 = vtk.vtkExtractCTHPart()
pvTemp79.SetInputConnection(pvTemp59.GetOutputPort())
pvTemp79.AddVolumeArrayName("Volume Fraction for Armor Plate")
pvTemp79.AddVolumeArrayName("Volume Fraction for Body, Nose")
pvTemp79.SetClipPlane(None)
pvTemp79.GenerateSolidGeometryOn();

pvTemp104 = vtk.vtkLookupTable()
pvTemp104.SetNumberOfTableValues(256)
pvTemp104.SetHueRange(0.6667,0)
pvTemp104.SetSaturationRange(1,1)
pvTemp104.SetValueRange(1,1)
pvTemp104.SetTableRange(0,1)
pvTemp104.SetVectorComponent(0)
pvTemp104.Build()

pvTemp79.Update()
compositeData = pvTemp79.GetOutput();
dataList = compositeData.NewIterator()
dataList.InitTraversal()

while not dataList.IsDoneWithTraversal():
  # get next object in the composite dataset
  currData = dataList.GetCurrentDataObject()

  # construct mapper
  pvTemp87 = vtk.vtkDataSetMapper()
  pvTemp87.SetInputData(currData)
  pvTemp87.SetScalarRange(0,1)
  pvTemp87.UseLookupTableScalarRangeOn()
  pvTemp87.SetScalarVisibility(1)
  pvTemp87.SetScalarModeToUsePointFieldData()
  pvTemp87.SelectColorArray("Part Index")
  pvTemp87.SetLookupTable(pvTemp104)

  # construct actor
  pvTemp88 = vtk.vtkActor()
  pvTemp88.SetMapper(pvTemp87)
  pvTemp88.GetProperty().SetRepresentationToSurface()
  pvTemp88.GetProperty().SetInterpolationToGouraud()
  pvTemp88.GetProperty().SetAmbient(0)
  pvTemp88.GetProperty().SetDiffuse(1)
  pvTemp88.GetProperty().SetSpecular(0)
  pvTemp88.GetProperty().SetSpecularPower(1)
  pvTemp88.GetProperty().SetSpecularColor(1,1,1)
  Ren1.AddActor(pvTemp88)

  # update iterator
  dataList.GoToNextItem()

renWin.Render()

alg.SetDefaultExecutivePrototype(None)
# --- end of script --
