#!/usr/bin/env python
import os
import vtk
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

# Read a field representing unstructured grid and display it (similar to blow.tcl)

# create a reader and write out field data
reader = vtk.vtkUnstructuredGridReader()
reader.SetFileName(VTK_DATA_ROOT + "/Data/blow.vtk")
reader.SetScalarsName("thickness9")
reader.SetVectorsName("displacement9")

ds2do = vtk.vtkDataSetToDataObjectFilter()
ds2do.SetInputConnection(reader.GetOutputPort())

# we must be able to write here
try:
    channel = open("UGridField.vtk", "wb")
    channel.close()

    write = vtk.vtkDataObjectWriter()
    write.SetInputConnection(ds2do.GetOutputPort())
    write.SetFileName("UGridField.vtk")
    write.Write()

    # Read the field and convert to unstructured grid.
    dor = vtk.vtkDataObjectReader()
    dor.SetFileName("UGridField.vtk")

    do2ds = vtk.vtkDataObjectToDataSetFilter()
    do2ds.SetInputConnection(dor.GetOutputPort())
    do2ds.SetDataSetTypeToUnstructuredGrid()
    do2ds.SetPointComponent(0, "Points", 0)
    do2ds.SetPointComponent(1, "Points", 1)
    do2ds.SetPointComponent(2, "Points", 2)
    do2ds.SetCellTypeComponent("CellTypes", 0)
    do2ds.SetCellConnectivityComponent("Cells", 0)
    do2ds.Update()

    fd2ad = vtk.vtkFieldDataToAttributeDataFilter()
    fd2ad.SetInputData(do2ds.GetUnstructuredGridOutput())
    fd2ad.SetInputFieldToDataObjectField()
    fd2ad.SetOutputAttributeDataToPointData()
    fd2ad.SetVectorComponent(0, "displacement9", 0)
    fd2ad.SetVectorComponent(1, "displacement9", 1)
    fd2ad.SetVectorComponent(2, "displacement9", 2)
    fd2ad.SetScalarComponent(0, "thickness9", 0)
    fd2ad.Update()

    # Now start visualizing
    warp = vtk.vtkWarpVector()
    warp.SetInputData(fd2ad.GetUnstructuredGridOutput())

    # extract mold from mesh using connectivity
    connect = vtk.vtkConnectivityFilter()
    connect.SetInputConnection(warp.GetOutputPort())
    connect.SetExtractionModeToSpecifiedRegions()
    connect.AddSpecifiedRegion(0)
    connect.AddSpecifiedRegion(1)

    moldMapper = vtk.vtkDataSetMapper()
    moldMapper.SetInputConnection(connect.GetOutputPort())
    moldMapper.ScalarVisibilityOff()

    moldActor = vtk.vtkActor()
    moldActor.SetMapper(moldMapper)
    moldActor.GetProperty().SetColor(.2, .2, .2)
    moldActor.GetProperty().SetRepresentationToWireframe()

    # extract parison from mesh using connectivity
    connect2 = vtk.vtkConnectivityFilter()
    connect2.SetInputConnection(warp.GetOutputPort())
    connect2.SetExtractionModeToSpecifiedRegions()
    connect2.AddSpecifiedRegion(2)

    parison = vtk.vtkGeometryFilter()
    parison.SetInputConnection(connect2.GetOutputPort())

    normals2 = vtk.vtkPolyDataNormals()
    normals2.SetInputConnection(parison.GetOutputPort())
    normals2.SetFeatureAngle(60)

    lut = vtk.vtkLookupTable()
    lut.SetHueRange(0.0, 0.66667)

    parisonMapper = vtk.vtkPolyDataMapper()
    parisonMapper.SetInputConnection(normals2.GetOutputPort())
    parisonMapper.SetLookupTable(lut)
    parisonMapper.SetScalarRange(0.12, 1.0)

    parisonActor = vtk.vtkActor()
    parisonActor.SetMapper(parisonMapper)

    cf = vtk.vtkContourFilter()
    cf.SetInputConnection(connect2.GetOutputPort())
    cf.SetValue(0, .5)

    contourMapper = vtk.vtkPolyDataMapper()
    contourMapper.SetInputConnection(cf.GetOutputPort())

    contours = vtk.vtkActor()
    contours.SetMapper(contourMapper)

    # Create graphics stuff
    ren1 = vtk.vtkRenderer()
    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(ren1)
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)

    # Add the actors to the renderer, set the background and size
    ren1.AddActor(moldActor)
    ren1.AddActor(parisonActor)
    ren1.AddActor(contours)

    ren1.ResetCamera()
    ren1.GetActiveCamera().Azimuth(60)
    ren1.GetActiveCamera().Roll(-90)
    ren1.GetActiveCamera().Dolly(2)
    ren1.ResetCameraClippingRange()
    ren1.SetBackground(1, 1, 1)

    renWin.SetSize(380, 200)
    renWin.SetMultiSamples(0)
    iren.Initialize()

    # render the image
    #
    renWin.Render()

    # cleanup
    #
    try:
        os.remove("UGridField.vtk")
    except OSError:
        pass

#    iren.Start()

except IOError:
    print("Couldn't open UGridField.vtk for writing.")
