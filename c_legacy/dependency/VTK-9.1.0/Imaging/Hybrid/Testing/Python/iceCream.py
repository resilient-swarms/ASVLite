#!/usr/bin/env python
import vtk
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

def GetRGBColor(colorName):
    '''
        Return the red, green and blue components for a
        color as doubles.
    '''
    rgb = [0.0, 0.0, 0.0]  # black
    vtk.vtkNamedColors().GetColorRGB(colorName, rgb)
    return rgb

ren1 = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren1)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)

# create implicit function primitives
cone = vtk.vtkCone()
cone.SetAngle(20)

vertPlane = vtk.vtkPlane()
vertPlane.SetOrigin(.1, 0, 0)
vertPlane.SetNormal(-1, 0, 0)

basePlane = vtk.vtkPlane()
basePlane.SetOrigin(1.2, 0, 0)
basePlane.SetNormal(1, 0, 0)

iceCream = vtk.vtkSphere()
iceCream.SetCenter(1.333, 0, 0)
iceCream.SetRadius(0.5)

bite = vtk.vtkSphere()
bite.SetCenter(1.5, 0, 0.5)
bite.SetRadius(0.25)

# combine primitives to build ice-cream cone
theCone = vtk.vtkImplicitBoolean()
theCone.SetOperationTypeToIntersection()
theCone.AddFunction(cone)
theCone.AddFunction(vertPlane)
theCone.AddFunction(basePlane)

theCream = vtk.vtkImplicitBoolean()
theCream.SetOperationTypeToDifference()
theCream.AddFunction(iceCream)
theCream.AddFunction(bite)

# iso-surface to create geometry
theConeSample = vtk.vtkSampleFunction()
theConeSample.SetImplicitFunction(theCone)
theConeSample.SetModelBounds(-1, 1.5, -1.25, 1.25, -1.25, 1.25)
theConeSample.SetSampleDimensions(60, 60, 60)
theConeSample.ComputeNormalsOff()

theConeSurface = vtk.vtkMarchingContourFilter()
theConeSurface.SetInputConnection(theConeSample.GetOutputPort())
theConeSurface.SetValue(0, 0.0)

coneMapper = vtk.vtkPolyDataMapper()
coneMapper.SetInputConnection(theConeSurface.GetOutputPort())
coneMapper.ScalarVisibilityOff()

coneActor = vtk.vtkActor()
coneActor.SetMapper(coneMapper)
coneActor.GetProperty().SetColor(GetRGBColor('chocolate'))

# iso-surface to create geometry
theCreamSample = vtk.vtkSampleFunction()
theCreamSample.SetImplicitFunction(theCream)
theCreamSample.SetModelBounds(0, 2.5, -1.25, 1.25, -1.25, 1.25)
theCreamSample.SetSampleDimensions(60, 60, 60)
theCreamSample.ComputeNormalsOff()

theCreamSurface = vtk.vtkMarchingContourFilter()
theCreamSurface.SetInputConnection(theCreamSample.GetOutputPort())
theCreamSurface.SetValue(0, 0.0)

creamMapper = vtk.vtkPolyDataMapper()
creamMapper.SetInputConnection(theCreamSurface.GetOutputPort())
creamMapper.ScalarVisibilityOff()

creamActor = vtk.vtkActor()
creamActor.SetMapper(creamMapper)
creamActor.GetProperty().SetColor(GetRGBColor('mint'))

# Add the actors to the renderer, set the background and size
#
ren1.AddActor(coneActor)
ren1.AddActor(creamActor)
ren1.SetBackground(1, 1, 1)

renWin.SetSize(250, 250)

ren1.ResetCamera()
ren1.GetActiveCamera().Roll(90)
ren1.GetActiveCamera().Dolly(1.5)
ren1.ResetCameraClippingRange()

iren.Initialize()

# render the image
#
renWin.Render()

#iren.Start()
