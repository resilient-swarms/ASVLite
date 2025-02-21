#!/usr/bin/env python
import vtk
from math import cos, sin, pi
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

# Plane cut an oriented volume

# Create the RenderWindow, Renderer and both Actors
#
ren1 = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren1)
renWin.SetSize(300,300)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)

math = vtk.vtkMath()

# Generate some random colors
def MakeColors (lut, n):
    lut.SetNumberOfColors(n)
    lut.SetTableRange(0, n - 1)
    lut.SetScaleToLinear()
    lut.Build()
    lut.SetTableValue(0, 0, 0, 0, 1)
    math.RandomSeed(5071)
    i = 1
    while i < n:
        lut.SetTableValue(i, math.Random(.2, 1),
          math.Random(.2, 1), math.Random(.2, 1), 1)
        i += 1

lut = vtk.vtkLookupTable()
MakeColors(lut, 256)
n = 20
radius = 10

# This has been moved outside the loop so that the code can be correctly
# translated to python
blobImage = vtk.vtkImageData()

i = 0
while i < n:
    sphere = vtk.vtkSphere()
    sphere.SetRadius(radius)
    max = 50 - radius
    sphere.SetCenter(int(math.Random(-max, max)),
      int(math.Random(-max, max)), int(math.Random(-max, max)))

    sampler = vtk.vtkSampleFunction()
    sampler.SetImplicitFunction(sphere)
    sampler.SetOutputScalarTypeToFloat()
    sampler.SetSampleDimensions(51, 51, 51)
    sampler.SetModelBounds(-50, 50, -50, 50, -50, 50)

    thres = vtk.vtkImageThreshold()
    thres.SetInputConnection(sampler.GetOutputPort())
    thres.ThresholdByLower(radius * radius)
    thres.ReplaceInOn()
    thres.ReplaceOutOn()
    thres.SetInValue(i + 1)
    thres.SetOutValue(0)
    thres.Update()
    if (i == 0):
        blobImage.DeepCopy(thres.GetOutput())

    maxValue = vtk.vtkImageMathematics()
    maxValue.SetInputData(0, blobImage)
    maxValue.SetInputData(1, thres.GetOutput())
    maxValue.SetOperationToMax()
    maxValue.Modified()
    maxValue.Update()

    blobImage.DeepCopy(maxValue.GetOutput())

    i += 1

angle = pi/6
orientation = [
  -cos(angle), 0, sin(angle),
  0, 1, 0,
  sin(angle), 0, cos(angle),
]
blobImage.SetDirectionMatrix(orientation)

# Create an outline around the image
outline = vtk.vtkImageDataOutlineFilter()
outline.SetInputData(blobImage)
outline.GenerateFacesOn()

outlineMapper = vtk.vtkPolyDataMapper()
outlineMapper.SetInputConnection(outline.GetOutputPort())

outlineActor = vtk.vtkActor()
outlineActor.SetMapper(outlineMapper)
outlineActor.GetProperty().SetOpacity(0.25)

# Now cut the volume
plane = vtk.vtkPlane()
plane.SetOrigin(1,1,1)
plane.SetNormal(1,1,1)

cutter = vtk.vtkFlyingEdgesPlaneCutter()
cutter.SetInputData(blobImage)
cutter.SetPlane(plane)

cutterMapper = vtk.vtkPolyDataMapper()
cutterMapper.SetInputConnection(cutter.GetOutputPort())

cutterActor = vtk.vtkActor()
cutterActor.SetMapper(cutterMapper)

ren1.AddActor(outlineActor)
ren1.AddActor(cutterActor)

renWin.Render()
iren.Start()
