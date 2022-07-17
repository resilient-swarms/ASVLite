#!/usr/bin/env python
import math
import vtk
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

# Image pipeline
reader = vtk.vtkPNGReader()
reader.SetFileName(VTK_DATA_ROOT + "/Data/fullhead15.png")

# Take the gradient in X, and smooth in Y
# Create a simple gradient filter
kernel = vtk.vtkFloatArray()
kernel.SetNumberOfTuples(3)
kernel.InsertValue(0, -1)
kernel.InsertValue(1, 0)
kernel.InsertValue(2, 1)

# Create a gaussian for Y
sigma = 1.5
sigma2 = sigma * sigma
gaussian = vtk.vtkFloatArray()
gaussian.SetNumberOfTuples(31)
i = 0
while i < 31:
    x = i - 15
    g = math.exp(-(x * x) / (2.0 * sigma2)) / (math.sqrt (2.0 * 3.1415) * sigma)
    gaussian.InsertValue(i, g)
    i += 1

convolve = vtk.vtkImageSeparableConvolution()
convolve.SetInputConnection(reader.GetOutputPort())
convolve.SetDimensionality(2)
convolve.SetXKernel(kernel)
convolve.SetYKernel(gaussian)

viewer = vtk.vtkImageViewer()
# viewer DebugOn
viewer.SetInputConnection(convolve.GetOutputPort())
viewer.SetColorWindow(500)
viewer.SetColorLevel(100)
viewer.Render()
