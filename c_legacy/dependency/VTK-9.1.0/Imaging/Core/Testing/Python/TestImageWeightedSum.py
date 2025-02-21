#!/usr/bin/env python
import vtk
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

# Image pipeline
reader = vtk.vtkImageReader()
reader.ReleaseDataFlagOff()
reader.SetDataByteOrderToLittleEndian()
reader.SetDataExtent(0, 63, 0, 63, 1, 93)
reader.SetFilePrefix(VTK_DATA_ROOT + "/Data/headsq/quarter")
reader.SetDataMask(0x7fff)

mag = vtk.vtkImageMagnify()
mag.SetInputConnection(reader.GetOutputPort())
mag.SetMagnificationFactors(4, 4, 1)

th = vtk.vtkImageThreshold()
th.SetInputConnection(mag.GetOutputPort())
th.SetReplaceIn(1)
th.SetReplaceOut(1)
th.ThresholdBetween(-1000, 1000)
th.SetOutValue(0)
th.SetInValue(2000)

cast = vtk.vtkImageCast()
cast.SetInputConnection(mag.GetOutputPort())
cast.SetOutputScalarTypeToFloat()

cast2 = vtk.vtkImageCast()
cast2.SetInputConnection(th.GetOutputPort())
cast2.SetOutputScalarTypeToFloat()

sum = vtk.vtkImageWeightedSum()
sum.AddInputConnection(cast.GetOutputPort())
sum.AddInputConnection(cast2.GetOutputPort())
sum.SetWeight(0, 10)
sum.SetWeight(1, 4)

viewer = vtk.vtkImageViewer()
viewer.SetInputConnection(sum.GetOutputPort())
viewer.SetZSlice(22)
viewer.SetColorWindow(1819)
viewer.SetColorLevel(939)
sum.SetWeight(0, 1)
# make interface
viewer.SetSize(256, 256)
viewer.Render()
