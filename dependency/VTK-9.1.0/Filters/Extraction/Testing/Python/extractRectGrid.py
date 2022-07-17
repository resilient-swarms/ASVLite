#!/usr/bin/env python
# import os
import vtk
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

# create pipeline - rectilinear grid
#
rgridReader = vtk.vtkRectilinearGridReader()
rgridReader.SetFileName(VTK_DATA_ROOT + "/Data/RectGrid2.vtk")

outline = vtk.vtkOutlineFilter()
outline.SetInputConnection(rgridReader.GetOutputPort())

mapper = vtk.vtkPolyDataMapper()
mapper.SetInputConnection(outline.GetOutputPort())

actor = vtk.vtkActor()
actor.SetMapper(mapper)

rgridReader.Update()

extract1 = vtk.vtkExtractRectilinearGrid()
extract1.SetInputConnection(rgridReader.GetOutputPort())
# extract1.SetVOI(0, 46, 0, 32, 0, 10)
extract1.SetVOI(23, 40, 16, 30, 9, 9)
extract1.SetSampleRate(2, 2, 1)
extract1.IncludeBoundaryOn()
extract1.Update()

surf1 = vtk.vtkDataSetSurfaceFilter()
surf1.SetInputConnection(extract1.GetOutputPort())

tris = vtk.vtkTriangleFilter()
tris.SetInputConnection(surf1.GetOutputPort())

mapper1 = vtk.vtkPolyDataMapper()
mapper1.SetInputConnection(tris.GetOutputPort())
mapper1.SetScalarRange(extract1.GetOutput().GetScalarRange())

actor1 = vtk.vtkActor()
actor1.SetMapper(mapper1)

# Create the RenderWindow, Renderer and both Actors
#
ren1 = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren1)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)

# ren1.AddActor(actor)
ren1.AddActor(actor1)
renWin.SetSize(340, 400)

iren.Initialize()

# render the image
#iren.Start()
