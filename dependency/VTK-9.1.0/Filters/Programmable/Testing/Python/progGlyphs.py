#!/usr/bin/env python
import vtk
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

res = 6
plane = vtk.vtkPlaneSource()
plane.SetResolution(res,res)
colors = vtk.vtkElevationFilter()
colors.SetInputConnection(plane.GetOutputPort())
colors.SetLowPoint(-0.25,-0.25,-0.25)
colors.SetHighPoint(0.25,0.25,0.25)
planeMapper = vtk.vtkPolyDataMapper()
planeMapper.SetInputConnection(colors.GetOutputPort())
planeActor = vtk.vtkActor()
planeActor.SetMapper(planeMapper)
planeActor.GetProperty().SetRepresentationToWireframe()
# procedure for generating glyphs
def Glyph (__vtk__temp0=0,__vtk__temp1=0):
    global res
    ptId = glypher.GetPointId()
    pd = glypher.GetPointData()
    xyz = glypher.GetPoint()
    x = lindex(xyz,0)
    y = lindex(xyz,1)
    length = glypher.GetInput(0).GetLength()
    scale = expr.expr(globals(), locals(),["length","/","(","2.0","*","res",")"])
    squad.SetScale(scale,scale,scale)
    squad.SetCenter(xyz)
    squad.SetPhiRoundness(expr.expr(globals(), locals(),["abs","(","x",")*","5.0"]))
    squad.SetThetaRoundness(expr.expr(globals(), locals(),["abs","(","y",")*","5.0"]))
    squad.Update()

# create simple poly data so we can apply glyph
squad = vtk.vtkSuperquadricSource()
squad.Update()
glypher = vtk.vtkProgrammableGlyphFilter()
glypher.SetInputConnection(colors.GetOutputPort())
glypher.SetSourceData(squad.GetOutput())
glypher.SetGlyphMethod(Glyph)
glyphMapper = vtk.vtkPolyDataMapper()
glyphMapper.SetInputConnection(glypher.GetOutputPort())
glyphActor = vtk.vtkActor()
glyphActor.SetMapper(glyphMapper)
# Create the rendering stuff
#
ren1 = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.SetMultiSamples(0)
renWin.AddRenderer(ren1)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)
ren1.AddActor(planeActor)
ren1.AddActor(glyphActor)
ren1.SetBackground(1,1,1)
renWin.SetSize(450,450)
renWin.Render()
ren1.GetActiveCamera().Zoom(1.5)
# Get handles to some useful objects
#
renWin.Render()
# prevent the tk window from showing up then start the event loop
# --- end of script --
