#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
=========================================================================

  Program:   Visualization Toolkit
  Module:    TestNamedColorsIntegration.py

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================
'''

import sys
import vtk
import vtk.test.Testing
from vtk.util.misc import vtkGetDataRoot
VTK_DATA_ROOT = vtkGetDataRoot()

'''
  Prevent .pyc files from being created.
  Stops the vtk source being polluted
  by .pyc files.
'''
sys.dont_write_bytecode = True

# Load base (spike and test)
import TestStyleBaseSpike
import TestStyleBase

class TestStyleJoystickCamera(vtk.test.Testing.vtkTest):

    def testStyleJoystickCamera(self):

        ren = vtk.vtkRenderer()
        renWin = vtk.vtkRenderWindow()
        iRen = vtk.vtkRenderWindowInteractor()
        iRen.SetRenderWindow(renWin);

        testStyleBaseSpike = TestStyleBaseSpike.StyleBaseSpike(ren, renWin, iRen)

        # Set interactor style
        inStyle = vtk.vtkInteractorStyleSwitch()
        iRen.SetInteractorStyle(inStyle)

        # Switch to Joystick+Camera mode

        iRen.SetKeyEventInformation(0, 0, 'j', 0, '0')
        iRen.InvokeEvent("CharEvent")

        iRen.SetKeyEventInformation(0, 0, 'c', 0, '0')
        iRen.InvokeEvent("CharEvent")

        # Test style

        testStyleBase = TestStyleBase.TestStyleBase(ren)
        testStyleBase.test_style(inStyle.GetCurrentStyle())

        # render and interact with data

        img_file = "TestStyleJoystickCamera.png"
        vtk.test.Testing.compareImage(iRen.GetRenderWindow(), vtk.test.Testing.getAbsImagePath(img_file), threshold=25)
        vtk.test.Testing.interact()

if __name__ == "__main__":
     vtk.test.Testing.main([(TestStyleJoystickCamera, 'test')])
