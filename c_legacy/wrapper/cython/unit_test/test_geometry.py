from geometry import py_Coordinates_3D, py_Rigid_body_DOF, py_normalise_angle_PI, py_normalise_angle_2PI 
import math
import unittest

class Test_py_Coordinates_3D(unittest.TestCase):

    def test_keys_getters(self):
        point = py_Coordinates_3D(1,2,3)
        self.assertEqual(point.x, 1)
        self.assertEqual(point.y, 2)
        self.assertEqual(point.z, 3)
    
    def test_keys_setters(self):
        point = py_Coordinates_3D(0,0,0)
        point.x = 1
        point.y = 2
        point.z = 3
        self.assertEqual(point.x, 1)
        self.assertEqual(point.y, 2)
        self.assertEqual(point.z, 3)
    
    def test_array_getters(self):
        point = py_Coordinates_3D(1,2,3)
        self.assertEqual(point[0], 1)
        self.assertEqual(point[1], 2)
        self.assertEqual(point[2], 3)

    def test_array_setters(self):
        point = py_Coordinates_3D(0,0,0)
        point[0] = 1
        point[1] = 2
        point[2] = 3
        self.assertEqual(point.x, 1)
        self.assertEqual(point.y, 2)
        self.assertEqual(point.z, 3)

    def test_iterator(self):
        point = py_Coordinates_3D(1,2,3)
        i = 1
        for item in point:
            self.assertEqual(item, i)
            i += 1


class Test_py_Rigid_body_DOF(unittest.TestCase):

    def test_keys_getters(self):
        point = py_Rigid_body_DOF(1,2,3,4,5,6)
        self.assertEqual(point.surge, 1)
        self.assertEqual(point.sway, 2)
        self.assertEqual(point.heave, 3)
        self.assertEqual(point.roll, 4)
        self.assertEqual(point.pitch, 5)
        self.assertEqual(point.yaw, 6)
    
    def test_keys_setters(self):
        point = py_Rigid_body_DOF(0,0,0,0,0,0)
        point.surge = 1
        point.sway = 2
        point.heave = 3
        point.roll = 4
        point.pitch = 5
        point.yaw = 6
        self.assertEqual(point.surge, 1)
        self.assertEqual(point.sway, 2)
        self.assertEqual(point.heave, 3)
        self.assertEqual(point.roll, 4)
        self.assertEqual(point.pitch, 5)
        self.assertEqual(point.yaw, 6)
    
    def test_array_getters(self):
        point = py_Rigid_body_DOF(1,2,3,4,5,6)
        self.assertEqual(point[0], 1)
        self.assertEqual(point[1], 2)
        self.assertEqual(point[2], 3)
        self.assertEqual(point[3], 4)
        self.assertEqual(point[4], 5)
        self.assertEqual(point[5], 6)

    def test_array_setters(self):
        point = py_Rigid_body_DOF(0,0,0,0,0,0)
        point[0] = 1
        point[1] = 2
        point[2] = 3
        point[3] = 4
        point[4] = 5
        point[5] = 6
        self.assertEqual(point.surge, 1)
        self.assertEqual(point.sway, 2)
        self.assertEqual(point.heave, 3)
        self.assertEqual(point.roll, 4)
        self.assertEqual(point.pitch, 5)
        self.assertEqual(point.yaw, 6)

    def test_iterator(self):
        point = py_Rigid_body_DOF(1,2,3,4,5,6)
        i = 1
        for item in point:
            self.assertEqual(item, i)
            i += 1

class Test_normalised_anlges(unittest.TestCase):

    def test_py_normalise_angle_PI(self):
        angle = 1.5 * math.pi
        normalised_angle = py_normalise_angle_PI(angle)
        self.assertEqual(normalised_angle, -math.pi/2.0)

        angle = -1.5 * math.pi
        normalised_angle = py_normalise_angle_PI(angle)
        self.assertEqual(normalised_angle, math.pi/2.0)

    def test_py_normalise_angle_2PI(self):
        angle = 2.5 * math.pi
        normalised_angle = py_normalise_angle_2PI(angle)
        self.assertEqual(normalised_angle, math.pi/2.0)

        angle = -math.pi
        normalised_angle = py_normalise_angle_2PI(angle)
        self.assertEqual(normalised_angle, math.pi)
