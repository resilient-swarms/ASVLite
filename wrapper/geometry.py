import ctypes
import dll

class _Dimensions(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double),
                ("y", ctypes.c_double),
                ("z", ctypes.c_double)]

class Dimensions:
    def __init__(self, x, y, z):
        self.__c_object = _Dimensions(ctypes.c_double(x),
                                      ctypes.c_double(y),
                                      ctypes.c_double(z))
    
    # Getter for c_object
    @property
    def c_object(self):
        return self.__c_object
        
    # Getter and setter for x   
    @property
    def x(self):
        return self.__c_object.x
    
    @x.setter
    def x(self, value):
        self.__c_object.x = value 

    # Getter and setter for y 
    @property
    def y(self):
        return self.__c_object.y
    
    @y.setter
    def y(self, value):
        self.__c_object.y = value 

    # Getter and setter for z 
    @property
    def z(self):
        return self.__c_object.z
    
    @z.setter
    def z(self, value):
        self.__c_object.z = value 