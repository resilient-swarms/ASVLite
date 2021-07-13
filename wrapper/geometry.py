import ctypes

_dll = ctypes.cdll.LoadLibrary("./lib/libASVLite-python.so")

class C_Dimensions(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double),
                ("y", ctypes.c_double),
                ("z", ctypes.c_double)]

class Dimensions:
    def __init__(self, x, y, z):
        global _dll
        self.__c_object = C_Dimensions(ctypes.c_double(x),
                                   ctypes.c_double(y),
                                   ctypes.c_double(z))
    
    # Getter for c_object
    @property
    def c_object(self):
        return self.__c_object
        
    # Getter and setter for x   
    @property
    def x(self):
        return self.c_dimension.x
    
    @x.setter
    def x(self, value):
        self.c_dimension.x = value 

    # Getter and setter for y 
    @property
    def y(self):
        return self.c_dimension.y
    
    @y.setter
    def y(self, value):
        self.c_dimension.y = value 

    # Getter and setter for z 
    @property
    def z(self):
        return self.c_dimension.z
    
    @z.setter
    def z(self, value):
        self.c_dimension.z = value 