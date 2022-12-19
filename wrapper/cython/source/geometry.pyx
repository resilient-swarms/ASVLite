cdef class py_Coordinates_3D:
    '''
    Cartesian coordinates for three-dimensional space. The class members can be accessed 
    using the keys x, y and z or as an array using the index 0..2. The class also provides 
    an iterator to access the members in the order of the keys mentioned.
    '''

    def __cinit__(self, double x=0, double y=0, double z=0):
        self._c_object.keys.x = x
        self._c_object.keys.y = y
        self._c_object.keys.z = z
        self.__i= 0
    
    def __iter__(self):
        self.__i= 0
        return self 
    
    def __next__(self):
        if self.__i< COUNT_COORDINATES:
            value = self._c_object.array[self.__i]
            self.__i+= 1
            return value
        else:
            raise StopIteration
    
    def __setitem__(self, int key, double value):
        if key < COUNT_COORDINATES:
            self._c_object.array[key] = value 
        else:
            raise IndexError
    
    def __getitem__(self, int key):
        if key < COUNT_COORDINATES:
            return self._c_object.array[key]
        else:
            raise IndexError

    @property
    def x(self):
        return self._c_object.keys.x
    
    @x.setter
    def x(self, double value):
        self._c_object.keys.x = value
    
    @property
    def y(self):
        return self._c_object.keys.y
    
    @y.setter
    def y(self, double value):
        self._c_object.keys.y = value
    
    @property
    def z(self):
        return self._c_object.keys.z
    
    @z.setter
    def z(self, double value):
        self._c_object.keys.z = value


cdef class py_Rigid_body_DOF:  
    '''
    Six degrees of freedom for a rigid body in a three-dimensional space. 
    The class members can be accessed using the keys surge, sway, heave, roll, 
    pitch and yaw or as an array using the index 0..5. The class also provides 
    an iterator to access the members in the order of the keys mentioned.
    '''

    def __cinit__(self, double surge=0, double sway=0, double heave=0, double roll=0, double pitch=0, double yaw=0):
        self._c_object.keys.surge = surge 
        self._c_object.keys.sway  = sway 
        self._c_object.keys.heave = heave
        self._c_object.keys.roll  = roll 
        self._c_object.keys.pitch = pitch 
        self._c_object.keys.yaw   = yaw
        self.__i= 0
    
    def __iter__(self):
        self.__i= 0
        return self 
    
    def __next__(self):
        if self.__i< COUNT_DOF:
            value = self._c_object.array[self.__i]
            self.__i+= 1
            return value
        else:
            raise StopIteration
    
    def __setitem__(self, int key, double value):
        if key < COUNT_DOF:
            self._c_object.array[key] = value 
        else:
            raise IndexError
    
    def __getitem__(self, int key):
        if key < COUNT_DOF:
            return self._c_object.array[key]
        else:
            raise IndexError
    
    @property
    def surge(self):
        return self._c_object.keys.surge
    
    @surge.setter
    def surge(self, double value):
        self._c_object.keys.surge = value
    
    @property
    def sway(self):
        return self._c_object.keys.sway
    
    @sway.setter
    def sway(self, double value):
        self._c_object.keys.sway = value
    
    @property
    def heave(self):
        return self._c_object.keys.heave
    
    @heave.setter
    def heave(self, double value):
        self._c_object.keys.heave = value
    
    @property
    def roll(self):
        return self._c_object.keys.roll
    
    @roll.setter
    def roll(self, double value):
        self._c_object.keys.roll = value
    
    @property
    def pitch(self):
        return self._c_object.keys.pitch
    
    @pitch.setter
    def pitch(self, double value):
        self._c_object.keys.pitch = value
    
    @property
    def yaw(self):
        return self._c_object.keys.yaw
    
    @yaw.setter
    def yaw(self, double value):
        self._c_object.keys.yaw = value
    
cpdef double py_normalise_angle_PI(double angle):
    '''
    Normalise angle so that -PI < angle <= PI.
    '''
    return normalise_angle_PI(angle)

cpdef double py_normalise_angle_2PI(double angle):
    '''
    Normalise angle so that 0 <= angle < 2PI.
    '''
    return normalise_angle_2PI(angle)