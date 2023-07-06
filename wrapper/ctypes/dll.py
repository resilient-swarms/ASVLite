import ctypes
import pathlib
path = pathlib.Path(__file__).parent.resolve()
dll = ctypes.cdll.LoadLibrary(str(path) + "/lib/libASVLite-python.so")