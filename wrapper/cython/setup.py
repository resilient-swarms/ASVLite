from distutils.core import setup, Extension
from Cython.Build import cythonize
import pathlib

module_dir  = pathlib.Path(__file__).parent.resolve()
root_dir    = module_dir.parent
include_dir = root_dir.joinpath("include")
source_dir  = root_dir.joinpath("source") 

extensions = []
extensions.append(Extension("geometry", sources=[str(source_dir)+"/geometry.c", 
                                                "source/geometry.pyx"]))
extensions.append(Extension("regular_wave", sources=[str(source_dir)+"/errors.c", 
                                                     str(source_dir)+"/geometry.c",
                                                     str(source_dir)+"/regular_wave.c", 
                                                     "source/regular_wave.pyx"]))
extensions.append(Extension("sea_surface", sources=[str(source_dir)+"/errors.c", 
                                             str(source_dir)+"/geometry.c",
                                             str(source_dir)+"/regular_wave.c", 
                                             str(source_dir)+"/sea_surface.c",
                                             "source/sea_surface.pyx"]))
extensions.append(Extension("asv", sources=[str(source_dir)+"/errors.c", 
                                            str(source_dir)+"/geometry.c",
                                            str(source_dir)+"/regular_wave.c", 
                                            str(source_dir)+"/sea_surface.c",
                                            str(source_dir)+"/asv.c",
                                            "source/asv.pyx"]))
setup(ext_modules=cythonize(extensions, language_level = "3"), include_dirs=[str(include_dir)])