from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
import numpy

ext_modules=[
    Extension("cy_yolo2_findboxes",
              sources=["cy_yolo2_findboxes.pyx"],
              #libraries=["m"], # Unix-like specific
			  include_dirs=[numpy.get_include()]
    )
]
ext_modules2=[
    Extension("cy_yolo_findboxes",
              sources=["cy_yolo_findboxes.pyx"],
              #libraries=["m"] # Unix-like specific
                          include_dirs=[numpy.get_include()]
    )
]
ext_modules3=[
    Extension("nms",
              sources=["nms.pyx"],
              #libraries=["m"] # Unix-like specific
                          include_dirs=[numpy.get_include()]
    )
]

setup(

    name= 'cy_yolo2_findboxes',
    ext_modules = cythonize(ext_modules),
)

setup(
    name= 'cy_yolo_findboxes',
    ext_modules = cythonize(ext_modules2),
)

setup(
    name= 'nms',
    ext_modules = cythonize(ext_modules3),
)
