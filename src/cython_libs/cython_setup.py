"""
use the following to build:

python3 cython_setup.py build_ext --inplace
"""

from distutils.core import setup
from Cython.Build import cythonize

setup(ext_modules=cythonize("./feagi/cython_libs/neuron_functions_cy.pyx"))
# setup(ext_modules=cythonize("ipu_vision_cy.pyx"))
