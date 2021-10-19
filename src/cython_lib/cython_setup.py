"""
use the following to build:

python3 cython_setup.py build_ext --inplace


WARNING: The presence of __init.py__ inside the cython folder will cause the build creation to fail!

"""

from distutils.core import setup
from Cython.Build import cythonize

setup(ext_modules=cythonize("./neuron_functions_cy.pyx"))
# setup(ext_modules=cythonize("ipu_vision_cy.pyx"))
