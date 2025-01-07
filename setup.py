from setuptools import setup, find_packages
from Cython.Build import cythonize

setup(
    name="stara",
    version="0.1",
    packages=find_packages(),
    ext_modules=cythonize("stara/pathfinders/a_star_cython.pyx", language_level=3),
)
