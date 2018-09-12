# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
f = generate_distutils_setup(
    packages=['fome_gui'],
    package_dir={'': 'src'},
    scripts=['scripts/fome_gui']
)

setup(**f)
