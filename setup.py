# USE CATKIN TO INVOKE
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# FETCH VALUSE FROM package.xml
setup_args = generate_distutils_setup(
	packages=['radar_localization'],
	package_dir={'': 'src'},
)

setup(**setup_args)
