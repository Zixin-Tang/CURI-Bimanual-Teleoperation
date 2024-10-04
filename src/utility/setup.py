from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d["packages"] = [
    "utility"
]
d["package_dir"] = {"": "src"}

setup(**d)
