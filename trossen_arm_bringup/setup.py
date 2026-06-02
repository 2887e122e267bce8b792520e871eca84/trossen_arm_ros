import glob
from setuptools import find_packages, setup, Extension
from setuptools.command.build_py import build_py as _build_py

package_name = 'trossen_arm_bringup'

sourceFiles = [
    "trossen_arm_bringup/trossen_arm_bringup_python.pyx",
    "trossen_arm_bringup/armor_kit.py",
    "trossen_arm_bringup/dual_trossen_arm.py",
    "trossen_arm_bringup/gravity_compensation.py",
    "trossen_arm_bringup/trossen_arm.py",
]

# Only invoke Cython if the .so hasn't been pre-built
if glob.glob(f"{package_name}/{package_name}*.so"):
    extensions = []
else:
    from Cython.Build import cythonize
    extensions = cythonize(Extension(
        name=f"{package_name}.{package_name}",
        sources=sourceFiles
    ))

class build_py(_build_py):
    def find_package_modules(self, package, package_dir):
        modules = super().find_package_modules(package, package_dir)
        return [
            (pkg, mod, file) for pkg, mod, file in modules
            if not (pkg == package_name and mod == package_name)
        ]

setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    ext_modules=extensions,
    cmdclass={'build_py': build_py},
)