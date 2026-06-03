import os

from setuptools import find_packages, setup


package_name = "piper_sdk"
here = os.path.abspath(os.path.dirname(__file__))
sdk_root_rel = os.path.join("..", "..", "..", "pyAgxArm")
sdk_root = os.path.abspath(os.path.join(here, sdk_root_rel))
about = {}
with open(os.path.join(sdk_root, "pyAgxArm", "version.py"), encoding="utf-8") as version_file:
    exec(version_file.read(), about)


setup(
    name=package_name,
    version=about["__version__"],
    packages=find_packages(where=sdk_root_rel, include=["pyAgxArm", "pyAgxArm.*"]),
    package_dir={"": sdk_root_rel},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "python-can>=3.3.4",
        "typing-extensions",
    ],
    include_package_data=True,
    package_data={
        "*": ["*.pyi"],
        "pyAgxArm": ["py.typed"],
    },
    zip_safe=True,
    author="Agilex Robotics Co., Ltd.",
    author_email="",
    maintainer="Giulio Turrisi",
    maintainer_email="giulio.turrisi@iit.it",
    description="pyAgxArm Python SDK packaged for ROS 2.",
    long_description=open(os.path.join(sdk_root, "README.md"), encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    license="LGPL-3.0-only",
    platforms=["Linux", "Windows", "Darwin"],
    python_requires=">=3.6",
)
