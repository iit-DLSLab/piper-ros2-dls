import os

from setuptools import find_packages, setup


package_name = "piper_sdk"
here = os.path.abspath(os.path.dirname(__file__))
sdk_root_rel = os.path.join("..", "..", "..", "piper_sdk")
sdk_root = os.path.abspath(os.path.join(here, sdk_root_rel))


setup(
    name=package_name,
    version="0.6.1",
    packages=find_packages(where=sdk_root_rel, include=["piper_sdk", "piper_sdk.*"]),
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
        "": ["LICENSE", "*.sh", "*.MD"],
        "piper_sdk": ["*.sh"],
        "piper_sdk.demo": ["*.MD"],
    },
    zip_safe=True,
    author="Agilex Robotice Co., Ltd.",
    author_email="",
    maintainer="Giulio Turrisi",
    maintainer_email="giulio.turrisi@iit.it",
    description="Piper Python SDK packaged for ROS 2.",
    long_description=open(os.path.join(sdk_root, "DESCRIPTION.MD"), encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    license="MIT",
    platforms=["Linux"],
    python_requires=">=3.6",
)
