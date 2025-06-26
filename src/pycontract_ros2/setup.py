from setuptools import setup, find_packages

package_name = "pycontract_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=[
        "pycontract",
        "rosidl_runtime_py",
    ],
    zip_safe=True,
    maintainer="Nicolas Rouquette",
    maintainer_email="nfr@jpl.nasa.gov",
    description="Helper utilities for integrating PyContract monitors with ROS 2 message types.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
