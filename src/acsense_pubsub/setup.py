from setuptools import find_packages, setup  # type: ignore

package_name = "acsense_pubsub"

setup(
    name=package_name,
    version="0.0.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Acbotics Research LLC",
    maintainer_email="support@acbotics.com",
    description="ROS2 pub/sub for Acbotics AcSense",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ac_publisher = acsense_pubsub.acsense_raw_publisher:main",
            "ac_subscriber = acsense_pubsub.acsense_raw_subscriber:main",
            "beamformer_raw_publisher = acsense_pubsub.acsense_beamformer_raw_publisher:main",
            "beamformer_2d_publisher = acsense_pubsub.acsense_beamformer_2d_publisher:main",
            "beamformer_subscriber = acsense_pubsub.acsense_beamformer_subscriber:main",
        ],
    },
)
