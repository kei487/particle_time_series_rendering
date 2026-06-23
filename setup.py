from setuptools import setup


package_name = "particle_time_series_rendering"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/particle_time_series_rendering.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="keitaro",
    maintainer_email="keitaro@example.com",
    description="ROS 2 node for rendering particle time-series overlays on a static map.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "particle_time_series_rendering_node = "
            "particle_time_series_rendering.particle_time_series_node:main",
        ],
    },
)
