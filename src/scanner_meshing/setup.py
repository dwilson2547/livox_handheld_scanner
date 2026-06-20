from setuptools import find_packages, setup

package_name = "scanner_meshing"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dan",
    maintainer_email="dan@example.com",
    description="VDBFusion live meshing for the handheld scanner.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vdb_meshing_node = scanner_meshing.vdb_meshing_node:main",
        ],
    },
)
