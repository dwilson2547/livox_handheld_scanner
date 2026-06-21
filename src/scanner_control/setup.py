from setuptools import find_packages, setup

package_name = "scanner_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/web", ["web/index.html"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dan",
    maintainer_email="dan@example.com",
    description="Browser control surface for the handheld scanner.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scanner_control_server = scanner_control.control_server:main",
        ],
    },
)
