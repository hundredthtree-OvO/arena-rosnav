import os
from glob import glob

from setuptools import find_packages, setup


package_name = "toilet_benchmark_ui"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where=".", include=[f"{package_name}*"]),
    package_dir={"": "."},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
        (os.path.join("share", package_name, "examples"), glob("examples/*.json")),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="stardust",
    maintainer_email="stardust@example.com",
    description="Standalone 2D route editor for the toilet benchmark.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "route_editor_node = toilet_benchmark_ui.editor_node:main",
        ]
    },
)
