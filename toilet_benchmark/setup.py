import os
from glob import glob

from setuptools import find_packages, setup

package_name = "toilet_benchmark"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(where=".", include=[f"{package_name}*"]),
    package_dir={"": "."},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "docs"), glob("docs/*.md")),
        (os.path.join("share", package_name, "hook"), glob("hook/*")),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="stardust",
    maintainer_email="stardust@example.com",
    description="Toilet-scene benchmark director skeleton for Arena-Rosnav + Isaac + Hunav.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "toilet_director_node = toilet_benchmark.toilet_director_node:main",
            "manual_collection_node = toilet_benchmark.manual_collection_node:main",
            "hunav_phase0_smoke = toilet_benchmark.hunav_phase0_smoke:main",
            "hunav_isaac_mirror = toilet_benchmark.hunav_isaac_mirror:main",
            "walkable_map_publisher = toilet_benchmark.walkable_map:main",
        ]
    },
)
