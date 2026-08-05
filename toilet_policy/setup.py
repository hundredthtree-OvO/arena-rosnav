from setuptools import find_packages, setup


setup(
    name="toilet_policy",
    version="0.0.0",
    packages=find_packages(where=".", include=["toilet_policy*"]),
    package_dir={"": "."},
    install_requires=[],
    zip_safe=True,
    maintainer="stardust",
    maintainer_email="stardust@example.com",
    description="Behavior-cloning baselines for the toilet benchmark.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "toilet_policy_export = toilet_policy.export:main",
            "toilet_policy_train = toilet_policy.train:main",
        ]
    },
)
