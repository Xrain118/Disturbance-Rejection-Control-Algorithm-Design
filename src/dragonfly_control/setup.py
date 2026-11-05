from setuptools import find_packages, setup


package_name = "dragonfly_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dragonfly",
    maintainer_email="dragonfly@example.com",
    description="Simulator-independent coupled dynamics, feedforward and NDOB algorithms.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
