from glob import glob
from setuptools import setup

setup(
    name="dragonfly_fcu_bridge",
    version="0.1.0",
    packages=["dragonfly_fcu_bridge"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/dragonfly_fcu_bridge"]),
        ("share/dragonfly_fcu_bridge", ["package.xml"]),
        ("share/dragonfly_fcu_bridge/config", glob("config/*.yaml")),
        ("share/dragonfly_fcu_bridge/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Xrain118",
    maintainer_email="zhangxusheng118@gmail.com",
    description="Physical body-wrench to PX4 normalized setpoint bridge",
    license="Apache-2.0",
    entry_points={"console_scripts": ["fcu_bridge = dragonfly_fcu_bridge.fcu_bridge:main"]},
)
