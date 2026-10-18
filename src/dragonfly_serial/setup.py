from glob import glob
from setuptools import setup
setup(name='dragonfly_serial', version='0.1.0', packages=['dragonfly_serial'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/dragonfly_serial']),
                  ('share/dragonfly_serial', ['package.xml']),
                  ('share/dragonfly_serial/config', glob('config/*')),
                  ('share/dragonfly_serial/launch', glob('launch/*'))],
      install_requires=['setuptools', 'pyserial'], zip_safe=True,
      maintainer='Xrain118', maintainer_email='zhangxusheng118@gmail.com',
      description='Dual arm telemetry without simulation dependencies', license='Apache-2.0',
      entry_points={'console_scripts': ['dual_arm_serial = dragonfly_serial.node:main',
                                       'telemetry_replay = dragonfly_serial.node:replay_main']})
