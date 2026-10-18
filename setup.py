from setuptools import setup
from glob import glob
import os

package_name='panda_fk_ik_demo'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+package_name]),
                  ('share/'+package_name, ['package.xml']),
                  (os.path.join('share',package_name,'launch'), glob('launch/*.launch.py')),
                  (os.path.join('share',package_name,'rviz'), glob('rviz/*.rviz'))],
      install_requires=['setuptools', 'numpy'], zip_safe=True,
      maintainer='Robotics Teaching Lab', maintainer_email='example@example.com',
      description='Panda URDF FK/IK RViz-only teaching demonstration', license='Apache-2.0',
      entry_points={'console_scripts': ['demo = panda_fk_ik_demo.demo:main']})
