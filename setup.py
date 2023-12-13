from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    name="reactorx200_ros_reacher",
    packages=['reactorx200_ros_reacher'],
    package_dir={'': 'src'},

    description="Simple Reach Task for RealROS and MultiROS Packages",
    url="https://github.com/ncbdrck/reactorx200_ros_reacher",
    keywords=['ROS', 'reinforcement learning', 'machine-learning', 'gym', 'robotics', 'openai', 'gazebo', 'realros',
              'multiros', 'reactorx200'],

    author='Jayasekara Kapukotuwa',
    author_email='j.kapukotuwa@research.ait.ie',
)

setup(**setup_args)