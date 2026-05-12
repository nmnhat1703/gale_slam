cmake_minimum_required(VERSION 3.0.2)
project(microstrain_inertial_description)

find_package(catkin REQUIRED)
catkin_package()

# Install the launch and config files
install(DIRECTORY urdf meshes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)