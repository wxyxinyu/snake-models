Installation Notes
============================

ros jade works with gazebo5 works with dart4
    also need to install a bunch of libraries
    look at compile errors and install whatever libraries you don't have
environment variables setup
    add gazebo_common.so folder to LD_INCLUDE_PATH
    add gazebo, sdformat and ignition math installed folders (for me it was usr/include/[gazebo-5.2|sdformat-3.5|ignition/math2])

follow plugins tutorial to create CMakeLists.txt for compilation.
