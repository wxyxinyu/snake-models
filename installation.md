Installation Notes
============================

Building gazebo
--------
* follow the instructions [here](http://gazebosim.org/tutorials?tut=install_from_source&cat=install)
* ros jade works with gazebo5 which works with dart4
* dependencies: There's a possible issure where I had some incompatible dependencies of ffmpeg installed on my computer. Uninstalling everything ffmpeg-related and following the installation instructions [here](https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu) fixed it
* environment variables setup (step 8 in the tutorial)
    * add folder with gazebo_common.so to LD_INCLUDE_PATH
    * add gazebo, sdformat and ignition math folders to CPLUS_INCLUDE_PATH
    * for me, I added ```export CPLUS_INCLUDE_PATH="/usr/include/eigen3:/usr/include/bullet:/usr/local/include/gazebo-5.2:/usr/include/sdformat-3.5:/usr/include/ignition/math2:/usr/include:$CPLUS_INCLUDE_PATH"``` and ```export LD_LIBRARY_PATH="/usr/local/lib:/usr/local/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"``` to the end of my bashrc

Plugins
------
I followed the tutorials [here](http://gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin) and [here](http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin).
