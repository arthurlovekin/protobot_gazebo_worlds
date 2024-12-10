How to run
---
```
cd path/to/ros2_workspace
colcon build --symlink-install --packages-select protobot_gazebo_worlds
source install/setup.bash # automatically sets GZ_SIM_RESOURCE_PATH
ros2 launch protobot_gazebo_worlds gazebo.launch.py
```
If you want to run parts individually, you can first check that the world works (without the robot) by running `gz sim <path/to/world.sdf>` eg.
`gz sim ~/Projects/protobot_ws/src/protobot_gazebo_worlds/worlds/simple.sdf`
Then compile a urdf file from the xacro in robot_description:
`xacro <model_name>.xacro > <model_name>.urdf`

Finally, spawn the urdf into the world
`gz service -s /world/simple_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/protobot.urdf", name: "protobot"'`

Package structure
---
- launch
    - Launch file that brings up a Gazebo world (with no robot).
- worlds
    - sdf world files

Tutorials/Documentation
---
- Official Gazebo Docs
    - [Guide to rs_gz_project_template](https://gazebosim.org/docs/latest/ros_gz_project_template_guide/#accessing-simulation-assets)
    - [Spawn URDF](https://gazebosim.org/docs/latest/spawn_urdf/)
- [Gazebosim/ros_gz (official repo of Gazebo)](https://github.com/gazebosim/ros_gz)
    - [ros_gz_sim package docs](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim)


More Notes from the development process
---
In general. you have to let gazebo know where to look for downloaded assets:
`export GZ_SIM_RESOURCE_PATH="<path/to/folder/containing/assets>"`
For example, in my case some meshes are in the gazebo package, and others (referenced by the urdf) are in the protobot_description package
`export GZ_SIM_RESOURCE_PATH="$HOME/Projects/protobot_ws/install/protobot_gazebo_worlds/share/protobot_gazebo_worlds/worlds:$HOME/Projects/protobot_ws/install/protobot_description/share"`

The official [ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template/tree/main/ros_gz_example_gazebo/hooks) shows how to automatically set the GZ_SIM_RESOURCE_PATH using a .dsv.in file that hooks into the ament build process. 
However, I chose instead to use the <export> tags in the package.xml, as described by the makers of the [ros_gz_sim](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim). They actually recommended I use `<gazebo_ros gazebo_model_path="${prefix}/../"/>` but I couldn't get this to work.
