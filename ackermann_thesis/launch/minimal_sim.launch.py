"""
Minimal simulator launch file for Ackermann robot.

This launch file starts a minimal Gazebo simulation with the robot model.
It's designed to work without the full robotverseny_gazebo24 dependencies.

Requirements:
- ros-humble-ros-gz-sim (install with: sudo apt install ros-humble-ros-gz-sim)
- ros-humble-ros-gz-bridge (usually already installed)
- robotverseny_description package (from robotverseny_gazebo24)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    pkg_ackermann_thesis = get_package_share_directory('ackermann_thesis')
    pkg_robotverseny_description = get_package_share_directory('robotverseny_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',  # We'll create a minimal world
        description='World file to load'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI, useful for WSL)'
    )
    
    # Load robot model SDF
    sdf_file = os.path.join(pkg_robotverseny_description, 'models', 'roboworks', 'model.sdf')
    
    if not os.path.exists(sdf_file):
        return LaunchDescription([
            LogInfo(msg=f'ERROR: Robot model not found at {sdf_file}'),
            LogInfo(msg='Make sure robotverseny_description is built and sourced'),
        ])
    
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Create a minimal empty world (if needed)
    # For now, we'll try to use the existing world or create a simple one
    world_file = os.path.join(pkg_robotverseny_description, 'models', 'roboworks', 'model.sdf')
    
    # Try to find an existing world file
    # Check if robotverseny_gazebo is built
    try:
        pkg_robotverseny_gazebo = get_package_share_directory('robotverseny_gazebo')
        world_paths = [
            os.path.join(pkg_robotverseny_gazebo, 'worlds', 'roboworks.sdf'),
            os.path.join(pkg_robotverseny_gazebo, 'worlds', 'empty.sdf'),
        ]
        
        world_file = None
        for path in world_paths:
            if os.path.exists(path):
                world_file = path
                break
    except:
        world_file = None
    
    if world_file is None:
        # Create a minimal world inline
        # Create a truly headless world - sensors will work but without rendering
        world_content = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="minimal_headless">
    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    <!-- Sensors system - LiDAR will work in headless mode -->
    <plugin
      filename="ignition-gazebo-sensors-system"
      name="ignition::gazebo::systems::Sensors">
    </plugin>
    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="ignition-gazebo-user-commands-system"
      name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>"""
        # Write minimal world to temp location
        import tempfile
        temp_dir = tempfile.gettempdir()
        world_file = os.path.join(temp_dir, 'minimal_world.sdf')
        with open(world_file, 'w') as f:
            f.write(world_content)
    
    # Launch Gazebo Sim using the standard launch file
    gz_sim_launch_file = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    
    if not os.path.exists(gz_sim_launch_file):
        return LaunchDescription([
            LogInfo(msg=f'ERROR: ros_gz_sim launch file not found at {gz_sim_launch_file}'),
            LogInfo(msg='Install with: sudo apt install ros-humble-ros-gz-sim'),
        ])
    
    # Build gz_args - use headless mode (-s) to avoid OGRE rendering issues in WSL
    # -s = server only (no GUI), -r = run immediately, -v 1 = verbosity level 1
    gz_args = f'-s -r -v 1 {world_file}'
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch_file]),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'True'
        }.items()
    )
    
    # Note: robot_state_publisher is not needed here because:
    # 1. The robot model is in SDF format (not URDF)
    # 2. Gazebo publishes TF transforms directly via the bridge
    # If you need robot_state_publisher, you would need to convert SDF to URDF using sdformat_urdf
    
    # Bridge for ROS-Gazebo communication
    bridge_config = os.path.join(pkg_ackermann_thesis, 'config', 'minimal_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file': bridge_config,
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            },
            {'use_sim_time': True},
        ],
        output='screen'
    )
    
    # Spawn robot model (delayed to let Gazebo start)
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', sdf_file,
                    '-name', 'roboworks',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )
    
    # RViz (optional, delayed) - only launch once
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        rviz_arg,
        world_arg,
        headless_arg,
        LogInfo(msg='Starting minimal Gazebo simulation...'),
        LogInfo(msg=f'World file: {world_file}'),
        LogInfo(msg=f'Robot model: {sdf_file}'),
        LogInfo(msg='Note: Running without robot_state_publisher (TF comes from Gazebo bridge)'),
        LogInfo(msg='Running Gazebo in headless mode (-s flag) to avoid WSL rendering issues'),
        gz_sim,  # Start Gazebo Sim first
        bridge,
        spawn_robot,  # Spawn robot after a delay
        rviz,
    ])
