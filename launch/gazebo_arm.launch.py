# Copyright (c) 2023
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import yaml, xacro

#https://github.com/bponsler/ros2-support

def generate_launch_description():

    package_path = get_package_share_directory("multi_robot_arm")
    
    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time", default_value='true', description="Use simulator time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    robot_type = "ur5"  # ROBOT_MODEL  #LaunchConfiguration("robot")
    
    world = LaunchConfiguration("world")
    declare_world_path = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            package_path,
            "worlds",
            "empty.world"
        ),
        description="Full path to world model file to load",
    )

    declare_world_path = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            package_path,
            "worlds",
            "empty.world",
        ),
        description="Full path to world model file to load",
    )

    declare_robot_type = DeclareLaunchArgument(
        name="robot_type", default_value=robot_type, description="Robot type"
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            "-u",
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_init.so",
            world,
        ],
        output="screen",
    )
    gazebo_client = ExecuteProcess(cmd=["gzclient"], output="screen")




    ld = LaunchDescription()
    ld.add_action(declare_world_path)
    ld.add_action(declare_robot_type)
    ld.add_action(declare_use_sim_time)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    robots = [
            {'name': 'arm1', 'x_pose': '-1.5', 'y_pose': '-1.50', 'Y':'0.0'},
            {'name': 'arm2', 'x_pose': '-1.5', 'y_pose': '1.5', 'Y':'0.0'},
            {'name': 'arm3', 'x_pose': '1.5', 'y_pose': '-1.5', 'Y':'-3.14'},
            {'name': 'arm4', 'x_pose': '1.5', 'y_pose': '1.5', 'Y':'-3.14'},
            # …
            # …
        ]

    # Multiple ARMs in gazebo must be spawned in a serial fashion due to 
    # a global namespace dependency introduced by ros2_control.
    # robot_final_action is the last action from previous robot and 
    # new robot will only be spawned once previous action is completed
    robot_final_action = None
    for robot in robots:    
        robot_final_action = spawn_robot(
            ld,
            "ur5",
            robot["name"] ,
            use_sim_time,
            robot["x_pose"],
            robot["y_pose"],
            robot["Y"],
            robot_final_action,
        )


    return ld


def spawn_robot(
        ld, robot_type, robot_name, use_sim_time, x, y, Y,
        previous_final_action=None):

    package_path = get_package_share_directory("multi_robot_arm")
    namespace = "/" + robot_name

    param_substitutions = {"use_sim_time": use_sim_time}
    configured_params = RewrittenYaml(
        source_file=package_path
        + "/config/ur/" + robot_type + "/ros_controllers_robot.yaml",
        root_key=robot_name,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    context = LaunchContext()
    controller_paramfile = configured_params.perform(context)
    xacro_path = os.path.join(package_path, "urdf", "ur", "ur5", "ur_urdf.xacro")

    robot_doc = xacro.process_file(
        xacro_path,
        mappings={
            "name": robot_name,
            "namespace": namespace,
            "sim_gazebo": "1",
            "simulation_controllers": controller_paramfile,
            "safety_limits": "true",
            "prefix": "",
            "pedestal_height": "0.1",
        },
    )

    robot_urdf = robot_doc.toprettyxml(indent="  ")


    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robot_params = {"robot_description": robot_urdf,
                    "use_sim_time": use_sim_time}
    robot_state_publisher = Node(
        package="robot_state_publisher",
        namespace=namespace,
        executable="robot_state_publisher",
        output="screen",
        remappings=remappings,
        parameters=[robot_params],
    )

    robot_description = {"robot_description": robot_urdf}

    kinematics_yaml = load_yaml(
        package_path, "config/ur/" + robot_type + "/kinematics.yaml"
    )

    robot_description_semantic_config = load_file(
        package_path, "config/ur/" + robot_type + "/robot.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        },
    }

    ompl_planning_yaml = load_yaml(
        package_path, "config/ur/" + robot_type + "/ompl_planning.yaml"
    )

    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    joint_limits_yaml = load_yaml(
        package_path, "config/ur/" + robot_type + "/joint_limits_planning.yaml"
    )

    joint_limits = {"robot_description_planning": joint_limits_yaml}

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        package_path,
        "config/ur/" + robot_type + "/moveit_controller_manager.yaml"
    )

    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": True,
        "trajectory_execution.controller_connection_timeout": 30.0,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "default_planning_pipeline": "ESTkConfigDefault",
        "use_sim_time": use_sim_time,
    }

    pipeline_names = {"pipeline_names": ["ompl"]}

    planning_pipelines = {
        "planning_pipelines": pipeline_names,
        "default_planning_pipeline": "ompl",
    }

    # https://industrial-training-master.readthedocs.io/en/foxy/_source/session3/ros2/3-Build-a-MoveIt-Package.html
    # Start the actual move_group node/action server
    robot_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits,
            planning_pipelines,
            {"planning_plugin": "ompl", "use_sim_time": use_sim_time},
        ],
        remappings=remappings,
        arguments=["--ros-args", "--log-level", "info"],
    )


    ros_distro = os.environ.get('ROS_DISTRO')
    
    # ROS2 Controller Manager in Foxy uses 'start' while Humble version expects 'active'

    controller_run_state = 'active'
    if ros_distro == 'foxy':
        controller_run_state = 'start'

    robot_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", namespace + "/robot_description",
            "-entity", robot_name,
            "-robot_namespace", namespace,
            "-x", x,
            "-y", y,
            "-z", "0.0",
            "-Y", Y,
            "-unpause",
        ],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            controller_run_state,
            "joint_state_broadcaster",
            "-c",
            namespace + "/controller_manager",
        ],
        output="screen",
    )

    load_arm_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            controller_run_state,
            "arm_controller",
            "-c",
            namespace + "/controller_manager",
        ],
        output="screen",
    )
    message = """ {
            'header': {
                'stamp': {
                'sec': 0,
                'nanosec': 0
                },
                'frame_id': ''
            },
            'joint_names': [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ],
            'points': [
                {
                'positions': [0.0, -0.97, 2.0, -2.56, -1.55, 0.0],
                'velocities': [],
                'accelerations': [],
                'effort': [],
                'time_from_start': {
                    'sec': 1,
                    'nanosec': 0
                }
                }
            ]
            }"""

    # Set initial joint position for robot.   This step is not needed for Humble 
    # In Humble, initial positions are taken from initial_positions.yaml and set by ros2 control plugin
    set_initial_pose = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/" + robot_name + "/arm_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            message,
        ],
        output="screen",
    )
    
    if previous_final_action is not None:
        spawn_entity = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=previous_final_action,
                on_exit=[robot_spawn_entity],
            )
        )
    else:
        spawn_entity = robot_spawn_entity

    state_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )
    arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_arm_trajectory_controller],
        )
    )

    set_initial_pose_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_trajectory_controller,
            on_exit=[set_initial_pose],
        )
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(robot_move_group_node)
    ld.add_action(spawn_entity)
    ld.add_action(state_controller_event)
    ld.add_action(arm_controller_event)
    ld.add_action(set_initial_pose_event)

    return load_arm_trajectory_controller


def load_file(package_path, file_path):

    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_path, file_path):

    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
