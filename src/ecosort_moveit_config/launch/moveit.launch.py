import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_args = [
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Lanzar RViz con el plugin Motion Planning. Apagado por '
                        'defecto: no hace falta para pick_place_node y, corriendo '
                        'junto a Gazebo, la carga de CPU/GPU provoca saltos en el '
                        'reloj simulado que a veces hacen crashear a RViz '
                        '(segfault visto en pruebas). Prender solo para inspeccionar '
                        'un plan a mano: launch_rviz:=true'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Usar el reloj de Gazebo (debe coincidir con sim.launch.py)'
        ),
    ]

    # El URDF vive en ecosort_sim, no en este paquete de config: se lo
    # indicamos a MoveItConfigsBuilder con una ruta absoluta.
    urdf_xacro_path = Path(get_package_share_directory('ecosort_sim')) / 'urdf' / 'my_robot.urdf.xacro'

    moveit_config = (
        MoveItConfigsBuilder(robot_name='ur5e_con_gripper', package_name='ecosort_moveit_config')
        .robot_description(file_path=urdf_xacro_path)
        .robot_description_semantic(Path('srdf') / 'ecosort.srdf.xacro')
        .trajectory_execution(file_path=Path('config') / 'moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    # IMPORTANTE: este launch NO levanta la simulación. Se lanza aparte,
    # después de `ros2 launch ecosort_sim sim.launch.py` (que ya publica
    # /robot_description, /tf, y tiene el controller_manager con
    # ur_manipulator_controller activo).
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
        ],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('ecosort_moveit_config'), 'config', 'moveit.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        condition=IfCondition(launch_rviz),
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(declare_args + [move_group_node, rviz_node])
