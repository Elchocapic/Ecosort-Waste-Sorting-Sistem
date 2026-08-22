from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # MoveGroupInterface (dentro de pick_place_node) arma su PROPIA copia
    # local del modelo del robot al construirse: si no le damos
    # robot_description/robot_description_semantic como parámetros del
    # propio nodo, se queda esperando a que alguien los publique como topic
    # -- y nada publica la SRDF como topic en nuestro setup. Le pasamos la
    # misma config que usa moveit.launch.py para move_group, así no depende
    # de descubrir nada de otra terminal para este paso.
    urdf_xacro_path = Path(get_package_share_directory('ecosort_sim')) / 'urdf' / 'my_robot.urdf.xacro'

    moveit_config = (
        MoveItConfigsBuilder(robot_name='ur5e_con_gripper', package_name='ecosort_moveit_config')
        .robot_description(file_path=urdf_xacro_path)
        .robot_description_semantic(Path('srdf') / 'ecosort.srdf.xacro')
        .trajectory_execution(file_path=Path('config') / 'moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    pick_place_node = Node(
        package='ecosort_pick_place',
        executable='pick_place_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([pick_place_node])
