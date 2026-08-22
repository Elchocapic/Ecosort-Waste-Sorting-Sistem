import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # 0a. Fix: GZ_SIM_SYSTEM_PLUGIN_PATH no se rellena al hacer `source
    #     /opt/ros/jazzy/setup.bash` (a diferencia de LD_LIBRARY_PATH, que sí
    #     incluye /opt/ros/jazzy/lib). Sin esto, "gz sim" no encuentra
    #     libgz_ros2_control-system.so, el plugin nunca carga y el
    #     controller_manager NO llega a existir -> los spawners se quedan
    #     esperando el servicio para siempre.
    set_gz_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.pathsep.join(filter(None, [
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''),
        ]))
    )

    # 0b. Fix: GZ_SIM_RESOURCE_PATH solo trae /opt/ros/jazzy/share tras el
    #     source. Los paquetes de este workspace (robotiq_description,
    #     ur_description, ecosort_sim...) viven en ~/robot_ws/install/*/share
    #     y no están ahí, así que Gazebo no resuelve las mallas referenciadas
    #     como package://robotiq_description/... (por eso el gripper no se ve).
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join(filter(None, [
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            *[os.path.join(p, 'share')
              for p in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if p],
        ]))
    )

    # 0c. Fix: "Exception sending a multicast message: Network is unreachable".
    #     gz-transport (la comunicación interna entre gz sim, ros_gz_bridge y
    #     ros_gz_sim create) por defecto intenta enviar sus mensajes de
    #     descubrimiento multicast por la interfaz de la ruta por defecto
    #     (aquí, la WiFi), y esa ruta ahora los rechaza. Como todo corre en
    #     esta misma máquina, forzamos que gz-transport use loopback.
    set_gz_ip = SetEnvironmentVariable('GZ_IP', '127.0.0.1')

    # 1. Buscar tu paquete y el archivo XACRO que creamos
    pkg_share = FindPackageShare('ecosort_sim').find('ecosort_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    # Convertir el XACRO a texto URDF entendible por ROS (una sola vez)
    doc = xacro.process_file(xacro_file)
    robot_description_xml = doc.toxml()
    robot_description = {'robot_description': robot_description_xml}

    # 2. Encender el Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 3. Lanzar el simulador Gazebo (Harmonic) con nuestro mundo:
    #    mesa + cámara cenital fija + 3 objetos de color (papel/plástico/vidrio)
    world_file = os.path.join(pkg_share, 'worlds', 'ecosort_world.sdf')
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # 3b. Puente de la cámara cenital: imagen y calibración a topics ROS 2.
    #     Verifica con `gz topic -l | grep camera` si el nombre exacto del
    #     topic de camera_info difiere (según versión de gz-sensors).
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/ecosort/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/ecosort/camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # 3c. Nodo de percepción por color: detecta papel/plástico/vidrio en la
    #     imagen de la cámara cenital y publica su posición 3D estimada.
    #     Va acá (no en su propia terminal) porque es infraestructura de la
    #     simulación, igual que el puente de la cámara del que depende.
    color_detector = Node(
        package='ecosort_perception',
        executable='color_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # 4. Hacer aparecer (Spawn) el robot en Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_xml,
                   '-name', 'ur5e_con_gripper',
                   '-allow_renaming', 'true',
                   '-z', '0.1']
    )

    # 4b. Puente de reloj: sin esto, el controller_manager (use_sim_time=true)
    #     nunca recibe /clock y queda repitiendo "No clock received..." para
    #     siempre. Necesario también para cualquier nodo futuro con
    #     use_sim_time=true (cámara, MoveIt2, etc.).
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 5. Encender los 3 controladores que pusimos en el YAML.
    #    --controller-manager-timeout amplio porque la primera carga de
    #    mallas/plugins en Gazebo puede tardar más que el timeout por defecto (10s).
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '60']
    )

    load_ur_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_manipulator_controller',
                   '--controller-manager-timeout', '60']
    )

    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller',
                   '--controller-manager-timeout', '60']
    )

    # 6. Secuenciar el arranque: el controller_manager solo existe DESPUÉS
    #    de que la entidad se haya creado dentro de Gazebo. Lanzar los
    #    spawners en paralelo con el spawn (como estaba antes) es justo lo
    #    que provoca el "Could not contact service /controller_manager/...".
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    delay_ur_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_ur_controller],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_ur_controller,
            on_exit=[load_gripper_controller],
        )
    )

    # 7. Devolver la lista de todo lo que debe ejecutarse
    #    (las variables de entorno van primero para que ya estén activas
    #    cuando arranque el proceso "gz sim")
    return LaunchDescription([
        set_gz_plugin_path,
        set_gz_resource_path,
        set_gz_ip,
        node_robot_state_publisher,
        gazebo,
        bridge_clock,
        bridge_camera,
        color_detector,
        gz_spawn_entity,
        delay_joint_state_broadcaster,
        delay_ur_controller,
        delay_gripper_controller,
    ])
