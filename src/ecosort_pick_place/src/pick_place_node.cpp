// EcoSort - pick_place_node
//
// Ciclo de recoger-y-depositar para los 3 objetos de ecosort_world.sdf.
// La posición de AGARRE de cada objeto viene de ecosort_perception (nodo
// color_detector, topics /ecosort/detected/<categoría>) -- son objetos que
// pueden estar en cualquier posición sobre la mesa, no solo la del spawn
// original. Si una categoría nunca llega a detectarse, se asume que ese
// residuo simplemente no está sobre la mesa y se omite (no se usa ninguna
// posición fija de respaldo: intentar agarrar donde ya no hay nada solo
// generaba viajes largos e inútiles antes de llegar al objeto real). La
// posición de DEPÓSITO (el tacho) sigue fija: los tachos no se mueven, no
// hace falta detectarlos.
//
// Usa moveit::planning_interface::MoveGroupInterface (C++) porque
// moveit_py no está instalado/disponible en este sistema; es el camino
// oficial de MoveIt2 para pick-and-place sin bindings de Python.
//
// El gripper NO pasa por MoveIt (ver config/moveit_controllers.yaml del
// paquete ecosort_moveit_config para el porqué): se comanda publicando
// directo a /gripper_controller/commands.

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;

struct SortItem
{
  std::string category;
  // pick_x/pick_y ya NO se usan como respaldo (ver comentario de cabecera);
  // se dejan solo de referencia/documentación de la posición de spawn
  // original del SDF. pick_z sí se usa siempre (altura constante conocida).
  double pick_x, pick_y, pick_z;
  double place_x, place_y, place_z; // coordenadas del SDF (bin de destino, fijo)
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "pick_place_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // MoveGroupInterface hace llamadas síncronas a servicios/acciones de
  // move_group por debajo; necesita que ESTE nodo esté girando en un
  // executor en paralelo o se bloquea esperando su propia respuesta.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  auto logger = node->get_logger();

  auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/gripper_controller/commands", 10);

  // Última posición detectada por color_detector (ecosort_perception) para
  // cada categoría, en coordenadas del SDF (mismo frame que las fijas de
  // respaldo). Si nunca llega nada para una categoría, se usa el respaldo.
  std::map<std::string, geometry_msgs::msg::Point> detections;
  auto make_detection_sub = [&](const std::string & category) {
      return node->create_subscription<geometry_msgs::msg::PointStamped>(
        "/ecosort/detected/" + category, 10,
        [&detections, category](geometry_msgs::msg::PointStamped::SharedPtr msg) {
          detections[category] = msg->point;
        });
    };
  auto sub_papel = make_detection_sub("papel");
  auto sub_plastico = make_detection_sub("plastico");
  auto sub_vidrio = make_detection_sub("vidrio");

  // Posición REAL del joint del gripper (no la comandada): sirve para
  // detectar un agarre fallido -- si al "cerrar" el joint llega casi hasta
  // el valor comandado, es que no encontró ninguna resistencia (no había
  // nada entre los dedos). Sin esto el nodo no tiene forma de saber que
  // el cubo se le resbaló y sigue la trayectoria como si lo llevara.
  double gripper_joint_pos = 0.0;
  auto sub_joint_states = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [&gripper_joint_pos](sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "robotiq_85_left_knuckle_joint") {
          gripper_joint_pos = msg->position[i];
          break;
        }
      }
    });

  // Cliente directo al controlador del brazo (sin pasar por MoveIt), SOLO
  // para la recuperación de emergencia de go_home() -- ver ese comentario.
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  auto trajectory_client = rclcpp_action::create_client<FollowJointTrajectory>(
    node, "/ur_manipulator_controller/follow_joint_trajectory");
  // dar tiempo a que el publisher matchee con el subscriber del controlador
  rclcpp::sleep_for(1s);

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.4);
  move_group.setMaxAccelerationScalingFactor(0.4);
  move_group.setGoalPositionTolerance(0.005);
  // Subido de 0.02 a 0.05 rad (~3deg): en zonas cercanas a la base del
  // robot (sobre todo con y negativa) el planeo a veces agota sus 4
  // intentos completos (10s cada uno) sin encontrar NINGÚN camino hacia la
  // orientación "mirando derecho hacia abajo" exacta -- darle un poco más
  // de margen angular amplía el conjunto de soluciones de IK válidas sin
  // afectar visiblemente el agarre (gripper plano sobre un cubo chico).
  move_group.setGoalOrientationTolerance(0.05);
  // RRTConnect (el planificador por defecto) encuentra un camino válido y
  // se detiene ahí -- puede ser un rodeo larguísimo aunque exista uno
  // corto. Con varios intentos internos por llamada a plan(), MoveIt los
  // compara y se queda con el más corto en vez del primero que aparezca.
  move_group.setNumPlanningAttempts(10);

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // ----------------------------------------------------------------------
  // Constantes de calibración -- ver comentarios de cada una.
  // ----------------------------------------------------------------------

  // sim.launch.py spawnea el robot con "-z 0.1" (ver gz_spawn_entity), así
  // que el link "world" del URDF (= planning frame de MoveIt, ya que la
  // SRDF no declara virtual_joint y se usa la raíz del URDF) queda 0.1m
  // por encima del frame "world" del SDF, donde definimos mesa/objetos/
  // bins con esas mismas coordenadas. Restamos este offset para pasar de
  // coordenadas del SDF a coordenadas del planning frame de MoveIt.
  const double kSpawnZOffset = 0.1;

  // Distancia real, a lo largo del eje +Z local de tool0, entre tool0 y la
  // punta del dedo. La estimación geométrica original (0.10m, sumando
  // orígenes de joints) quedaba corta: medido con /check_state_validity
  // contra la mesa real (contacto a profundidad ~0mm con tool0 a Z=0.42),
  // el valor real es ~0.12m. Se deja 0.125 para un pequeño margen y no
  // rozar la mesa.
  const double kToolToGraspOffsetZ = 0.125;

  // Altura de aproximación/retirada por encima del punto de agarre/destino.
  const double kApproachHeight = 0.15;

  // Margen de seguridad extra para el descenso final de agarre/depósito.
  // Medido con TF (tool0 -> robotiq_85_left_finger_tip_link): el offset
  // real es 0.110m, no 0.125 como se asumía. Con margen de 1.5cm algunos
  // casos llegaban a 100% y otros se frenaban en 86-93% (cada objeto se
  // agarra desde un ángulo distinto del brazo, así que el punto de
  // contacto exacto varía). Subido a 2.5cm: el cubo mide 4cm, así que
  // agarrar 2.5cm arriba de su centro sigue estando dentro del cubo.
  const double kGraspSafetyMarginZ = 0.025;

  // Orientación con el gripper mirando derecho hacia abajo.
  // Por la cadena de joints del gripper (todas las articulaciones del
  // mecanismo se offsetean en +Z local de robotiq_85_base_link, que
  // coincide exactamente con tool0 -- robotiq_85_base_joint tiene origen
  // identidad), el eje +Z local de tool0 ES el eje de aproximación del
  // gripper. Una rotación de 180° sobre el eje X del planning frame
  // ("world") deja ese eje +Z local apuntando a -Z mundo, es decir: hacia
  // abajo. No depende de la configuración del brazo (es la orientación
  // absoluta que le pedimos al solver de IK).
  geometry_msgs::msg::Quaternion down_orientation;
  down_orientation.x = 1.0;
  down_orientation.y = 0.0;
  down_orientation.z = 0.0;
  down_orientation.w = 0.0;

  const double kGripperOpen = 0.0;
  // Más cerca del límite (0.8 / gripper_closed_position=0.7929) que antes
  // (0.62): al ser un position_controller, comandar una posición "más
  // allá" de donde está realmente el objeto es justamente lo que genera
  // fuerza de agarre (vía el error de posición y la ganancia del
  // controlador) -- con 0.62 probablemente no llegaba a hacer contacto
  // firme. No se deja el límite exacto para no saturar el controlador.
  const double kGripperClosed = 0.79;

  // Umbral para detectar un agarre "en el aire" o demasiado débil: si al
  // cerrar el gripper el joint real (gripper_joint_pos) queda por ENCIMA
  // de este valor, se considera agarre fallido/insuficiente y se reintenta.
  // Calibrado con datos reales de pruebas: los agarres firmes cierran
  // sistemáticamente cerca de 0.45 (papel y vidrio, varias corridas), muy
  // lejos del comandado 0.79 -- pero un agarre visto en 0.634 (mucho más
  // cerca del comandado que los buenos) resultó en fallos intermitentes al
  // llevar el objeto al tacho, probablemente por resbalarse en el trayecto
  // largo. 0.60 deja margen sobre los agarres buenos conocidos (~0.45-0.50)
  // pero atrapa ese caso débil como el de 0.634 para forzar un reintento.
  const double kGripFailThreshold = 0.60;

  // ----------------------------------------------------------------------
  // Objeto de colisión: la mesa. Sin esto, MoveIt no sabe que existe (vive
  // solo en el SDF de Gazebo) y podría planear trayectorias que la
  // atraviesen visualmente. Los objetos pequeños y los bins NO se agregan
  // en esta primera versión -- limitación conocida, ver mensaje de chat.
  // ----------------------------------------------------------------------
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  {
    // Solo la TAPA de la mesa (ver ecosort_world.sdf: ahora tiene patas
    // finas y espacio libre debajo, ya no es un bloque sólido desde el
    // piso). Un bloque sólido como antes hacía que ciertas configuraciones
    // del codo, cerca de la base del robot, chocaran contra la mesa aunque
    // el punto de agarre en sí fuera alcanzable (visto con
    // /check_state_validity: upper_arm_link contra work_table).
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = move_group.getPlanningFrame();
    table.id = "work_table";

    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape.BOX;
    shape.dimensions = {0.6, 0.8, 0.03};  // solo la tapa, igual que en ecosort_world.sdf

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.45;
    pose.position.y = 0.0;
    pose.position.z = 0.385 - kSpawnZOffset;
    pose.orientation.w = 1.0;

    table.primitives.push_back(shape);
    table.primitive_poses.push_back(pose);
    table.operation = table.ADD;
    planning_scene_interface.applyCollisionObject(table);

    // Las 4 patas (ver ecosort_world.sdf: pata_1..pata_4). Faltaban --
    // MoveIt encontraba caminos que rozaban una pata (invisibles para él)
    // y los reportaba como "éxito" porque no sabía que existían, aunque en
    // Gazebo sí chocaban de verdad.
    const double kLegRadius = 0.02;
    const double kLegLength = 0.37;
    const double kLegZ = 0.185 - kSpawnZOffset;
    std::vector<std::pair<double, double>> leg_xy = {
      {0.20, 0.35}, {0.20, -0.35}, {0.70, 0.35}, {0.70, -0.35}
    };
    for (size_t i = 0; i < leg_xy.size(); ++i) {
      moveit_msgs::msg::CollisionObject leg;
      leg.header.frame_id = move_group.getPlanningFrame();
      leg.id = "table_leg_" + std::to_string(i + 1);

      shape_msgs::msg::SolidPrimitive leg_shape;
      leg_shape.type = leg_shape.CYLINDER;
      leg_shape.dimensions = {kLegLength, kLegRadius};  // [height, radius]

      geometry_msgs::msg::Pose leg_pose;
      leg_pose.position.x = leg_xy[i].first;
      leg_pose.position.y = leg_xy[i].second;
      leg_pose.position.z = kLegZ;
      leg_pose.orientation.w = 1.0;

      leg.primitives.push_back(leg_shape);
      leg.primitive_poses.push_back(leg_pose);
      leg.operation = leg.ADD;
      planning_scene_interface.applyCollisionObject(leg);
    }
    rclcpp::sleep_for(500ms);
  }

  // Posiciones de picking y de destino (coordenadas del SDF, tal cual
  // ecosort_world.sdf) -- luego reemplazadas por percepción en Fase 3b.
  std::vector<SortItem> items = {
    {"papel",    0.35, 0.15, 0.42, 0.65, 0.25, 0.405},
    // place_y = 0.03 (no 0.00): el bin de plástico está justo al frente
    // del robot (y=0), en el eje de simetría del brazo -- ahí la muñeca
    // cae cerca de una singularidad cinemática y el planificador falla
    // casi siempre para llegar. 3cm no se nota (el tacho mide 14cm de
    // ancho) pero saca el objetivo de esa línea exacta.
    {"plastico", 0.45, 0.00, 0.42, 0.65, 0.03, 0.405},
    {"vidrio",   0.55, -0.15, 0.42, 0.65, -0.25, 0.405},
  };

  // settle_ms más largo al cerrar (agarrar): hay que darle tiempo al
  // position_controller para que realmente alcance el error de posición
  // contra el objeto (y que ese "empuje" se sienta como agarre) antes de
  // que el brazo empiece a moverse -- si se mueve muy rápido después de
  // cerrar, el agarre puede no haber "asentado" todavía.
  auto set_gripper = [&](double position, int settle_ms = 1500) {
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {position};
      gripper_pub->publish(msg);
      rclcpp::sleep_for(std::chrono::milliseconds(settle_ms));
    };

  // x,y,z_sdf: coordenadas del SDF (se les resta kSpawnZOffset acá adentro)
  auto make_target = [&](double x, double y, double z_sdf) {
      geometry_msgs::msg::Pose target;
      target.position.x = x;
      target.position.y = y;
      target.position.z = (z_sdf - kSpawnZOffset) + kToolToGraspOffsetZ;
      target.orientation = down_orientation;
      return target;
    };

  // Movimiento "grande": de donde esté el brazo (que puede tener cualquier
  // orientación -- viene de 'home' o de otro objeto) hasta quedar arriba
  // del punto de interés, ya con el gripper mirando hacia abajo. Usa
  // plan()+RRTConnect (espacio articular): es la única de las dos técnicas
  // que puede resolver una reconfiguración grande como esta.
  auto move_to_joint_space = [&](const std::string & label, double x, double y,
                                  double z_sdf) -> bool {
      geometry_msgs::msg::Pose target = make_target(x, y, z_sdf);
      move_group.setPoseTarget(target);
      // Bajado de 8 a 4: cada llamada a plan() ahora ya hace 10 intentos
      // internos (setNumPlanningAttempts), así que 4×10=40 intentos totales
      // en el peor caso siguen siendo muchos, sin multiplicar la espera
      // como sería con 8×10.
      const int kMaxPlanAttempts = 4;
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool ok = false;
      for (int attempt = 1; attempt <= kMaxPlanAttempts && !ok; ++attempt) {
        ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok) {
          RCLCPP_WARN(logger, "  [%s] intento de planeo %d/%d falló, reintentando...",
            label.c_str(), attempt, kMaxPlanAttempts);
        }
      }
      if (!ok) {
        RCLCPP_ERROR(logger, "  [%s] No se pudo PLANEAR a (%.3f, %.3f, %.3f) tras %d intentos",
          label.c_str(), x, y, target.position.z, kMaxPlanAttempts);
        return false;
      }
      ok = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!ok) {
        RCLCPP_ERROR(logger, "  [%s] No se pudo EJECUTAR hacia (%.3f, %.3f, %.3f)",
          label.c_str(), x, y, target.position.z);
      }
      return ok;
    };

  // Movimiento "corto": línea recta vertical manteniendo la MISMA
  // orientación (bajar a agarrar, subir a retirarse, etc.). Usa
  // computeCartesianPath(): al ser un tramo corto con orientación
  // constante desde donde ya se está, evita el problema del "pasillo
  // angosto" que sufre RRTConnect cerca de la mesa. Requiere que el brazo
  // YA esté con el gripper hacia abajo (llamar siempre después de un
  // move_to_joint_space a la misma zona).
  // avoid_collisions=false para pick/place (bajada a agarrar/soltar,
  // acercarse a la mesa ahí es intencional) Y para retreat-pick/
  // retreat-place (retirarse por la MISMA línea recta, solo que al revés
  // -- si bajar fue seguro, subir por el mismo camino también lo es;
  // dejarlo en true ahí causaba el mismo atasco por margen que en pick).
  // move_to_joint_space (aproximación grande, con orientación variable) sí
  // mantiene la verificación de colisión siempre activa.
  auto move_to_cartesian = [&](const std::string & label, double x, double y,
                                double z_sdf, bool avoid_collisions = true) -> bool {
      geometry_msgs::msg::Pose target = make_target(x, y, z_sdf);
      std::vector<geometry_msgs::msg::Pose> waypoints = {target};
      const double kEefStep = 0.01;
      const int kMaxAttempts = 5;
      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = 0.0;
      for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
        fraction = move_group.computeCartesianPath(waypoints, kEefStep, trajectory, avoid_collisions);
        if (fraction > 0.99) {break;}
        RCLCPP_WARN(logger,
          "  [%s] intento de trayectoria cartesiana %d/%d: %.0f%% del camino, reintentando...",
          label.c_str(), attempt, kMaxAttempts, fraction * 100.0);
      }
      if (fraction <= 0.99) {
        RCLCPP_ERROR(logger,
          "  [%s] No se pudo trazar el camino completo a (%.3f, %.3f, %.3f) (%.0f%% logrado)",
          label.c_str(), x, y, target.position.z, fraction * 100.0);
        return false;
      }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory = trajectory;
      bool ok = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!ok) {
        RCLCPP_ERROR(logger, "  [%s] No se pudo EJECUTAR hacia (%.3f, %.3f, %.3f)",
          label.c_str(), x, y, target.position.z);
      }
      // Pequeño respiro tras un movimiento cartesiano antes de que el
      // siguiente paso pida un plan nuevo: visto en pruebas, un plan
      // articular inmediatamente después de un cartesiano fue rechazado en
      // ejecución en 14ms ("el estado actual no coincide con el inicio del
      // plan") -- probablemente el estado recién asentaba.
      rclcpp::sleep_for(300ms);
      return ok;
    };

  // Recuperación de emergencia: manda la trayectoria a 'home' directo al
  // controlador, SIN pasar por la verificación de colisión de MoveIt.
  // Existe porque avoid_collisions=false en pick/place/retreat (necesario
  // para poder acercarse a la mesa/tachos) a veces deja al brazo en un
  // roce real que ni pick_place_node ni MoveIt detectan -- y desde ahí,
  // MoveIt rechaza CUALQUIER plan nuevo (hasta ir a home) porque considera
  // el estado de partida inválido. Es exactamente lo mismo que se hacía a
  // mano por terminal para destrabarlo durante las pruebas.
  const std::vector<std::string> kArmJoints = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  };
  const std::vector<double> kHomePositions = {0.0, -1.5707, 0.0, 0.0, 0.0, 0.0};

  auto force_home = [&]() -> bool {
      if (!trajectory_client->wait_for_action_server(5s)) {
        RCLCPP_ERROR(logger, "  [recuperación] no se pudo contactar al controlador");
        return false;
      }
      FollowJointTrajectory::Goal goal;
      goal.trajectory.joint_names = kArmJoints;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = kHomePositions;
      point.time_from_start = rclcpp::Duration::from_seconds(4.0);
      goal.trajectory.points.push_back(point);

      auto goal_future = trajectory_client->async_send_goal(goal);
      if (goal_future.wait_for(6s) != std::future_status::ready || !goal_future.get()) {
        RCLCPP_ERROR(logger, "  [recuperación] meta rechazada o sin respuesta");
        return false;
      }
      auto result_future = trajectory_client->async_get_result(goal_future.get());
      if (result_future.wait_for(8s) != std::future_status::ready) {
        RCLCPP_ERROR(logger, "  [recuperación] sin resultado a tiempo");
        return false;
      }
      bool ok = (result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED);
      RCLCPP_WARN(logger, "  [recuperación] %s", ok ? "brazo desatascado, listo para seguir" :
        "no se pudo desatascar");
      // Esta trayectoria se mandó directo al controlador, sin pasar por
      // MoveIt -- su "estado actual" tarda un instante en actualizarse con
      // la nueva posición. Sin esta pausa, el siguiente plan()+execute()
      // puede fallar al EJECUTAR (no al planear) por un desfase mínimo
      // entre lo que MoveIt cree que es el estado actual y el real.
      rclcpp::sleep_for(1500ms);
      return ok;
    };

  // Vuelve a 'home' -- una configuración conocida y ya probada -- entre
  // objetos (haya salido bien o mal el anterior). Sin esto, tras un fallo
  // el brazo se queda en un punto intermedio cualquiera y la transición
  // directa de ahí al siguiente objetivo (una ruta que nunca planeamos a
  // propósito) puede terminar en un movimiento raro/largo. Así toda
  // transición queda acotada a home<->objeto, que sí probamos.
  auto go_home = [&]() {
      move_group.setNamedTarget("home");
      moveit::planning_interface::MoveGroupInterface::Plan home_plan;
      if (move_group.plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(home_plan);
        // Mismo motivo que en force_home(): el siguiente plan()+execute()
        // (típicamente "approach-place" justo después) a veces fallaba al
        // EJECUTAR, no al planear -- un desfase mínimo entre que MoveIt
        // reporta "listo" y que su estado actual realmente se actualiza.
        rclcpp::sleep_for(1000ms);
      } else {
        RCLCPP_WARN(logger,
          "No se pudo planear de vuelta a 'home' -- probablemente un roce real "
          "que MoveIt no vio venir. Intentando recuperación de emergencia...");
        force_home();
      }
    };

  // Si algo falla DESPUÉS de haber cerrado el gripper (agarrado un objeto),
  // hay que soltarlo antes de ir a 'home' -- si no, el objeto viaja
  // "fantasma" en la pinza y estorba al siguiente ciclo (ni el código ni
  // el gripper tienen forma de saber que sigue ahí atrapado).
  bool holding = false;
  auto recover = [&]() {
      if (holding) {
        set_gripper(kGripperOpen);
        holding = false;
      }
      go_home();
    };

  RCLCPP_INFO(logger, "=== EcoSort pick-and-place: iniciando (%zu objetos) ===", items.size());
  // Más tiempo que el open/close normal del ciclo (1.5s): el gripper puede
  // arrancar CERRADO de fábrica (initial_value en 2f_85.ros2_control.xacro),
  // así que este primer "abrir" a veces tiene que recorrer todo el rango
  // -- más lejos que un open/close normal -- y con la ganancia blanda del
  // controlador (0.1) puede no alcanzar a llegar en 1.5s.
  set_gripper(kGripperOpen, /*settle_ms=*/ 4000);

  // color_detector (ecosort_perception) normalmente ya lleva rato corriendo
  // en la Terminal 1 para cuando este nodo arranca, pero por si acaso: un
  // margen corto para que lleguen las primeras detecciones antes de decidir
  // "sin detección, usar respaldo" en el primer objeto.
  rclcpp::sleep_for(3000ms);
  for (const auto & category : {"papel", "plastico", "vidrio"}) {
    if (detections.count(category)) {
      RCLCPP_INFO(logger, "Detección de cámara recibida para: %s", category);
    } else {
      RCLCPP_WARN(logger, "Todavía sin detección de cámara para: %s (se omitirá si no llega)",
        category);
    }
  }

  for (const auto & item : items) {
    // La posición de agarre viene SIEMPRE de la cámara: si esta categoría
    // nunca fue detectada, se asume que ese residuo no está sobre la mesa
    // y se omite -- ya no se usa ninguna coordenada fija de respaldo (eso
    // hacía que el brazo fuera a buscar en el punto de spawn original
    // aunque el objeto ya no estuviera ahí).
    auto det = detections.find(item.category);
    if (det == detections.end()) {
      RCLCPP_WARN(logger,
        "--- Objeto: %s no está sobre la mesa (sin detección de cámara) -- se omite ---",
        item.category.c_str());
      continue;
    }
    double pick_x = det->second.x;
    double pick_y = det->second.y;
    RCLCPP_INFO(logger, "--- Objeto: %s (detectado por cámara: %.3f, %.3f) ---",
      item.category.c_str(), pick_x, pick_y);

    if (!move_to_joint_space(
        "approach-pick", pick_x, pick_y, item.pick_z + kApproachHeight))
    {
      recover();
      continue;
    }
    if (!move_to_cartesian(
        "pick", pick_x, pick_y, item.pick_z + kGraspSafetyMarginZ,
        /*avoid_collisions=*/ false))
    {
      recover();
      continue;
    }

    set_gripper(kGripperClosed, /*settle_ms=*/ 4000);
    RCLCPP_INFO(logger, "  [grip] posición real del dedo tras cerrar: %.3f (comandado: %.3f)",
      gripper_joint_pos, kGripperClosed);
    bool grasped = gripper_joint_pos < kGripFailThreshold;
    if (!grasped) {
      // Cerró casi del todo sin resistencia -- no había nada entre los
      // dedos (se resbaló al bajar, o el objeto no estaba exactamente
      // donde lo puso la cámara). El brazo NO se movió todavía desde el
      // punto de agarre, así que basta con reabrir y volver a cerrar ahí
      // mismo antes de rendirse.
      RCLCPP_WARN(logger,
        "  [grip] no se sintió resistencia al cerrar -- probable agarre fallido, "
        "reintentando en el mismo punto...");
      set_gripper(kGripperOpen, /*settle_ms=*/ 800);
      set_gripper(kGripperClosed, /*settle_ms=*/ 4000);
      RCLCPP_INFO(logger, "  [grip] posición real del dedo tras reintento: %.3f (comandado: %.3f)",
        gripper_joint_pos, kGripperClosed);
      grasped = gripper_joint_pos < kGripFailThreshold;
    }
    if (!grasped) {
      RCLCPP_ERROR(logger,
        "  [grip] no se pudo agarrar %s tras reintentar -- se omite este objeto "
        "(no se lleva nada al tacho)", item.category.c_str());
      set_gripper(kGripperOpen, /*settle_ms=*/ 500);
      go_home();
      continue;
    }
    holding = true;

    if (!move_to_cartesian(
        "retreat-pick", pick_x, pick_y, item.pick_z + kApproachHeight,
        /*avoid_collisions=*/ false))
    {
      recover();
      continue;
    }

    // Pasar siempre por 'home' antes de ir hacia el tacho: sin esto,
    // "approach-place" arranca desde donde haya quedado el brazo tras
    // agarrar -- un punto de partida DISTINTO según qué objeto fue (cada
    // uno se agarra en un lugar diferente) -- y RRTConnect es mucho menos
    // confiable con puntos de partida variables que con uno fijo y ya
    // probado. Cuesta unos segundos extra de viaje, a cambio de mucha
    // más consistencia. El objeto se queda agarrado durante este tramo
    // (no se suelta, solo se usa 'home' como punto de paso).
    go_home();

    if (!move_to_joint_space(
        "approach-place", item.place_x, item.place_y, item.place_z + kApproachHeight))
    {
      recover();
      continue;
    }
    if (!move_to_cartesian(
        "place", item.place_x, item.place_y, item.place_z + kGraspSafetyMarginZ,
        /*avoid_collisions=*/ false))
    {
      recover();
      continue;
    }

    set_gripper(kGripperOpen);
    holding = false;

    if (!move_to_cartesian(
        "retreat-place", item.place_x, item.place_y, item.place_z + kApproachHeight,
        /*avoid_collisions=*/ false))
    {
      recover();
      continue;
    }

    RCLCPP_INFO(logger, "--- %s: listo ---", item.category.c_str());
    go_home();
  }

  RCLCPP_INFO(logger, "=== EcoSort pick-and-place: terminado ===");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
