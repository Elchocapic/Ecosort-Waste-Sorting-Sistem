#!/usr/bin/env python3
"""EcoSort - color_detector

Detecta los 3 objetos (papel=azul, plástico=amarillo, vidrio=verde) en la
imagen de la cámara cenital fija y publica su posición 3D estimada (en el
frame "world" del SDF, las mismas coordenadas que ya usa pick_place_node
para sus posiciones fijas).

Método: como la cámara es fija y mira derecho hacia abajo, y la altura de
los objetos sobre la mesa es conocida y constante, no hace falta un sensor
de profundidad real -- basta con la fórmula de deproyección de un modelo
pinhole con la profundidad (altura cámara - altura objeto) conocida de
antemano.

Los intrínsecos de la cámara se calculan directo del FOV/resolución
configurados en ecosort_world.sdf, en vez de leerlos del topic
camera_info (que en este entorno no llega de forma confiable).

La fórmula de deproyección y los umbrales de color fueron verificados
empíricamente contra las posiciones reales conocidas del SDF (ver sesión
de calibración): error < 5mm en los 3 casos.

IMPORTANTE: los tachos de destino son del MISMO color que cada objeto,
pero mucho más grandes en la imagen (son círculos, los objetos son
cuadrados chicos) -- se descartan por área. Además, el umbral de
saturación/brillo (S,V) debe ser alto: el café de la mesa cae dentro del
rango de tono "amarillo" y sin ese filtro se detecta la mesa entera como
si fuera el objeto de plástico.
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

# --- Geometría de la cámara (ecosort_world.sdf: modelo "overhead_camera") ---
IMG_WIDTH = 640
IMG_HEIGHT = 480
HORIZONTAL_FOV = 1.047  # rad
CAMERA_X = 0.45
CAMERA_Y = 0.0
CAMERA_Z = 1.1
# Altura del centro de los objetos sobre la mesa (misma para los 3, ver
# ecosort_world.sdf: objeto_papel/plastico/vidrio, todos z=0.42)
OBJECT_Z = 0.42

# --- Rangos de color HSV (OpenCV: H en 0-179) ---
# S y V altos a propósito: separan el color "puro" de los objetos/tachos
# de superficies apagadas como la mesa (que si no, cae dentro del rango
# de tono "amarillo").
COLOR_RANGES = {
    'papel': ((100, 150, 150), (140, 255, 255)),      # azul
    'plastico': ((15, 150, 150), (45, 255, 255)),      # amarillo
    'vidrio': ((40, 150, 150), (80, 255, 255)),        # verde
}

# Área del contorno en píxeles esperada para el objeto (no el tacho, que
# es mucho más grande al ser un círculo de ~0.08-0.09m de radio).
MIN_OBJECT_AREA = 200
MAX_OBJECT_AREA = 3000


class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge()

        self.fx = (IMG_WIDTH / 2.0) / math.tan(HORIZONTAL_FOV / 2.0)
        self.fy = self.fx  # píxeles cuadrados, verificado con el FOV configurado
        self.cx = IMG_WIDTH / 2.0
        self.cy = IMG_HEIGHT / 2.0
        self.depth = CAMERA_Z - OBJECT_Z

        self.publishers_by_category = {
            category: self.create_publisher(PointStamped, f'/ecosort/detected/{category}', 10)
            for category in COLOR_RANGES
        }

        self.create_subscription(Image, '/ecosort/camera/image_raw', self.on_image, 10)
        self.get_logger().info('color_detector listo, esperando imágenes...')

    def on_image(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        stamp = msg.header.stamp

        for category, (lo, hi) in COLOR_RANGES.items():
            mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            candidates = [
                c for c in contours
                if MIN_OBJECT_AREA < cv2.contourArea(c) < MAX_OBJECT_AREA
            ]
            if not candidates:
                continue  # no se ve ahora mismo (tapado, ya lo agarraron, etc.)

            best = max(candidates, key=cv2.contourArea)
            m = cv2.moments(best)
            # Centroid of the largest matching blob, in pixel coordinates (u,v).
            u = m['m10'] / m['m00']
            v = m['m01'] / m['m00']

            # Pinhole deprojection: pixel (u,v) -> offset from the camera's
            # optical axis, in meters, at the known constant depth. This is
            # standard "z * (pixel - principal_point) / focal_length" per axis.
            x_cam = (u - self.cx) * self.depth / self.fx
            y_cam = (v - self.cy) * self.depth / self.fy
            # The camera looks straight down (-Z world) with the image's
            # +X axis (u, columns) aligned with world -Y, and the image's
            # +Y axis (v, rows) aligned with world -X (rotated top-down
            # view, verified empirically against known SDF spawn points --
            # see the module docstring). Hence the axis swap below instead
            # of a direct x_cam->world_x, y_cam->world_y mapping.
            world_x = CAMERA_X - y_cam
            world_y = CAMERA_Y - x_cam

            point = PointStamped()
            point.header.stamp = stamp
            point.header.frame_id = 'world'  # frame del SDF (Gazebo), no el de MoveIt
            point.point.x = world_x
            point.point.y = world_y
            point.point.z = OBJECT_Z
            self.publishers_by_category[category].publish(point)


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
