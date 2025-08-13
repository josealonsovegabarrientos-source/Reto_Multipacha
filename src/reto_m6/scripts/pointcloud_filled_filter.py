#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class GeometricFilledFilter:
    """
    Filtro geométrico para nubes de puntos (PointCloud2) con modos:
      - circle   : disco de radio 'radius' centrado en (cx, cy)
      - rectangle: caja AABB de tamaño (width x height) centrada en (cx, cy)
      - annulus  : corona circular entre 'inner_radius' y 'outer_radius'

    Suscribe:  ~topic_in  (default: /formas_geometricas_llenas)
    Publica:   ~topic_out (default: /pointcloud_filtrada)
               ~topic_out_rejected (default: /filtro_info)
    """

    def __init__(self):
        rospy.init_node('pointcloud_filled_filter', anonymous=True)

        # --- tópicos (parametrizables) ---
        self.topic_in  = rospy.get_param('~topic_in',  '/formas_geometricas_llenas')
        self.topic_out = rospy.get_param('~topic_out', '/pointcloud_filtrada')
        self.topic_out_rej = rospy.get_param('~topic_out_rejected', '/filtro_info')

        # Publishers
        self.pub_ok  = rospy.Publisher(self.topic_out,      PointCloud2, queue_size=1)
        self.pub_rej = rospy.Publisher(self.topic_out_rej,  PointCloud2, queue_size=1)

        # Subscriber
        self.sub = rospy.Subscriber(self.topic_in, PointCloud2, self.callback, queue_size=1)

        rospy.loginfo("Filtro listo. IN: %s | OUT: %s | REJ: %s",
                      self.topic_in, self.topic_out, self.topic_out_rej)

    def _read_params(self):
        # --- modo y parámetros geométricos ---
        self.mode = rospy.get_param('~mode', 'circle')  # circle | rectangle | annulus
        self.cx   = float(rospy.get_param('~centro_x', 0.0))
        self.cy   = float(rospy.get_param('~centro_y', 0.0))

        # CÍRCULO (pensando en tus tamaños: circle R=1.5 del generador ⇒ default radio=1.0 para “recortar”)
        self.radius = float(rospy.get_param('~radius', 1.0))

        # RECTÁNGULO (tu generador usa 3.0 x 2.0 ⇒ por defecto recortamos una ventana más pequeña)
        self.width  = float(rospy.get_param('~width',  1.5))
        self.height = float(rospy.get_param('~height', 1.0))

        # ANILLO (tu generador usa inner=0.8, outer=1.5)
        self.inner_radius = float(rospy.get_param('~inner_radius', 0.8))
        self.outer_radius = float(rospy.get_param('~outer_radius', 1.5))

        # Filtro opcional por Z
        self.enable_z = bool(rospy.get_param('~enable_z', False))
        self.min_z    = float(rospy.get_param('~min_z', -1e9))
        self.max_z    = float(rospy.get_param('~max_z',  1e9))

    def callback(self, cloud_msg: PointCloud2):
        # Lee parámetros en cada mensaje (permite ajustar con rosparam sin reiniciar)
        self._read_params()

        # Extraer puntos (x,y,z) como array float32
        pts_list = list(pc2.read_points(cloud_msg, field_names=("x","y","z"), skip_nans=True))
        if not pts_list:
            rospy.logwarn_throttle(2.0, "PointCloud vacía en %s", self.topic_in)
            return

        pts = np.asarray(pts_list, dtype=np.float32)  # shape (N,3)
        x, y, z = pts[:,0], pts[:,1], pts[:,2]

        # --- Máscaras geométricas ---
        if self.mode == 'rectangle':
            mask = (np.abs(x - self.cx) <= self.width/2.0) & (np.abs(y - self.cy) <= self.height/2.0)

        elif self.mode == 'annulus':
            r2 = (x - self.cx)**2 + (y - self.cy)**2
            mask = (r2 >= self.inner_radius**2) & (r2 <= self.outer_radius**2)

        else:  # 'circle' por defecto
            mask = (x - self.cx)**2 + (y - self.cy)**2 <= self.radius**2

        # Filtro Z opcional
        if self.enable_z:
            mask = mask & (z >= self.min_z) & (z <= self.max_z)

        kept = pts[mask]
        rej  = pts[~mask]

        # Publicar resultados
        header = Header(stamp=rospy.Time.now(), frame_id=cloud_msg.header.frame_id)
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        if kept.size:
            self.pub_ok.publish(pc2.create_cloud(header, fields, kept.tolist()))
        if rej.size:
            self.pub_rej.publish(pc2.create_cloud(header, fields, rej.tolist()))

        rospy.loginfo_throttle(1.0, "Filtro [%s] centro=(%.2f,%.2f) | kept=%d  rej=%d",
                               self.mode, self.cx, self.cy, kept.shape[0], rej.shape[0])

if __name__ == '__main__':
    try:
        GeometricFilledFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
