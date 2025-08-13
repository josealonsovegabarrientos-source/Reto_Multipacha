#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import math

class FilledShapeGenerator:
    def __init__(self):
        rospy.init_node('pointcloud_filled_gen', anonymous=True)

        # ===== Parámetros =====
        self.frame_id     = rospy.get_param('~frame_id', 'base_link')
        self.mode         = rospy.get_param('~mode', 'cycle')   # 'cycle'|'circle'|'rectangle'|'annulus'|'random'
        self.filled       = rospy.get_param('~filled', True)    # True=lleno, False=borde
        self.res          = float(rospy.get_param('~res', 0.05))
        self.radius       = float(rospy.get_param('~radius', 1.5))
        self.inner_radius = float(rospy.get_param('~inner_radius', 0.8))
        self.width        = float(rospy.get_param('~width', 3.0))
        self.height       = float(rospy.get_param('~height', 2.0))
        self.n_outline    = int(rospy.get_param('~n_outline', 120))
        self.n_random     = int(rospy.get_param('~n_random', 300))
        self.z            = float(rospy.get_param('~z', 0.0))
        period            = float(rospy.get_param('~period', 2.0))

        # >>> Nuevo: tópico paramétrico (por defecto, cambiado)
        self.topic        = rospy.get_param('~topic', '/formas_geometricas_llenas')

        # Publicador al nuevo tópico
        self.pub = rospy.Publisher(self.topic, PointCloud2, queue_size=1)

        self.count = 0
        self.timer = rospy.Timer(rospy.Duration(period), self.publish_shape)

        rospy.loginfo("Nodo pointcloud_filled_gen listo | topic=%s | filled=%s | res=%.3f",
                      self.topic, self.filled, self.res)

    def publish_shape(self, _):
        shape = self._select_shape()

        if shape == 'circle':
            pts = self.make_circle_filled(self.radius, self.res, self.z) \
                  if self.filled else self.make_circle_outline(self.radius, self.n_outline, self.z)
            rospy.loginfo("Publicando CIRCULO (%s) -> %s", 'lleno' if self.filled else 'borde', self.topic)

        elif shape == 'rectangle':
            pts = self.make_rectangle_filled(self.width, self.height, self.res, self.z) \
                  if self.filled else self.make_rectangle_outline(self.width, self.height, self.res, self.z)
            rospy.loginfo("Publicando RECTANGULO (%s) -> %s", 'lleno' if self.filled else 'borde', self.topic)

        elif shape == 'annulus':
            pts = self.make_annulus_filled(self.inner_radius, self.radius, self.res, self.z)
            rospy.loginfo("Publicando ANILLO LLENO (ri=%.2f, ro=%.2f) -> %s", self.inner_radius, self.radius, self.topic)

        else:
            pts = self.make_random(self.n_random, 2.0, self.z)
            rospy.loginfo("Publicando PUNTOS ALEATORIOS -> %s", self.topic)

        self.pub.publish(self.points_to_pointcloud2(pts))
        self.count += 1

    def _select_shape(self):
        if self.mode == 'cycle':
            return ['circle', 'rectangle', 'annulus', 'random'][self.count % 4]
        return self.mode

    # ---------- Bordes ----------
    def make_circle_outline(self, r, n, z=0.0):
        i = np.arange(n, dtype=float)
        a = 2.0*np.pi*i/n
        x = r*np.cos(a); y = r*np.sin(a)
        return np.column_stack((x, y, np.full_like(x, z))).tolist()

    def make_rectangle_outline(self, w, h, res=0.05, z=0.0):
        xs = np.arange(-w/2.0, w/2.0 + res, res)
        ys = np.arange(-h/2.0, h/2.0 + res, res)
        top = np.column_stack((xs, np.full_like(xs,  h/2.0), np.full_like(xs, z)))
        bottom = np.column_stack((xs, np.full_like(xs, -h/2.0), np.full_like(xs, z)))
        left = np.column_stack((np.full_like(ys, -w/2.0), ys, np.full_like(ys, z)))
        right = np.column_stack((np.full_like(ys,  w/2.0), ys, np.full_like(ys, z)))
        return np.vstack((top, bottom, left, right)).tolist()

    # ---------- Figuras llenas ----------
    def make_circle_filled(self, r, res=0.05, z=0.0):
        xs = np.arange(-r, r + res, res)
        ys = np.arange(-r, r + res, res)
        X, Y = np.meshgrid(xs, ys)
        mask = X**2 + Y**2 <= r**2
        return np.column_stack((X[mask], Y[mask], np.full(np.count_nonzero(mask), z))).tolist()

    def make_annulus_filled(self, ri, ro, res=0.05, z=0.0):
        xs = np.arange(-ro, ro + res, res)
        ys = np.arange(-ro, ro + res, res)
        X, Y = np.meshgrid(xs, ys)
        R2 = X**2 + Y**2
        mask = (R2 <= ro**2) & (R2 >= ri**2)
        return np.column_stack((X[mask], Y[mask], np.full(np.count_nonzero(mask), z))).tolist()

    def make_rectangle_filled(self, w, h, res=0.05, z=0.0):
        xs = np.arange(-w/2.0, w/2.0 + res, res)
        ys = np.arange(-h/2.0, h/2.0 + res, res)
        X, Y = np.meshgrid(xs, ys)
        return np.column_stack((X.ravel(), Y.ravel(), np.full(X.size, z))).tolist()

    def make_random(self, n, extent=2.0, z=0.0):
        x = np.random.uniform(-extent, extent, n)
        y = np.random.uniform(-extent, extent, n)
        return np.column_stack((x, y, np.full(n, z))).tolist()

    # ---------- Utilidad ----------
    def points_to_pointcloud2(self, points):
        header = Header(stamp=rospy.Time.now(), frame_id=self.frame_id)
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        return pc2.create_cloud(header, fields, points)

if __name__ == '__main__':
    try:
        FilledShapeGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
