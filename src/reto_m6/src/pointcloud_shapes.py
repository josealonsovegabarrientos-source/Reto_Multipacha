#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import math

class SimpleShapeGenerator:
    def __init__(self):
        rospy.init_node('pointcloud_shapes', anonymous=True)
        
        # Publicador - NUEVO TOPIC
        self.pub = rospy.Publisher('/formas_geometricas', PointCloud2, queue_size=10)
        
        # Timer para publicar cada 3 segundos
        self.timer = rospy.Timer(rospy.Duration(3), self.publish_shape)
        
        # Contador para alternar formas
        self.count = 0
        
        rospy.loginfo("Generador iniciado. Publicando en: /formas_geometricas")

    def publish_shape(self, event):
        """Publica diferentes formas cada vez"""
        
        if self.count % 3 == 0:
            # Círculo
            points = self.make_circle()
            rospy.loginfo("Publicando CÍRCULO")
            
        elif self.count % 3 == 1:
            # Rectángulo
            points = self.make_rectangle()
            rospy.loginfo("Publicando RECTÁNGULO")
            
        else:
            # Puntos aleatorios
            points = self.make_random()
            rospy.loginfo("Publicando PUNTOS ALEATORIOS")
        
        # Crear y publicar PointCloud2
        cloud = self.points_to_pointcloud2(points)
        self.pub.publish(cloud)
        
        self.count += 1

    def make_circle(self):
        """Crear puntos en forma de círculo"""
        points = []
        center_x, center_y = 0.0, 0.0
        radius = 1.5
        
        # 40 puntos en el círculo
        for i in range(40):
            angle = 2 * math.pi * i / 40
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            z = 0.0
            points.append([x, y, z])
        
        return points

    def make_rectangle(self):
        """Crear puntos en forma de rectángulo"""
        points = []
        
        # Rectángulo 3x2
        width, height = 3.0, 2.0
        
        # Lado de abajo
        for i in range(15):
            x = -width/2 + (width * i / 14)
            y = -height/2
            z = 0.0
            points.append([x, y, z])
        
        # Lado de arriba
        for i in range(15):
            x = -width/2 + (width * i / 14)
            y = height/2
            z = 0.0
            points.append([x, y, z])
        
        # Lado izquierdo (sin esquinas)
        for i in range(1, 9):
            x = -width/2
            y = -height/2 + (height * i / 9)
            z = 0.0
            points.append([x, y, z])
        
        # Lado derecho (sin esquinas)
        for i in range(1, 9):
            x = width/2
            y = -height/2 + (height * i / 9)
            z = 0.0
            points.append([x, y, z])
        
        return points

    def make_random(self):
        """Crear puntos aleatorios"""
        points = []
        
        for i in range(50):
            x = np.random.uniform(-2, 2)
            y = np.random.uniform(-2, 2)
            z = 0.0
            points.append([x, y, z])
        
        return points

    def points_to_pointcloud2(self, points):
        """Convertir lista de puntos a PointCloud2"""
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base_link"
        
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        
        return pc2.create_cloud(header, fields, points)

if __name__ == '__main__':
    try:
        generator = SimpleShapeGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
