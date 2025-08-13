#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import math

class FiltroCircular:
    def __init__(self):
        rospy.init_node('filtro_circular', anonymous=True)
        
        # PARÁMETROS DEL CÍRCULO (puedes cambiarlos)
        self.centro_x = 0.0      # Centro X del círculo
        self.centro_y = 0.0      # Centro Y del círculo  
        self.radio = 2.0         # Radio del área circular
        
        # Suscriptor - PointCloud original
        self.sub = rospy.Subscriber('/formas_geometricas', PointCloud2, self.filtrar_puntos)
        
        # Publicador - PointCloud filtrada
        self.pub = rospy.Publisher('/pointcloud_filtrada', PointCloud2, queue_size=10)
        
        # Publicador - Información del filtrado
        self.info_pub = rospy.Publisher('/filtro_info', PointCloud2, queue_size=10)
        
        rospy.loginfo("=== FILTRO CIRCULAR INICIADO ===")
        rospy.loginfo(f"Centro: ({self.centro_x}, {self.centro_y})")
        rospy.loginfo(f"Radio: {self.radio} metros")
        rospy.loginfo("Suscrito a: /formas_geometricas")
        rospy.loginfo("Publicando filtrado en: /pointcloud_filtrada")

    def filtrar_puntos(self, cloud_msg):
        """Función principal que filtra los puntos"""
        
        # PASO 1: Extraer todos los puntos
        puntos_originales = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            puntos_originales.append([x, y, z])
        
        if len(puntos_originales) == 0:
            rospy.logwarn("No hay puntos en la PointCloud")
            return
        
        # PASO 2: Filtrar puntos dentro del círculo
        puntos_filtrados = []
        puntos_descartados = []
        
        for punto in puntos_originales:
            x, y, z = punto
            
            # Calcular distancia del punto al centro del círculo
            distancia = self.calcular_distancia(x, y, self.centro_x, self.centro_y)
            
            # Si está dentro del radio, lo guardamos
            if distancia <= self.radio:
                puntos_filtrados.append([x, y, z])
            else:
                puntos_descartados.append([x, y, z])
        
        # PASO 3: Crear PointCloud2 con puntos filtrados
        if len(puntos_filtrados) > 0:
            cloud_filtrada = self.crear_pointcloud2(puntos_filtrados, cloud_msg.header.frame_id)
            self.pub.publish(cloud_filtrada)
        
        # PASO 4: Mostrar estadísticas
        total = len(puntos_originales)
        dentro = len(puntos_filtrados)
        fuera = len(puntos_descartados)
        
        rospy.loginfo(f"FILTRADO: {dentro}/{total} puntos dentro del círculo ({fuera} descartados)")
        
        # OPCIONAL: Publicar también los puntos descartados para visualización
        if len(puntos_descartados) > 0:
            cloud_descartada = self.crear_pointcloud2(puntos_descartados, cloud_msg.header.frame_id)
            self.info_pub.publish(cloud_descartada)

    def calcular_distancia(self, x1, y1, x2, y2):
        """Calcular distancia euclidiana entre dos puntos"""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def crear_pointcloud2(self, puntos, frame_id):
        """Convertir lista de puntos a PointCloud2"""
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        
        return pc2.create_cloud(header, fields, puntos)

    def cambiar_parametros(self, nuevo_centro_x, nuevo_centro_y, nuevo_radio):
        """Función para cambiar parámetros del filtro dinámicamente"""
        self.centro_x = nuevo_centro_x
        self.centro_y = nuevo_centro_y
        self.radio = nuevo_radio
        
        rospy.loginfo(f"PARÁMETROS ACTUALIZADOS:")
        rospy.loginfo(f"Nuevo centro: ({self.centro_x}, {self.centro_y})")
        rospy.loginfo(f"Nuevo radio: {self.radio}")

if __name__ == '__main__':
    try:
        filtro = FiltroCircular()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


