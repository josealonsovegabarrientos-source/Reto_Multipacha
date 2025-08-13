#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String

class DetectorFormas:
    def __init__(self):
        rospy.init_node('detector_formas', anonymous=True)
        
        # Suscriptor - escucha el topic de formas
        self.sub = rospy.Subscriber('/formas_geometricas', PointCloud2, self.analizar_forma)
        
        # Publicador - publica el resultado
        self.pub = rospy.Publisher('/resultado_deteccion', String, queue_size=10)
        
        rospy.loginfo("Detector iniciado. Escuchando: /formas_geometricas")
        rospy.loginfo("Publicando resultados en: /resultado_deteccion")

    def analizar_forma(self, cloud_msg):
        """Función principal que analiza cada PointCloud que llega"""
        
        # PASO 1: Sacar los puntos del mensaje
        puntos = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            puntos.append([x, y])  # Solo X e Y
        
        puntos = np.array(puntos)
        
        if len(puntos) < 10:
            rospy.logwarn("Muy pocos puntos")
            return
        
        # PASO 2: ¿Es un círculo?
        if self.es_circulo(puntos):
            resultado = "CIRCULO"
        
        # PASO 3: ¿Es un rectángulo?
        elif self.es_rectangulo(puntos):
            resultado = "RECTANGULO"
        
        # PASO 4: No es ninguna forma conocida
        else:
            resultado = "DESCONOCIDO"
        
        # PASO 5: Publicar resultado
        msg = String()
        msg.data = resultado
        self.pub.publish(msg)
        
        rospy.loginfo(f"DETECTADO: {resultado}")

    def es_circulo(self, puntos):
        """Verificar si los puntos forman un círculo"""
        
        # Encontrar el centro (promedio de todos los puntos)
        centro_x = np.mean(puntos[:, 0])
        centro_y = np.mean(puntos[:, 1])
        
        # Calcular distancia de cada punto al centro
        distancias = []
        for punto in puntos:
            x, y = punto
            distancia = np.sqrt((x - centro_x)**2 + (y - centro_y)**2)
            distancias.append(distancia)
        
        # Si todas las distancias son parecidas, es un círculo
        distancia_promedio = np.mean(distancias)
        diferencias = []
        
        for dist in distancias:
            diferencia = abs(dist - distancia_promedio)
            diferencias.append(diferencia)
        
        diferencia_promedio = np.mean(diferencias)
        
        # Si la diferencia promedio es pequeña, es círculo
        if diferencia_promedio < 0.2:
            rospy.loginfo(f"Círculo detectado - Radio: {distancia_promedio:.2f}")
            return True
        
        return False

    def es_rectangulo(self, puntos):
        """Verificar si los puntos forman un rectángulo"""
        
        # Encontrar los límites
        min_x = np.min(puntos[:, 0])
        max_x = np.max(puntos[:, 0])
        min_y = np.min(puntos[:, 1])
        max_y = np.max(puntos[:, 1])
        
        # Contar cuántos puntos están cerca de los bordes
        puntos_en_bordes = 0
        tolerancia = 0.2
        
        for punto in puntos:
            x, y = punto
            
            # ¿Está cerca del borde izquierdo o derecho?
            cerca_horizontal = (abs(x - min_x) < tolerancia) or (abs(x - max_x) < tolerancia)
            
            # ¿Está cerca del borde superior o inferior?
            cerca_vertical = (abs(y - min_y) < tolerancia) or (abs(y - max_y) < tolerancia)
            
            if cerca_horizontal or cerca_vertical:
                puntos_en_bordes += 1
        
        # Si más del 70% de puntos están en los bordes, es rectángulo
        porcentaje = puntos_en_bordes / len(puntos)
        
        if porcentaje > 0.7:
            rospy.loginfo(f"Rectángulo detectado - {puntos_en_bordes}/{len(puntos)} puntos en bordes")
            return True
        
        return False

if __name__ == '__main__':
    try:
        detector = DetectorFormas()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
