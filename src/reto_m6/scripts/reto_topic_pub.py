#!/usr/bin/env python3
#!/usr/bin/env python3

"""
Archivo: reto_topic_pub.py
Descripcion: Publicador que envia mensajes de tipo std_msgs/String 
             con el texto "M6 Reto" a una frecuencia de 10 Hz
Autor: Desarrollado para el reto M6
"""

import rospy
from std_msgs.msg import String

def publisher_node():
    # Inicializar el nodo ROS
    rospy.init_node('reto_topic_publisher', anonymous=True)
    
    # Crear el publicador
    # Publica en el topic 'reto_topic' mensajes de tipo String
    pub = rospy.Publisher('reto_topic', String, queue_size=10)
    
    # Configurar la frecuencia de publicacion (10 Hz)
    rate = rospy.Rate(10)  # 10 Hz
    
    # Mensaje a publicar
    mensaje = "M6 Reto"
    
    rospy.loginfo("Iniciando publicador en el topic 'reto_topic'")
    rospy.loginfo(f"Publicando mensaje: '{mensaje}' a 10 Hz")
    
    # Bucle principal de publicacion
    contador = 0
    while not rospy.is_shutdown():
        # Crear el mensaje
        msg = String()
        msg.data = f"{mensaje} - Contador: {contador}"
        
        # Publicar el mensaje
        pub.publish(msg)
        
        # Log cada 50 mensajes para no saturar la consola
        if contador % 50 == 0:
            rospy.loginfo(f"Mensaje publicado: '{msg.data}'")
        
        contador += 1
        
        # Esperar hasta la siguiente publicacion
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Publicador detenido por el usuario")
        pass
