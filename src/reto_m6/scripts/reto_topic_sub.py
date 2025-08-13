#!/usr/bin/env python3
#!/usr/bin/env python3

"""
Archivo: reto_topic_sub.py
Descripcion: Suscriptor que recibe mensajes del topic 'reto_topic'
             y los muestra en la terminal
Autor: Desarrollado para el reto M6
"""

import rospy
from std_msgs.msg import String

def callback_function(msg):
    """
    Funcion callback que se ejecuta cada vez que llega un mensaje
    Args:
        msg (std_msgs.msg.String): Mensaje recibido del topic
    """
    # Mostrar el mensaje recibido en la terminal
    rospy.loginfo(f"Mensaje recibido: '{msg.data}'")
    
    # También imprimir directamente en consola para mayor visibilidad
    print(f">>> RECIBIDO: {msg.data}")

def subscriber_node():
    """
    Nodo suscriptor principal
    """
    # Inicializar el nodo ROS
    rospy.init_node('reto_topic_subscriber', anonymous=True)
    
    # Crear el suscriptor
    # Se suscribe al topic 'reto_topic', espera mensajes String
    # y ejecuta callback_function cuando llega un mensaje
    rospy.Subscriber('reto_topic', String, callback_function)
    
    rospy.loginfo("Suscriptor iniciado. Escuchando mensajes en el topic 'reto_topic'...")
    rospy.loginfo("Presiona Ctrl+C para detener el suscriptor")
    
    # Mantener el nodo activo
    # rospy.spin() mantiene el programa corriendo y procesando callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Suscriptor detenido por el usuario")
        pass
