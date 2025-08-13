#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class MostrarResultados:
    def __init__(self):
        rospy.init_node('mostrar_resultados', anonymous=True)
        
        # Suscriptor - escucha los resultados
        self.sub = rospy.Subscriber('/resultado_deteccion', String, self.mostrar_resultado)
        
        self.contador = 0
        
        rospy.loginfo("=== VISOR DE RESULTADOS ===")
        rospy.loginfo("Mostrando resultados de detección...")
        print("\n" + "="*50)

    def mostrar_resultado(self, msg):
        """Mostrar cada resultado de forma bonita"""
        
        self.contador += 1
        resultado = msg.data
        
        print(f"DETECCIÓN #{self.contador}")
        print(f"RESULTADO: {resultado}")
        
        if resultado == "CIRCULO":
            print("🔵 Se detectó una forma circular")
        elif resultado == "RECTANGULO":
            print("🔲 Se detectó una forma rectangular")
        else:
            print("❓ Forma no identificada")
        
        print("="*50)

if __name__ == '__main__':
    try:
        viewer = MostrarResultados()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
