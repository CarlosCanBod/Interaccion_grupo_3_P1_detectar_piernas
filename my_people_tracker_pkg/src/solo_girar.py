#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Twist
import threading

class PersonTurner:
    def __init__(self):
        rospy.init_node('person_turner_node', anonymous=True)

        # ---- Parámetros Simplificados (Solo para Girar) ----
        # Ganancia proporcional para la velocidad angular. Un valor más alto hará que gire más rápido.
        self.k_angular = 1.2 
        # Límite de seguridad para la velocidad de giro (radianes/segundo)
        self.max_angular_speed = 0.8
        # Umbral de fiabilidad para confiar en una detección
        self.reliability_threshold = 0.6
        # Periodo de gracia para no perder el objetivo si el detector "parpadea"
        self.lost_target_timeout = rospy.Duration(1.5)
        # Frecuencia del bucle de control (15 Hz para una respuesta más rápida)
        self.control_rate = 15.0

        # ---- Estado del Seguidor ----
        self.target_person = None
        self.last_seen_time = None
        self.lock = threading.Lock()

        # ---- Publicadores y Suscriptores ----
        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        rospy.Subscriber('people_tracker_measurements', PositionMeasurementArray, self.people_callback)

        rospy.loginfo("Nodo 'Solo Girar' inicializado. Buscando a quién encarar...")

    def people_callback(self, msg):
        """
        Este callback es muy ligero. Solo encuentra el mejor candidato
        y lo guarda en una variable compartida.
        """
        closest_person = None
        min_dist = float('inf')
        
        for person in msg.people:
            if person.reliability > self.reliability_threshold:
                dist = math.sqrt(person.pos.x**2 + person.pos.y**2)
                rospy.loginfo("Persona es: {} y la distancia en x {:2f} y en y {:.2f} y la reliability es {:.2f}".format(person.name, person.pos.x, person.pos.y, person.reliability))
                if dist < min_dist:
                    min_dist = dist
                    closest_person = person

        
        with self.lock:
            # Si encontramos a alguien, lo convertimos en nuestro objetivo.
            if closest_person:
                self.target_person = closest_person
                self.last_seen_time = rospy.Time.now()

    def run(self):
        """
        Bucle de control rápido que se ejecuta 15 veces por segundo.
        Su única misión es calcular el giro necesario y enviarlo.
        """
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            twist_msg = Twist() # Por defecto, el comando es PARAR
            twist_msg.linear.x = 0.0 # NUNCA nos moveremos hacia adelante o atrás

            with self.lock:
                # Comprobamos si tenemos un objetivo y si es reciente
                if self.target_person and self.last_seen_time:
                    time_since_last_seen = rospy.Time.now() - self.last_seen_time

                    if time_since_last_seen < self.lost_target_timeout:
                        # --- Lógica de Giro ---
                        # Posición del objetivo
                        x = self.target_person.pos.x
                        y = self.target_person.pos.y

                        rospy.loginfo("Persona objetivo a girar es: {} y la distancia en x {:2f} y en y {:.2f} y la reliability es {:.2f}".format(self.target_person.name, x, y, self.target_person.reliability))
                        # Calculamos el ángulo para encarar al objetivo.
                        # atan2 nos da el ángulo en radianes en el plano XY.
                        angle_to_target = math.atan2(y, x)

                        # El error es el propio ángulo (queremos que sea 0).
                        angular_error = angle_to_target
                        
                        # Calculamos la velocidad de giro proporcional al error.
                        angular_speed = self.k_angular * angular_error
                        
                        # Aplicamos los límites de seguridad.
                        angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
                        
                        twist_msg.angular.z = angular_speed
                    else:
                        # Si ha pasado el timeout, reseteamos el objetivo.
                        self.target_person = None

            # Publicamos el comando (sea de giro o de parada).
            #self.cmd_pub.publish(twist_msg)
            
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("El tiempo de ROS retrocedió, reiniciando espera.")


if __name__ == '__main__':
    try:
        turner = PersonTurner()
        turner.run()
    except rospy.ROSInterruptException:
        pass
