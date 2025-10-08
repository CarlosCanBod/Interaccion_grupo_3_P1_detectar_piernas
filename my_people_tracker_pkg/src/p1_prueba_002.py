#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Twist
import threading # Importante para la seguridad entre hilos

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower_node', anonymous=True)

        # ---- Parámetros ----
        self.desired_distance = 0.5
        self.k_linear = 0.5  # He aumentado un poco las ganancias para mayor reactividad
        self.k_angular = 1.0
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.7
        self.reliability_threshold = 0.7
        self.lost_target_timeout = rospy.Duration(2.0)
        
        # Frecuencia del bucle de control en Hz
        self.control_rate = 10.0

        # ---- Estado del Seguidor (Compartido entre hilos) ----
        self.target_id = None
        self.last_seen_target = None
        self.last_seen_time = None
        
        # NUEVO: Un Lock para evitar problemas de concurrencia al acceder a self.last_seen_target
        self.lock = threading.Lock()

        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        rospy.Subscriber('people_tracker_measurements', PositionMeasurementArray, self.people_callback)

        rospy.loginfo("Nodo seguidor de personas inicializado. Esperando detecciones...")

    def people_callback(self, msg):
        # El callback ahora es muy simple. Solo busca al objetivo y actualiza la variable.
        current_time = rospy.Time.now()
        
        with self.lock: # Adquirimos el lock para modificar variables compartidas
            if self.target_id:
                target_person = next((p for p in msg.people if p.name == self.target_id and p.reliability > self.reliability_threshold), None)
                
                if target_person:
                    self.last_seen_target = target_person
                    self.last_seen_time = current_time
                # Si no lo encuentra, el bucle principal se encargará del timeout
            else:
                # Lógica para encontrar un nuevo objetivo (más simple aquí)
                closest_person = None
                min_dist = float('inf')
                for person in msg.people:
                    if person.reliability > self.reliability_threshold:
                        dist = math.sqrt(person.pos.x**2 + person.pos.y**2)
                        if dist < min_dist:
                            min_dist = dist
                            closest_person = person
                
                if closest_person:
                    self.target_id = closest_person.name
                    self.last_seen_target = closest_person
                    self.last_seen_time = current_time
                    rospy.loginfo("Nuevo objetivo adquirido: {} a {:.2f} metros.".format(self.target_id, min_dist))

    def run(self):
        """
        NUEVO: Bucle principal que se ejecuta a una frecuencia constante.
        Aquí es donde vive toda la lógica de control y movimiento.
        """
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            twist_msg = Twist() # Empezamos con un comando de parada por defecto

            with self.lock: # Adquirimos el lock para leer variables compartidas
                if self.target_id and self.last_seen_target and self.last_seen_time:
                    time_since_last_seen = rospy.Time.now() - self.last_seen_time

                    if time_since_last_seen > self.lost_target_timeout:
                        rospy.logwarn("Objetivo {} perdido (timeout). Buscando nuevo objetivo.".format(self.target_id))
                        self.target_id = None
                        self.last_seen_target = None
                    else:
                        # --- LÓGICA DE MOVIMIENTO ---
                        x = self.last_seen_target.pos.x
                        y = self.last_seen_target.pos.y
                        distance = math.sqrt(x**2 + y**2)
                        angle_to_target = math.atan2(y, x)

                        angular_error = angle_to_target
                        angular_speed = self.k_angular * angular_error
                        angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

                        linear_error = distance - self.desired_distance
                        linear_speed = self.k_linear * linear_error
                        linear_speed = max(min(linear_speed, self.max_linear_speed), -self.max_linear_speed)

                        if distance < self.desired_distance:
                            linear_speed = 0
                        if abs(angle_to_target) > 0.3: # Priorizar girar si el objetivo está muy ladeado
                             linear_speed = 0

                        twist_msg.linear.x = linear_speed
                        twist_msg.angular.z = angular_speed
			rospy.loginfo("Velocidad en X: {:2f} y en z {:.2f}".format(linear_speed, angular_speed))
            
            # Publicamos el comando de velocidad calculado (o el de parada si no hay objetivo)
            self.cmd_pub.publish(twist_msg)
            
            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("El tiempo de ROS retrocedió, reiniciando espera.")


if __name__ == '__main__':
    try:
        follower = PersonFollower()
        follower.run() # Ahora llamamos al bucle principal
    except rospy.ROSInterruptException:
        pass
