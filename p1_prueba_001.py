#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Twist

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower_node', anonymous=True)

        self.desired_distance = 1.5
        self.k_linear = 0.4
        self.k_angular = 0.8
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.5
        
        # ---- NUEVOS PARÁMETROS DE MEMORIA ----
        # Tiempo en segundos que esperaremos antes de declarar un objetivo como perdido.
        self.lost_target_timeout = rospy.Duration(2.0)
        # Distancia máxima (en metros) para considerar que un nuevo objetivo es el mismo que el anterior.
        self.reacquisition_threshold = 0.5 

        # ---- ESTADO DEL SEGUIDOR (MODIFICADO) ----
        self.target_id = None
        self.last_seen_target = None 
        # NUEVO: Guardaremos la hora a la que vimos por última vez al objetivo
        self.last_seen_time = rospy.Time.now()

        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        rospy.Subscriber('people_tracker_measurements', PositionMeasurementArray, self.people_callback)

        rospy.loginfo("Nodo seguidor de personas inicializado. Esperando detecciones...")

    def people_callback(self, msg):
        # Buscamos nuestro objetivo actual en la nueva lista de detecciones
        target_found = False
        if self.target_id is not None:
            for person in msg.people:
                if person.object_id == self.target_id:
                    self.last_seen_target = person
                    self.last_seen_time = rospy.Time.now() # NUEVO: Actualizamos el tiempo
                    target_found = True
                    break
        
        # --- LÓGICA DE CONTROL (COMPLETAMENTE NUEVA) ---

        if self.target_id is None:
            # CASO 1: No tenemos objetivo. Buscamos al más cercano.
            closest_person = None
            min_dist = float('inf')

            for person in msg.people:
                dist = math.sqrt(person.pos.x**2 + person.pos.y**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_person = person
            
            if closest_person is not None:
                self.target_id = closest_person.object_id
                self.last_seen_target = closest_person
                self.last_seen_time = rospy.Time.now()
                rospy.loginfo("Nuevo objetivo adquirido: {} a {:.2f} metros.".format(self.target_id, min_dist))
                self.move_towards_target() # Empezamos a movernos
        
        elif target_found:
            # CASO 2: Tenemos objetivo y lo hemos encontrado. Simplemente nos movemos.
            self.move_towards_target()

        else:
            # CASO 3: Tenemos objetivo pero no lo hemos encontrado en esta trama.
            # Comprobamos si ha pasado el tiempo de gracia.
            time_since_last_seen = rospy.Time.now() - self.last_seen_time
            if time_since_last_seen > self.lost_target_timeout:
                rospy.logwarn("Objetivo {} perdido (timeout). Buscando nuevo objetivo.".format(self.target_id))
                self.target_id = None
                self.stop_robot()
            else:
                # Aún no ha pasado el timeout. Intentamos re-adquirir por proximidad.
                reacquired = False
                for person in msg.people:
                    dist_to_last_pos = math.sqrt(
                        (person.pos.x - self.last_seen_target.pos.x)**2 + 
                        (person.pos.y - self.last_seen_target.pos.y)**2
                    )
                    if dist_to_last_pos < self.reacquisition_threshold:
                        rospy.loginfo("Objetivo re-adquirido por proximidad. ID antiguo: {}, ID nuevo: {}".format(self.target_id, person.object_id))
                        self.target_id = person.object_id
                        self.last_seen_target = person
                        self.last_seen_time = rospy.Time.now()
                        reacquired = True
                        break
                
                if reacquired:
                    self.move_towards_target()
                else:
                    # No hemos podido re-adquirir, así que esperamos y detenemos el robot.
                    rospy.loginfo("Objetivo {} no encontrado. Esperando...".format(self.target_id))
                    self.stop_robot()

    def move_towards_target(self):
        if self.last_seen_target is None:
            return

        x = self.last_seen_target.pos.x
        y = self.last_seen_target.pos.y

        distance = math.sqrt(x**2 + y**2)
        angle_to_target = math.atan2(y, x)

        angular_error = angle_to_target
        angular_speed = self.k_angular * angular_error
        angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

        # MODIFICADO: Permitimos retroceder un poco si estamos demasiado cerca.
        linear_error = distance - self.desired_distance
        linear_speed = self.k_linear * linear_error
        linear_speed = max(min(linear_speed, self.max_linear_speed), -self.max_linear_speed)
        
        # Medida de seguridad: si el objetivo está muy ladeado, priorizamos girar sobre avanzar.
        if abs(angle_to_target) > 0.5: # 0.5 radianes son unos 30 grados
            linear_speed = 0

        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_pub.publish(twist_msg)

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.cmd_pub.publish(twist_msg)

    def run(self):
        rospy.spin()
        self.stop_robot()

if __name__ == '__main__':
    try:
        follower = PersonFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass