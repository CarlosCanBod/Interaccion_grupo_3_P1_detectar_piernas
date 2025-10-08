#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Twist

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower_node', anonymous=True)

        # ---- Parámetros de Control ----
        self.desired_distance = 1.5
        self.k_linear = 0.4
        self.k_angular = 1
        self.max_linear_speed = 0.2
        self.max_angular_speed = 0.8
        self.reliability_threshold = 0.65

        # ---- Parámetros de Comportamiento Avanzado ----
        self.stationary_timeout = rospy.Duration(10.0)
        self.stationary_distance_threshold = 0.10
        
        # NUEVO: Periodo de gracia en segundos para la pérdida de objetivo (anti-parpadeo)
        self.lost_target_timeout = rospy.Duration(5.0)
        # NUEVO: Duración en segundos que un objetivo inmóvil será ignorado
        self.ignore_duration = rospy.Duration(30.0)

        # ---- Estado del Seguidor ----
        self.target_id = None
        self.last_seen_target = None
        self.monitoring_start_pos = None
        self.monitoring_start_time = None
        
        # NUEVO: Hora en que vimos por última vez al objetivo (para el anti-parpadeo)
        self.last_seen_time = None
        # NUEVO: Lista de IDs ignorados y cuándo expira su "baneo"
        self.ignore_list = {} # Diccionario: { 'PersonID': expiration_time }

        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        rospy.Subscriber('people_tracker_measurements', PositionMeasurementArray, self.people_callback)

        rospy.loginfo("Nodo seguidor de personas inicializado. Esperando detecciones...")

    def people_callback(self, msg):
        current_time = rospy.Time.now()
        
        # 1. Limpiar la lista de ignorados de entradas expiradas
        self.ignore_list = {pid: expiry for pid, expiry in self.ignore_list.items() if expiry > current_time}
        
        # 2. Filtrar las detecciones para quedarnos solo con las fiables
        reliable_people = [p for p in msg.people if p.reliability > self.reliability_threshold]
        
        # 3. Lógica principal de seguimiento
        if self.target_id:
            # Si tenemos un objetivo, intentamos encontrarlo
            target_person = next((p for p in reliable_people if p.name == self.target_id), None)

            if target_person:
                # --- OBJETIVO ENCONTRADO Y FIABLE ---
                self.last_seen_target = target_person
                self.last_seen_time = current_time

                # Comprobar si el objetivo está inmóvil
                distance_moved = math.sqrt(
                    (self.last_seen_target.pos.x - self.monitoring_start_pos.x)**2 +
                    (self.last_seen_target.pos.y - self.monitoring_start_pos.y)**2
                )
                
                if distance_moved > self.stationary_distance_threshold:
                    self.monitoring_start_pos = self.last_seen_target.pos
                    self.monitoring_start_time = current_time
                    self.move_towards_target()
                else:
                    if current_time - self.monitoring_start_time > self.stationary_timeout:
                        rospy.logwarn("Objetivo {} ignorado por estar inmóvil. Añadido a la lista de ignorados.".format(self.target_id))
                        self.ignore_list[self.target_id] = current_time + self.ignore_duration
                        self.reset_target()
                    else:
                        self.move_towards_target()
            else:
                # --- OBJETIVO NO ENCONTRADO (POSIBLE PARPADEO) ---
                if current_time - self.last_seen_time > self.lost_target_timeout:
                    rospy.logwarn("Objetivo {} perdido (timeout). Buscando nuevo objetivo.".format(self.target_id))
                    self.reset_target()
                else:
                    # Aún estamos en el periodo de gracia, solo nos detenemos y esperamos.
                    self.stop_robot()
        
        if not self.target_id:
            # --- BUSCANDO UN NUEVO OBJETIVO ---
            closest_person = None
            min_dist = float('inf')

            # Considerar solo personas que no estén en la lista de ignorados
            candidate_people = [p for p in reliable_people if p.name not in self.ignore_list]

            for person in candidate_people:
                dist = math.sqrt(person.pos.x**2 + person.pos.y**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_person = person
            
            if closest_person:
                self.target_id = closest_person.name
                self.last_seen_target = closest_person
                self.monitoring_start_pos = closest_person.pos
                self.monitoring_start_time = current_time
                self.last_seen_time = current_time
                rospy.loginfo("Nuevo objetivo adquirido: {} a {:.2f} metros.".format(self.target_id, min_dist))

    def reset_target(self):
        """Función auxiliar para limpiar el estado del objetivo."""
        self.target_id = None
        self.monitoring_start_pos = None
        self.monitoring_start_time = None
        self.last_seen_target = None
        self.stop_robot()

    def move_towards_target(self):
        # (Esta función no necesita cambios)
        if self.last_seen_target is None: return
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
        if distance < self.desired_distance: linear_speed = 0
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_pub.publish(twist_msg)

    def stop_robot(self):
        # (Esta función no necesita cambios)
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
