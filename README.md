IMPORTANTE, HE DESCUBIERTO DONDE FALLABA, el fallo residia en el archivo .launch que os muestro a continuación el bueno:
<launch>
	<arg name="scan" default="/scan"/>
	<!--<arg name="machine" default="localhost"/>
	<arg name="user" default=""/> -->
	<node pkg="leg_detector" type="leg_detector" name="leg_detector" args="scan:=$(arg scan) $(find leg_detector)/config/trained_leg_detector.yaml" respawn="true" output="screen">
		<param name="fixed_frame" type="string" value="base_link"/>
	</node>
</launch>

Pues el problema estaba en la última línea <param name="fixed_frame" type="string" value="base_link"/> más en concreto en la parte de value="base_link", porque tenía configurado que el maldito value fuera value="odom" y eso lo que hacía era que referenciaba todo a donde se encendía el robot, no a la posición donde se encontraba el robot, y solo había que cambiar en el launch de value="odom" a value="base_link" que es donde fijamos la posición al robot.
Importante también, no funciona con los rosbag, o lo que es decir, los rosbag se inicia en el mismo sitio el odom que el base_link, por lo tanto no se mueve el robot del punto de inicio del rosbag por lo tanto es el mismo, entonces claro no se ve diferencia por eso yo decía que en los rosbag si que funcionaba y que en real no pues ese era el truco, entre que nosotros arrancábamos el robot en un sitio y lo usábamos en otro con el odom ese era el truco del almendruco, fallo solventado a las 23:27 asique podré dormir hoy supongo :)

