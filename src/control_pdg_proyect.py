#!/usr/bin/env python3

#############################################
#############################################
#             CONTROL ARTICULAR
#############################################
#############################################


import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *
from roslib import packages
from std_msgs.msg import Float64
import PyKDL
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel




if __name__ == '__main__':

  rospy.init_node("control_pdg_proyect")
  pub = rospy.Publisher('joint_states', JointState, queue_size=1)
  
  bmarker_actual  = BallMarker(color['RED'])
  bmarker_deseado = BallMarker(color['GREEN'])
  
  # Archivos donde se almacenara los datos
  fqact = open("/home/user/qactual_articular.txt", "w")
  fqdes = open("/home/user/qdeseado_articular.txt", "w")
  fxact = open("/home/user/xactual_articular.txt", "w")
  fxdes = open("/home/user/xdeseado_articular.txt", "w")
  ftorque = open("/home/user/torque_articular.txt", "w")
  
  # Nombres de las articulaciones
  jnames = ['motor1', 'motor2', 'motor3', 'motor4', 'motor5', 'motor6']
  # Objeto (mensaje) de tipo JointState
  jstate = JointState()
  # Valores del mensaje
  jstate.header.stamp = rospy.Time.now()
  jstate.name = jnames
  
  # =============================================================
  # Configuracion articular inicial (en radianes)
  q = np.array([0.0, 0, 0, 0, 0, 0.0])
  # Velocidad inicial
  dq = np.array([0., 0., 0., 0., 0., 0.])
  # Configuracion articular deseada
  qdes = np.array([2.7, -0.9, 0.2, 0.2, 0.2, 0.2])




  # =============================================================
  
  # Posicion resultante de la configuracion articular deseada
  xdes = fkine_mh5lsii(qdes)[0:3,3]
  # Copiar la configuracion articular en el mensaje a ser publicado
  jstate.position = q
  pub.publish(jstate)
  
  # Definir metodos de PyKDL para el UR5
  # Cargar modelo URDF

    # Verifica que la cadena se creó correctamente



  robot_urdf = URDF.from_xml_file("/home/user/lab_ws/src/frlabs/lab7/urdf/robot_version1_proyect_grip.urdf")
  ok, tree = treeFromUrdfModel(robot_urdf)
  chain = tree.getChain("dummy_base", "link6_1")

  for i in range(chain.getNrOfSegments()):
    joint_name = chain.getSegment(i).getJoint().getName()
    print(f"{i}: {joint_name}")
  


  from kdl_parser_py.urdf import treeFromString

  with open("/home/user/lab_ws/src/frlabs/lab7/urdf/robot_version1_proyect_grip.urdf", 'r') as f:
    urdf_str = f.read()
    
    ok, tree = treeFromString(urdf_str)
  if not ok:
    rospy.logerr("¡Error al construir el árbol KDL!")
    exit(1)



  ndof = chain.getNrOfJoints()
  print(ndof)
  gravity = PyKDL.Vector(0.0, 0.0, -9.81)
  dyn_solver = PyKDL.ChainDynParam(chain, gravity)

  # Vectores de PyKDL
  q_kdl = PyKDL.JntArray(ndof)
  dq_kdl = PyKDL.JntArray(ndof)
  g_kdl = PyKDL.JntArray(ndof)
  
  # Frecuencia del envio (en Hz)
  freq = 20
  dt = 1.0/freq
  rate = rospy.Rate(freq)
  
  # Simulador dinamico del robot
  robot = Robot(q, dq, ndof, dt)

  # Se definen las ganancias del controlador
  Kp_val = np.array([50.0, 0.15, 0.2, 0.25, 0.05, 0.005])
  Kd_val = 2 * np.sqrt(Kp_val)
  Kp     = np.diag(Kp_val)
  Kd     = np.diag(Kd_val)     
  
  # Bucle de ejecucion continua
  t = 0.0
  while not rospy.is_shutdown():
  
    # Leer valores del simulador
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Posicion actual del efector final
    x = fkine_mh5lsii(q)[0:3,3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Almacenamiento de datos
    #ftorque.write(str(u)+' '+str(u[0])+' '+str(u[1])+' '+str(u[2])+'\n')
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')

    # ----------------------------
    # Control dinamico (COMPLETAR)
    # ----------------------------
    u = np.zeros(ndof)   # Reemplazar por la ley de control
    # Convertir a JntArray para PyKDL
    for i in range(ndof):
        q_kdl[i] = q[i]

    # Calcular la compensación de gravedad
    dyn_solver.JntToGravity(q_kdl, g_kdl)
    g = np.array([g_kdl[i] for i in range(ndof)]) 

    rospy.loginfo_throttle(1.0, f"Gravedad calculada: {g} \n")  
    rospy.loginfo_throttle(1.0, f"q deseado: {q}  \n")  
    rospy.loginfo_throttle(1.0, f"dq deseado: {dq}  \n")


    # Ley de control PD + gravedad
    e = qdes - q       # Error de posición
    de = -dq           # Error de velocidad (qd_des = 0)
    u = Kp.dot(e) + Kd.dot(de) + g

    ftorque.write(str(t) + ' ' + ' '.join(str(val) for val in u) + '\n')

    # Simulacion del robot
    robot.send_command(u)

    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t + dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

  fqact.close()
  fqdes.close()
  fxact.close()
  fxdes.close()
  ftorque.close()
