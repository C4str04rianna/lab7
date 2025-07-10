import rbdl
import numpy as np

if __name__ == '__main__':

  # Lectura del modelo del robot a partir de URDF (parsing)
  modelo = rbdl.loadModel("/home/user/lab_ws/src/frlabs/lab7/urdf/robot_version1_proyect_grip.urdf")
  # Grados de libertad
  ndof = modelo.q_size
  print("el numero de grados es :",ndof)

  # Configuracion articular
  q = np.array([0.06, 0.4, 0.3, 0.4, 0.2, 0.6])
  # Velocidad articular
  dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
  # Aceleracion articular
  ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])
  
  
  # Arrays numpy
  zeros = np.zeros(ndof)          # Vector de ceros
  tau   = np.zeros(ndof)          # Para torque
  g     = np.zeros(ndof)          # Para la gravedad
  c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
  M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
  e     = np.eye(8)               # Vector identidad
  
  # Torque dada la configuracion del robot
  rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
  
  # Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
  # y matriz M usando solamente InverseDynamics
  
 # vector de gravedad: g = ID(q,0,0)
  rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
  # vector de coriolis: c = (ID(q,dq,0)-g)/dq
  rbdl.InverseDynamics(modelo, q, dq, zeros, c)
  c = c - g
  
  # matriz de inercia: M[1,:] = (ID(dq,0,e[1,:]) )/e[1,:]
  for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i,:], M[i,:])
    M[i,:] = M[i,:] - g 
  print("Vector de gravedad (g):")
  print(np.round(g,2))
  print("\n")
  print("Vector de Coriolis + Centrifuga (c):")
  print(np.round(c,2))
  print("\n")
  print("Matriz de Inercia (M):")
  print(np.round(M,2))
  print("\n")

  
  # Parte 2: Calcular M y los efectos no lineales b usando las funciones
  # CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
  # en los arreglos llamados M2 y b2
    # CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
  # en los arreglos llamados M2 y b2
  b2 = np.zeros(ndof)          # Para efectos no lineales
  M2 = np.zeros([ndof, ndof])  # Para matriz de inercia
 
  


  print("Parte 2:")

  rbdl.CompositeRigidBodyAlgorithm(modelo, q, M2)
  print("Matriz M2:")
  print(np.round(M2,2))
  print("\n")
  
  rbdl.NonlinearEffects(modelo, q, dq, b2)
  print("Efectos No Lineales:")
  print(np.round(b2,2))
  print("\n")

  # Parte 2: Verificacion de valores
  
  print("Verificando M:")
  print(np.round(M-M2,3))
  print("\n")
  
  print("Verificando :")
  print(np.round(c+g-b2,3))
  print("\n")
   

  # Parte 3: Verificacion de la expresion de la dinamica
  print("Parte 3:")
  tau1 = M.dot(ddq)+c+g
  tau2 = M2.dot(ddq)+b2
  
  print("Vector de torque inical:")
  print(np.round(tau,3))
  print("\n") 

  
  print("Vector de torque hallado con Inverse Dynamics:")
  print(np.round(tau1,3))
  print("\n")
  
  print("Vector de torque hallado con CompositeRigidBodyAlgorithm y NonlinearEffects:")
  print(np.round(tau2,3))
  print("\n")
  
  print("Error entre tao y tao 1:")
  print(np.round(tau - tau1,3))
  print("\n")
  
  print("Error entre tao1 y tao 2:")
  print(np.round(tau1 - tau2,3))
  print("\n")
  print(g,c,M)

