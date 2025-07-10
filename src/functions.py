#!/usr/bin/env python3
import numpy as np
from copy import copy
import rbdl

cos=np.cos; sin=np.sin; pi=np.pi

import numpy as np
import rbdl

import numpy as np
import rbdl

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0.copy()       # Estado de posición
        self.dq = dq0.copy()     # Estado de velocidad
        self.M = np.zeros((ndof, ndof))  # Matriz de inercia
        self.b = np.zeros(ndof)          # Términos no lineales
        self.dt = dt                     # Paso de integración
        self.ndof = ndof
        self.robot = rbdl.loadModel("/home/user/lab_ws/src/frlabs/lab7/urdf/robot_version1_proyect_grip.urdf")

        # Límites de protección
        self.max_torque = 50.0          # Nm
        self.max_velocity = 5.0         # rad/s
        self.max_position = np.pi * 2   # 360° por seguridad

    def send_command(self, tau):
        # Saturar torque
        tau = np.clip(tau, -self.max_torque, self.max_torque)

        # Verificar que q y dq no tienen NaNs
        if np.any(np.isnan(self.q)) or np.any(np.isnan(self.dq)):
            print("❌ NaN detectado en q o dq. Reinicia o detén la simulación.")
            return

        # Calcular dinámica
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)

        # Condición de matriz M
        cond_M = np.linalg.cond(self.M)
        if cond_M > 1e12:
            print(f"⚠️ Matriz M mal condicionada: cond(M) = {cond_M:.2e}")
            return

        # Aceleraciones
        try:
            ddq = np.linalg.solve(self.M, tau - self.b)
        except np.linalg.LinAlgError:
            print("❌ Error: matriz M no invertible.")
            return

        # Euler semiimplícito
        self.dq += self.dt * ddq
        self.dq = np.clip(self.dq, -self.max_velocity, self.max_velocity)

        self.q += self.dt * self.dq
        self.q = np.clip(self.q, -self.max_position, self.max_position)

        # Verificar que no se generaron NaNs
        if np.any(np.isnan(self.q)) or np.any(np.isnan(self.dq)):
            print("❌ NaN generado tras integración. Verifica el control.")
            return

    def read_joint_positions(self):
        return self.q.copy()

    def read_joint_velocities(self):
        return self.dq.copy()



def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.
    """
    # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    cth = cos(theta);    sth = sin(theta)
    ca = cos(alpha);  sa = sin(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                [sth,  ca*cth, -sa*cth, a*sth],
                [0,        sa,     ca,      d],
                [0,         0,      0,      1]])

    return T
    
    

def fkine_mh5lsii(q):
    """
    Cinemática directa del robot MH5LSII
    q: Vector numpy [q1, q2, q3, q4, q5, q6] en radianes
    Retorna: Matriz de transformación homogénea 4x4
    """
    # Parámetros DH actualizados según tu URDF [d, theta, a, alpha]
    T1 = dh(0.100, q[0], 0, 0)
    T2 = dh(0, q[1] + pi/2, 0.300, pi/2)
    T3 = dh(0.250, q[2] - pi/2, 0, -pi/2)
    T4 = dh(0, q[3] + pi/2, 0.300, pi/2)
    T5 = dh(0.300, q[4] - pi/2, 0, -pi/2)
    T6 = dh(0, q[5] + pi/2, 0.140, pi/2)

    
    # Transformación final (corregida sin duplicación)
    T = T1 @ T2 @ T3 @ T4 @ T5 @ T6
    return T

def fkine_mh5lsi(q):
    """
    Cinemática directa del robot MH5LSII
    q: Vector numpy [q1, q2, q3, q4, q5, q6] en radianes
    Retorna: Matriz de transformación homogénea 4x4
    """
    # Parámetros DH actualizados según tu URDF [d, theta, a, alpha]
    T1 = dh(0.100, q[0], 0, 0)
    T2 = dh(0, q[1] + pi/2, 0.300, pi/2)
    T3 = dh(0.250, q[2] - pi/2, 0, -pi/2)
    T4 = dh(0, q[3] + pi/2, 0.300, pi/2)
    T5 = dh(0.300, q[4] - pi/2, 0, -pi/2)
    T6 = dh(0, q[5] + pi/2, 0.140, pi/2)
    T7 = dh(0, 0 + q[6], 0, 0)
    T8 = dh(0, 0 + q[7], 0, 0)

    
    # Transformación final (corregida sin duplicación)
    T = T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ T7 @ T8
    return T


def jacobian_mh5lsii(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x6
    J = np.zeros((3,6))
    # Transformacion homogenea inicial (usando q) Cinematica directa
    T = fkine_mh5lsii(q)
   
    # Iteracion para la derivada de cada columna
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i] + delta
        # Transformacion homogenea luego del incremento (q+delta)
        dT = fkine_mh5lsii(dq)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[:, i] = (dT[0:3, 3] - T[0:3, 3])/delta
    return J

def ikine_mh5lsii(xdes, q0,documento):


    epsilon  = 0.001
    max_iter = 100
    delta    = 0.001

    q  = copy(q0)
    for i in range(max_iter):
        q1 = q[0]; q2 = q[1];q3 = q[2];q4 = q[3];q5 = q[4];q6 = q[5];q7 = q[6]
        if q4 > 0.05:
            q[3] = 0.05
        elif q4 <0.00:
            q[3] = 0.0    
        J = jacobian_mh5lsii(q)
        T = fkine_mh5lsii(q)
        f =T[0:3,3]
        e = xdes-f # Error
        q = q + np.dot(np.linalg.pinv(J), e)# Actualización de q (método de Newton)
        print("iteración: {}, error: {}".format(i, np.linalg.norm(e)))
        
        documento.write(str(i)+' '+str(np.linalg.norm(e)) +' '+str(epsilon)+'\n')
        
        if (np.linalg.norm(e) < epsilon): # Condición de término
            break
    
    return q

def ik_gradient_mh5lsii(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo gradiente
    """
    epsilon  = 0.0001
    max_iter = 10000
    delta    = 0.00001
    alfa     = 0.9

    q  = copy(q0)
    for i in range(max_iter):
        q1 = q[0]; q2 = q[1];q3 = q[2];q4 = q[3];q5 = q[4];q6 = q[5]
        J = jacobian_mh5lsii(q,delta)
        T = fkine_mh5lsii(q)# Posición del efector final (cinemática directa)
        f = T[0:3,3]
        e = xdes-f # Error
        q = q + alfa*np.dot(J.T, e)# Actualización del valor articular (descenso del gradiente)
        if (np.linalg.norm(e) < epsilon): # Condición de término
            break
    
    return q

def jacobian_pose(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    J = np.zeros((7,6))
    # Implementar este Jacobiano aqui
    T = fkine_mh5lsii(q)
    R = np.array(T[0:3,0:3])
    Q = TF2xyzquat(T)
    #Qi = np.array(Q)
    for i in range (7):
        # Copiar la configuracion articular inicial (usar este dq para cada
        # incremento en una articulacion)
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i]+delta
        # Transformacion homogenea luego del incremento (q+dq)
        Tq = fkine_mh5lsii(dq)
        Rq = np.array(Tq[0:3,0:3])
        Qq = TF2xyzquat(Tq)
        #Qq = np.array(Qqq)
    	# Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0][i] = (Tq[0][3]-T[0][3])/delta #x
        J[1][i] = (Tq[1][3]-T[1][3])/delta #y
        J[2][i] = (Tq[2][3]-T[2][3])/delta #z
        J[3][i] = (Qq[3]-Q[3])/delta #w
        J[4][i] = (Qq[4]-Q[4])/delta #ex
        J[5][i] = (Qq[5]-Q[5])/delta #ey
        J[6][i] = (Qq[6]-Q[6])/delta #ez
        
    return J



def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion

    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]

    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R

def jacobian_position(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]


    """
    # Alocacion de memoria
    J = np.zeros((3,6))
    # Transformacion homogenea inicial (usando q)
    T = fkine_mh5lsii(q)
    T = T[0:3, -1:]


    # Iteracion para la derivada de cada columna
    for i in range(7):
        # Copiar la configuracion articular inicial (usar este dq para cada
        # incremento en una articulacion)
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i] + delta


        # Transformacion homogenea luego del incremento (q+dq)
        Tf = fkine_mh5lsii(dq)   #Tf= T_final
        Tf =  Tf[0:3, -1:]


        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        Jq = 1/delta*(Tf-T)
        J[:, i:i+1] = Jq 


    return J