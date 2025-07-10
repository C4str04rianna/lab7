
import numpy as np
import matplotlib.pyplot as plt

# Leer archivos
qactual = np.loadtxt("/home/user/qactual_articular.txt")
qdeseado = np.loadtxt("/home/user/qdeseado_articular.txt")
xactual = np.loadtxt("/home/user/xactual_articular.txt")
xdeseado = np.loadtxt("/home/user/xdeseado_articular.txt")

#qactual = np.loadtxt("/tmp/qactual_inv.txt")
#qdeseado = np.loadtxt("/tmp/qdeseado_inv.txt")
#xactual = np.loadtxt("/tmp/xcurrent_inv.txt")
#xdeseado = np.loadtxt("/tmp/xdesired_inv.txt")

# Verifica que tengan el mismo número de muestras
assert qactual.shape[0] == qdeseado.shape[0] == xactual.shape[0] == xdeseado.shape[0], "Los archivos no tienen la misma longitud"

# Tiempo
t = qactual[:, 0]

# Plot q_actual vs q_deseado para cada articulación
plt.figure(figsize=(12, 8))
for i in range(1, 7):
    plt.subplot(3, 2, i)
    plt.plot(t, qactual[:, i], label=f'q_actual[{i-1}]', linewidth=1.5)
    plt.plot(t, qdeseado[:, i], '--', label=f'q_deseado[{i-1}]', linewidth=1.5)
    plt.xlabel('Tiempo [s]')
    plt.ylabel('q [rad]')
    plt.legend()
    plt.grid(True)
plt.suptitle("Comparación q_actual vs q_deseado")
plt.tight_layout()
plt.show()

# Plot x_actual vs x_deseado en X, Y, Z
labels = ['X', 'Y', 'Z']
plt.figure(figsize=(10, 6))
for i in range(1, 4):
    plt.subplot(3, 1, i)
    plt.plot(t, xactual[:, i], label=f'x_actual {labels[i-1]}', linewidth=1.5)
    plt.plot(t, xdeseado[:, i], '--', label=f'x_deseado {labels[i-1]}', linewidth=1.5)
    plt.xlabel('Tiempo [s]')
    plt.ylabel(f'{labels[i-1]} [m]')
    plt.legend()
    plt.grid(True)
plt.suptitle("Comparación x_actual vs x_deseado")
plt.tight_layout()
plt.show()
