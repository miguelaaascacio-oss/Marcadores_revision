import numpy as np
#puntos desnormalizados

Puntos = np.array([[ 1.00000000e+01,  1.00000000e+01],
                 [-1.59373661e-08,  1.00000000e+01],
                 [3.52941176e+00,  7.80392156e+00],
                 [6.47058824e+00,  7.80392156e+00],
                 [-1.59373661e-08,  2.48834553e-09],
                 [1.00000000e+01,  4.53996165e-08]
                 ])
                 
Pdesplazados = np.zeros((6,2))
Pnormal = np.zeros((6,2))
                 
#primer paso a los puntos desnormalizados, trasladarlos 5 unidades

desplazamiento = np.array([-5,-5])
Pdesplazados = Puntos + desplazamiento

#calcular nuevo centroide
cx = np.average(Pdesplazados[:,0])
cy = np.average(Pdesplazados[:,1])

#calcular nueva varianza
distancia_x = Pdesplazados[:,0] - cx
distancia_y = Pdesplazados[:,1] - cy
 
sigma_x = np.sqrt(np.mean(distancia_x**2))
sigma_y = np.sqrt(np.mean(distancia_y**2))

#nuevo vpd

Vdp = [cx, sigma_x, cy, sigma_y]

#normalizamo

Pnormal[:,0] = (Pdesplazados[:,0] - cx)/ sigma_x 
Pnormal[:,1] = (Pdesplazados[:,1] - cy)/ sigma_y 

#desnormalizamos para ver

Pdesplazados[:,0] = Pnormal[:,0]*Vdp[1] + Vdp[0]
Pdesplazados[:,1] = Pnormal[:,1]*Vdp[3] + Vdp[2]

print(Pdesplazados)
print("\n")
print(Pnormal)
print("\n")
print(np.array(Vdp))
