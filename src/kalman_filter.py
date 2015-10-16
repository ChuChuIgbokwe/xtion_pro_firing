import numpy as np 
def filter(x,P):
	x = F*x + u
	P = F * P * F.I

	Z = np.matrix(measurements[n])
	y = Z - (H*x)
	S = H * P * H.T + R
	K = P * H.T * S.I
	x = x + (K * y)
	P = (I * (K * H)) * P

measurements = [1,2,3]
x = np.matrix('1000.0,0.0; 0.0 ,1000.0') # initial state (location and velocity)
P = np.matrix('1000.0,0.0 ; 0.0, 1000.0') # initial uncertainty
u = np.matrix('0.0; 0.0') # external motion
F = np.matrix('1.0,1.0; 0.0,1.0') # next state function
H = np.matrix('1.0,0.0') # measurement function
R = np.matrix('1.0') # measurement uncertainty
I = np.matrix('1.0,0.0; 0.0,1.0') # identity matrix
filter (x,P)

