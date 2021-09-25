import sys
import pybullet as p
import numpy as np

R = np.reshape(p.getMatrixFromQuaternion(p.getQuaternionFromEuler([float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])])),[3,3])

print(str(R[0,0])+" "+str(R[0,1])+" "+str(R[0,2])+" 5.34" )
print(str(R[1,0])+" "+str(R[1,1])+" "+str(R[1,2])+" 2.90")
print(str(R[2,0])+" "+str(R[2,1])+" "+str(R[2,2])+" 1.4")
print(str(0)+" "+str(0)+" "+str(0)+" 1")

