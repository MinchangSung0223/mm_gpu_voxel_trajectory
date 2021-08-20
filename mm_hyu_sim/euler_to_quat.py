import pybullet as p
import sys

print(p.getQuaternionFromEuler([float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])]))
