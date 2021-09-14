import pybullet as p
import numpy as np
np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.5f}".format(x)})

from parameter import *
import time
import pybullet_data
from functions import *
import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2, PointField
import math
import struct
from std_msgs.msg import Header
from sensor_msgs import point_cloud2

import threading

targetPosition = [1.0,2.5,0,0,0,0,0,0,0];
pi = np.pi
near = 0.01
far = 1000
fov = 60

focal_length_x = 1/(math.tan((fov/180.0*pi)/2)*2/640)
focal_length_y = 1/(math.tan((fov/180.0*pi)/2)*2/480)
print(focal_length_x)
print(focal_length_y)
def callback(data):

	for i in range(len(data.position)):
		targetPosition[i] = data.position[i]
	
	
def convert_depth_frame_to_pointcloud(depth_image):
	camera_intrinsics ={"fx":focal_length_x,"ppx": 320,"fy":focal_length_y,"ppy":240}
	[height, width] = depth_image.shape
	nx = np.linspace(0, width-1, width)
	ny = np.linspace(0, height-1, height)
	u, v = np.meshgrid(nx, ny)
	x = (u.flatten() - camera_intrinsics["ppx"])/camera_intrinsics["fx"]
	y = (v.flatten() - camera_intrinsics["ppy"])/camera_intrinsics["fy"]

	z = depth_image.flatten() / 1000.0;
	x = np.multiply(x,z)
	y = np.multiply(y,z)

	x = x[np.nonzero(z)]
	y = y[np.nonzero(z)]
	z = z[np.nonzero(z)]
	return x, y, z
def getCameraImage(cam_pos,cam_orn):

	aspect = 640/480
	angle = 0.0;
	q = p.getQuaternionFromEuler(cam_orn)
	cam_orn = np.reshape(p.getMatrixFromQuaternion(q ),(3,3));
	view_pos = np.matmul(cam_orn,np.array([0.001,0,0.0]).T)
	view_pos = np.array(view_pos+cam_pos)
	view_matrix = p.computeViewMatrix([cam_pos[0],cam_pos[1],cam_pos[2]], view_pos, [0,0,1])
	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	images = p.getCameraImage(640,
					480,
					view_matrix,
					projection_matrix,
					shadow=False,
					renderer=p.ER_BULLET_HARDWARE_OPENGL)
	return images
def publishPointCloud(d435Id,d435Id2):
	global pub
	global pub_joint
	while 1:
		print("pub_start")
		d435pos, d435orn = p.getBasePositionAndOrientation(d435Id)
		d435quat = d435orn
		d435orn =  p.getEulerFromQuaternion(d435orn)
		image = getCameraImage(d435pos,d435orn)
		depth_img = np.array(image[3],dtype=np.float)
		depth_img = far * near / (far - (far - near) * depth_img)
		color_img = image[2]
		color_img = np.reshape(color_img,[640*480,4])
		depth = np.transpose(np.array(convert_depth_frame_to_pointcloud(depth_img),dtype=np.float))
		points = []
		roll = 0;
		pitch = 0;
		yaw = 0;
		Rx = np.array([[1 ,0 ,0],[0, math.cos(roll), -math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
		Ry = np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
		Rz = np.array([[math.cos(yaw) ,-math.sin(yaw) ,0],[math.sin(yaw), math.cos(yaw), 0],[0 ,0,1]])
		R2 = np.matmul(np.matmul(Rx,Ry),Rz);
		R = np.eye(3);
		R = np.matmul(R,R2);
		T = np.array([0.0,0.0,0.0])
		for i in range(0,len(depth),8):
		    x = (R[0,0]*depth[i,0]*1000.0+R[0,1]*depth[i,1]*1000.0+R[0,2]*depth[i,2]*1000.0+T[0])
		    y = (R[1,0]*depth[i,0]*1000.0+R[1,1]*depth[i,1]*1000.0+R[1,2]*depth[i,2]*1000.0+T[1])
		    z = (R[2,0]*depth[i,0]*1000.0+R[2,1]*depth[i,1]*1000.0+R[2,2]*depth[i,2]*1000.0+T[2])
		    r = int(color_img[i,0])
		    g = int(color_img[i,1])
		    b = int(color_img[i,2])
		    a = 255
		    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
		    pt = [x, y, z, rgb]
		    points.append(pt)

		fields = [PointField('x', 0, PointField.FLOAT32, 1),
			  PointField('y', 4, PointField.FLOAT32, 1),
			  PointField('z', 8, PointField.FLOAT32, 1),
			  # PointField('rgb', 12, PointField.UINT32, 1),
			  PointField('rgba', 12, PointField.UINT32, 1),
			  ]
		header = Header()
		header.frame_id = "map"
		pc2 = point_cloud2.create_cloud(header, fields, points)
		pc2.header.stamp = rospy.Time.now()

		pub.publish(pc2)
		js = JointState()
		js.name.append("lin_x_joint")
		js.name.append("lin_y_joint")
		js.name.append("rot_z_joint")		
		js.name.append("Arm_Joint_1")
		js.name.append("Arm_Joint_2")
		js.name.append("Arm_Joint_3")
		js.name.append("Arm_Joint_4")
		js.name.append("Arm_Joint_5")
		js.name.append("Arm_Joint_6")

		for i in range(0,8):
			js.position.append(targetPosition[i])
		pub_joint.publish(js)
def getHomogeneousMatrix(Id):
	pos, orn = p.getBasePositionAndOrientation(Id)
	T = np.eye(4);
	R =  np.reshape(p.getMatrixFromQuaternion(orn),(3,3))
	R = R
	T[0:3,0:3] = R
	T[0,3] = pos[0];
	T[1,3] = pos[1];
	T[2,3] = pos[2];
	return T

def getJointState(robotId,ArmJoint):
	jointState = p.getJointStates(robotId,ArmJoint)
	q = [jointState[0][0],jointState[1][0],jointState[2][0],jointState[3][0],jointState[4][0],jointState[5][0]]
	qdot =[jointState[0][1],jointState[1][1],jointState[2][1],jointState[3][1],jointState[4][1],jointState[5][1]]
	return q,qdot

def main():
	global pub
	global pub_joint

	p.connect(p.GUI)
	p.setAdditionalSearchPath(pybullet_data.getDataPath())

	useRealTimeSim = False
	p.setRealTimeSimulation(useRealTimeSim)
	p.setTimeStep(1/240.0)

	plane= p.loadURDF("urdf/Environment/environment.urdf",[0.0,2.5,0.0], p.getQuaternionFromEuler([0,0,0]))
	robotPATH = "urdf/mm_hyu/right_sim.urdf"
	p.setGravity(0, 0, -9.8)
	robotId = p.loadURDF(robotPATH,[1.0,0,0.0], p.getQuaternionFromEuler([0,0,0]))
	d435Id = p.loadURDF("./urdf/d435/d435.urdf", [0, 0, 0.0])
	p.resetBasePositionAndOrientation(d435Id, [0.0, 2.5, 1.5],p.getQuaternionFromEuler([0,pi/8,0]))


	NumberofJoint = p.getNumJoints(robotId)
	for i in range(p.getNumJoints(robotId)):
		print(p.getJointInfo(robotId, i))
		p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

	MobileJoint = [0,1,2]
	ArmJoint = [12,13,14,15,16,17]
	FTsensor = [11]
	x=1.0;
	y=0;
	wz = 0;

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/joint_states_desired", JointState, callback)

	pub = rospy.Publisher("/camera/depth/color/points", PointCloud2, queue_size=2)
	pub_joint = rospy.Publisher("/joint_states", JointState, queue_size=2)
	
	t = threading.Thread(target=publishPointCloud, args=(d435Id,d435Id))
	t.start()
	print(getHomogeneousMatrix(d435Id));
	rate=rospy.Rate(10);
	while(1):
		x = targetPosition[0];
		y = targetPosition[1];
		wz = targetPosition[2];
		p.resetBasePositionAndOrientation(robotId, [x, y, 0], p.getQuaternionFromEuler([0,0,wz]))	
		armNum=0;
		for j in ArmJoint:
			p.resetJointState(robotId, j, targetPosition[armNum+3])
			p.resetJointState(robotId, 17, 0.5)
			print(armNum,targetPosition[armNum+3])
			armNum = armNum+1
		joint_states = targetPosition

		rate.sleep()
		


if __name__ == "__main__":
    # execute only if run as a script
    main()
