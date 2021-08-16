import pybullet as p
import time
import numpy as np
import math
from datetime import datetime
import pybullet_data
import rospy
import cv2
from sensor_msgs.msg import JointState
import struct
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import threading

near = 0.01
far = 1000

def callback(data):
	global trig_command;
	global q_list
	q_list = [];
	trig_command=trig_command+1;
	for i in range(len(data.points)):
		q_list.append(data.points[i].positions)
	print(q_list)
	

def convert_depth_frame_to_pointcloud(depth_image):
	camera_intrinsics ={"fx":380,"ppx": 320,"fy":380,"ppy":240}
	[height, width] = depth_image.shape
	nx = np.linspace(0, width-1, width)
	ny = np.linspace(0, height-1, height)
	u, v = np.meshgrid(nx, ny)
	x = (u.flatten() - camera_intrinsics["ppx"])/camera_intrinsics["fx"]
	y = (v.flatten() - camera_intrinsics["ppy"])/camera_intrinsics["fy"]

	z = depth_image.flatten() / 1000;
	x = np.multiply(x,z)
	y = np.multiply(y,z)

	x = x[np.nonzero(z)]
	y = y[np.nonzero(z)]
	z = z[np.nonzero(z)]
	return x, y, z

 
def getCameraImage(cam_pos,cam_orn):
	fov = 60
	aspect = 640/480
	angle = 0.0;
	q = p.getQuaternionFromEuler(cam_orn)
	cam_orn = np.reshape(p.getMatrixFromQuaternion(q ),(3,3));
	view_pos = np.matmul(cam_orn,np.array([-0.001,0,0.0]).T)
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

def getHomogeneousMatrix(Id):
	pos, orn = p.getBasePositionAndOrientation(Id)
	T = np.eye(4);
	R =  np.reshape(p.getMatrixFromQuaternion(orn),(3,3))
	R = R.T
	T[0:3,0:3] = R
	T[0,3] = pos[0];
	T[1,3] = pos[1];
	T[2,3] = pos[2];
	return T
def publishPointCloud(d435Id,d435Id2):
	global pub

	while 1:
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
		print(T)
		for i in range(0,len(depth),8):
		    x = (R[0,0]*depth[i,0]*1000+R[0,1]*depth[i,1]*1000+R[0,2]*depth[i,2]*1000+T[0])
		    y = (R[1,0]*depth[i,0]*1000+R[1,1]*depth[i,1]*1000+R[1,2]*depth[i,2]*1000+T[1])
		    z = (R[2,0]*depth[i,0]*1000+R[2,1]*depth[i,1]*1000+R[2,2]*depth[i,2]*1000+T[2])
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
		header.frame_id = "panda_link0"
		pc2 = point_cloud2.create_cloud(header, fields, points)
		pc2.header.stamp = rospy.Time.now()

		pub.publish(pc2)
def jointStatePublisher():
	global joint_states
	js = JointState()
	js.header.stamp = rospy.Time.now()
	js.name.append("panda_joint1")
	js.name.append("panda_joint2")
	js.name.append("panda_joint3")
	js.name.append("panda_joint4")
	js.name.append("panda_joint5")
	js.name.append("panda_joint6")
	js.name.append("panda_joint7")
	js.position.append(0.0);
	js.position.append(0.0);
	js.position.append(0.0);
	js.position.append(0.0);
	js.position.append(0.0);
	js.position.append(0.0);
	js.position.append(0.0);
	
	js.position[0]=joint_states[0];
	js.position[1]=joint_states[1];
	js.position[2]=joint_states[2];
	js.position[3]=joint_states[3];
	js.position[4]=joint_states[4];
	js.position[5]=joint_states[5];
	js.position[6]=joint_states[6];
	return js;
		
    		
if __name__ == "__main__":
	global trig_commnad
	global joint_states
	global q_list
	global pub
	global pub2
	clid = p.connect(p.SHARED_MEMORY)
	if (clid < 0):
		p.connect(p.GUI)
		#p.connect(p.SHARED_MEMORY_GUI)

	p.setAdditionalSearchPath(pybullet_data.getDataPath())

	#p.loadURDF("plane.urdf", [0, 0, -1.0])
	tableId=p.loadURDF("./urdf/shelfandtable/shelfandtable.urdf", [0, 0, 0.0])
	#obstacleId = p.loadURDF("./urdf/checkerboard/calibration.urdf", [0.8, 0.0, 0.15],p.getQuaternionFromEuler([0,-math.pi/2,0]))

	d435Id = p.loadURDF("./urdf/d435/d435.urdf", [0, 0, 0.0])
	p.resetBasePositionAndOrientation(d435Id, [0.5, 0.5, 0.5],p.getQuaternionFromEuler([0,-math.pi+math.pi/4,-math.pi/2]))
	
	obstacleId = p.loadURDF("./urdf/obstacle/obstacle.urdf", [0, 0, 0.0])
	p.resetBasePositionAndOrientation(obstacleId, [0.0, 0.0, 1.5],p.getQuaternionFromEuler([0,0,0]))
		
	pandaId = p.loadURDF("./urdf/Panda/panda.urdf", [0, 0, 0])
	p.resetBasePositionAndOrientation(pandaId, [0, 0, 0], [0, 0, 0, 1])
	cid = p.createConstraint(tableId, -1, pandaId, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., 0],[0, 0, 0, 1])
	kukaEndEffectorIndex = 6
	numJoints = 7
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/joint_trajectory", JointTrajectory, callback)
	pub = rospy.Publisher("/camera/depth/color/points", PointCloud2, queue_size=2)
	pub2 = rospy.Publisher("/joint_states", JointState, queue_size=2)
	trig_command=0;
	q_list = [];
	
	x_Id = p.addUserDebugParameter("x", 0, 1, 0.663)
	y_Id = p.addUserDebugParameter("y", -1, 1, 0.0)
	z_Id = p.addUserDebugParameter("z", 0, 2, 0.263)
	joint_states = [0.000 ,0.3, 0.000, -1.556, 0.000, 1.571 ,0.7849457];
	t = threading.Thread(target=publishPointCloud, args=(d435Id,d435Id))
	
	t.start()
	js=jointStatePublisher()
	pub2.publish(js)
	while 1:
		for i in range(numJoints):
			p.resetJointState(pandaId, i, joint_states[i])
		x = p.readUserDebugParameter(x_Id)
		y = p.readUserDebugParameter(y_Id)
		z = p.readUserDebugParameter(z_Id)		
		p.resetBasePositionAndOrientation(obstacleId, [x, y, z], p.getQuaternionFromEuler([0,0,0]))	
		print(getHomogeneousMatrix(d435Id))
		if trig_command>0:
			for j in range(len(q_list)):
				joint_states = q_list[j]
				print(joint_states)
				for i in range(numJoints):
					p.resetJointState(pandaId, i, joint_states[i])
				#js=jointStatePublisher()
				#pub2.publish(js)
				p.stepSimulation();
				time.sleep(0.2);
		
		
			q_list = []
			trig_command=0;
		js=jointStatePublisher()
		pub2.publish(js)															
		p.stepSimulation();
		time.sleep(0.2);

	p.disconnect()

