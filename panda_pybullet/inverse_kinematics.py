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

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import threading
joint_states = [0,0,0,0,0,0,0];
near = 0.01
far = 1000
def callback(data):
	global joint_states
	joint_states = data.position
	
	#print(data)
	print(joint_states)

def convert_depth_frame_to_pointcloud(depth_image):
	camera_intrinsics ={"fx":554.2563,"ppx": 320,"fy":415.6922,"ppy":240}
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
if __name__ == "__main__":
	clid = p.connect(p.SHARED_MEMORY)
	if (clid < 0):
		p.connect(p.GUI)
		#p.connect(p.SHARED_MEMORY_GUI)

	p.setAdditionalSearchPath(pybullet_data.getDataPath())

	#p.loadURDF("plane.urdf", [0, 0, -1.0])
	p.loadURDF("shelfandtable/shelfandtable.urdf", [0, 0, 0.0])
	p.loadURDF("checkerboard/calibration.urdf", [0.45, 0.3, 0.15],p.getQuaternionFromEuler([0,-math.pi/2,math.pi/4]))

	d435Id = p.loadURDF("d435/d435.urdf", [0, 0, 0.0])
	p.resetBasePositionAndOrientation(d435Id, [-0.3, 0.4, 0.3],p.getQuaternionFromEuler([0,-math.pi,0]))
	pandaId = p.loadURDF("Panda/panda.urdf", [0, 0, 0])
	p.resetBasePositionAndOrientation(pandaId, [0, 0, 0], [0, 0, 0, 1])
	kukaEndEffectorIndex = 6
	numJoints = 7
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/joint_states_desired", JointState, callback)
	pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
	

	while 1:
		#print("joint state : ",joint_states)
		d435pos, d435orn = p.getBasePositionAndOrientation(d435Id)
		d435quat = d435orn
		d435orn =  p.getEulerFromQuaternion(d435orn)
		for i in range(numJoints):
			p.resetJointState(pandaId, i, joint_states[i])
		image = getCameraImage(d435pos,d435orn)
		depth_img = np.array(image[3],dtype=np.float)

		depth_img = far * near / (far - (far - near) * depth_img)
		#print(depth_img)
		color_img = image[2]
		color_img = np.reshape(color_img,[640*480,4])
		#print(color_img.shape)
		depth = np.transpose(np.array(convert_depth_frame_to_pointcloud(depth_img),dtype=np.float))
		#print(depth.shape)
		points = []
		R = np.reshape(np.array(p.getMatrixFromQuaternion(p.getQuaternionFromEuler([d435orn[0]+math.pi/2,d435orn[1],d435orn[2]-math.pi/2+math.pi])),dtype=np.float),(3,3))
		#print(R)
		T = np.array(d435pos,dtype=np.float)
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
		header.frame_id = "world"
		pc2 = point_cloud2.create_cloud(header, fields, points)
		pc2.header.stamp = rospy.Time.now()
		pub.publish(pc2)
		cv2.imshow("Depth",depth_img)
		cv2.waitKey(1)
		rospy.sleep(0.1)
	p.disconnect()
