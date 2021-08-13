import pybullet as p
import time
import numpy as np
np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.3f}".format(x)})
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
imageWidth  = 640;
imageHeight  = 640;
def callback(data):
	global joint_states
	joint_states = data.position
	#print(data)
	#print(joint_states)

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
def getMatrixfromEEf(eef_pose):
	Tbe = np.eye(4)
	eef_p = np.array(eef_pose[0])
	eef_o = np.array(eef_pose[1])
	print(eef_o)
	R = np.reshape(p.getMatrixFromQuaternion(eef_o),[3,3])
	
	Tbe[0:3,0:3] = R
	Tbe[0,3] = eef_p[0]
	Tbe[1,3] = eef_p[1]
	Tbe[2,3] = eef_p[2]
	return Tbe
 
def getCameraImage(cam_pos,cam_orn):
	fov = 60
	aspect = imageWidth/imageHeight
	near = 0.01
	far = 1000
	angle = 0.0;
        q = p.getQuaternionFromEuler(cam_orn)
	cam_orn = np.reshape(p.getMatrixFromQuaternion(q ),(3,3));
	view_pos = np.matmul(cam_orn,np.array([-0.001,0,0.0]).T)
	view_pos = np.array(view_pos+cam_pos[0:3])
	view_matrix = p.computeViewMatrix([cam_pos[0],cam_pos[1],cam_pos[2]], view_pos, [0,0,1])
	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	images = p.getCameraImage(imageWidth,
					imageHeight,
					view_matrix,
					projection_matrix,
					shadow=True,
					renderer=p.ER_BULLET_HARDWARE_OPENGL)
	return images


def getCameraImageEEF(eef_pose):
	#print(eef_pose)
	com_p = eef_pose[0]
	com_o = eef_pose[1]
	rot_matrix = p.getMatrixFromQuaternion(com_o)
	rot_matrix = np.array(rot_matrix).reshape(3, 3)
	# Initial vectors
	init_camera_vector = (0, 0, 1) # z-axis
	init_up_vector = (0, 1, 0) # y-axis
	# Rotated vectors
	camera_vector = rot_matrix.dot(init_camera_vector)
	up_vector = rot_matrix.dot(init_up_vector)
	view_matrix = p.computeViewMatrix([com_p[0],com_p[1],com_p[2]],[com_p[0],com_p[1],com_p[2]] + 100 * camera_vector, up_vector)
	fov = 60
	aspect = imageWidth/imageHeight
	near = 0.01
	far = 1000
	angle = 0.0;


	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	images = p.getCameraImage(imageWidth,
					imageHeight,
					view_matrix,
					projection_matrix,
					shadow=True,
					renderer=p.ER_BULLET_HARDWARE_OPENGL)
	return images

if __name__ == "__main__":
	clid = p.connect(p.SHARED_MEMORY)
	if (clid < 0):
		p.connect(p.GUI)
		#p.connect(p.SHARED_MEMORY_GUI)

	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	p.loadURDF("checkerboard/calibration.urdf", [0.5, 0, 0.01])
	p.loadURDF("plane.urdf", [0, 0, 0.0])
	d435Id = p.loadURDF("d435/d435.urdf", [0, 0, 0.0])
	p.resetBasePositionAndOrientation(d435Id, [1, 0, 1],p.getQuaternionFromEuler([0,-math.pi/4,0]))
	pandaId = p.loadURDF("Panda/panda.urdf", [0, 0, 0])
	p.resetBasePositionAndOrientation(pandaId, [0, 0, 0], [0, 0, 0, 1])
	kukaEndEffectorIndex = 6
	numJoints = 7
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/joint_states_desired", JointState, callback)
	pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
	count = 0

	while 1:
		#print("joint state : ",joint_states)
		d435pos, d435orn = p.getBasePositionAndOrientation(d435Id)
		d435orn =  p.getEulerFromQuaternion(d435orn)
		for i in range(numJoints):
			p.resetJointState(pandaId, i, joint_states[i])
		

		
		eef_pose = p.getLinkState(pandaId,9)
		
		
		image =getCameraImageEEF(eef_pose) #getCameraImage(d435pos,d435orn)
		eef_pose = p.getLinkState(pandaId,8)
		Tbe = getMatrixfromEEf(eef_pose)
		Tec = np.array([[0 ,-1 ,0, 0.1],[1 ,0 ,0, 0.0],[0 ,0 ,1, 0.0],[0,0,0,1]])
		Tbc = np.matmul(Tbe,Tec)
		depth_img = np.array(image[3],dtype=np.float)
		near = 0.01
		far = 1000
		depth_img = far * near / (far - (far - near) * depth_img)
		#print(depth_img)
		color_img = image[2]
		color_img_temp = color_img.copy()
		color_img = np.reshape(color_img,[imageWidth*imageHeight,4])
		#print(color_img.shape)
		depth = np.transpose(np.array(convert_depth_frame_to_pointcloud(depth_img),dtype=np.float))
		#print(depth.shape)
		points = []
		R = np.reshape(np.array(p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0,0,math.pi])),dtype=np.float),(3,3))
		#print(R)
		T = np.array(d435pos,dtype=np.float)
		#depth[:,0] = depth[:,0]-0.1;
		for i in range(0,len(depth),8):
		    x = (R[0,0]*(depth[i,0])*1000+R[0,1]*depth[i,1]*1000+R[0,2]*depth[i,2]*1000)
		    y = (R[1,0]*(depth[i,0])*1000+R[1,1]*depth[i,1]*1000+R[1,2]*depth[i,2]*1000)
		    z = (R[2,0]*(depth[i,0])*1000+R[2,1]*depth[i,1]*1000+R[2,2]*depth[i,2]*1000)
		    #x = -depth[i,0]*1000+0.1;
		    #y = -depth[i,1]*1000;
		    #z = depth[i,2]*1000;

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
		header.frame_id = "panda_hand"
		pc2 = point_cloud2.create_cloud(header, fields, points)
		pc2.header.stamp = rospy.Time.now()
		pub.publish(pc2)
		cv2.imshow("Depth",depth_img)

		k = cv2.waitKey(1)
		if k == 27:         # wait for ESC key to exit    
			cv2.destroyAllWindows()
		elif k == ord('s'): # wait for 's' key to save and exit  
			print("Save "+str(count)+" data" )
			cv2.imwrite("Images/"+str(count)+'.png',color_img_temp)    
			np.savetxt("poses/"+str(count)+".txt",Tbe)
			count = count+1
		rospy.sleep(0.1)
	p.disconnect()
