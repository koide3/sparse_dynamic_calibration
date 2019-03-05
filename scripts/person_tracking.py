#!/usr/bin/python
import tf
import cv2
import numpy
import pyquaternion
from matplotlib import pyplot

import rospy
import cv_bridge
import message_filters
from opt_msgs.msg import *
from tfpose_ros.msg import *
from sensor_msgs.msg import *


class PersonTrackingNode:
	def __init__(self):
		self.image = numpy.zeros((256, 256, 3), dtype=numpy.uint8)
		self.cv_bridge = cv_bridge.CvBridge()
		self.tf_listener = tf.TransformListener()

		self.tracks = numpy.loadtxt('data/tracks.csv')

		self.trajectory = []
		self.camera_trajectory = []
		self.image_pub = rospy.Publisher('image', Image, queue_size=10)

		self.camera_info_sub = message_filters.Subscriber('/camera/camera_info', CameraInfo)
		self.image_sub = message_filters.Subscriber('/camera/image_raw', Image)
		self.pose_sub = message_filters.Subscriber('/pose_estimator/pose', Persons)

		self.sync = message_filters.TimeSynchronizer([self.camera_info_sub, self.image_sub, self.pose_sub], 30)
		self.sync.registerCallback(self.callback)

	def callback(self, camera_info_msg, image_msg, pose_msg):
		image = self.cv_bridge.imgmsg_to_cv2(image_msg)
		canvas = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

		leg_pos = []
		for person in pose_msg.persons:
			heights = [part.y * pose_msg.image_h for part in person.body_part]
			height = max(heights) - min(heights)
			if height < 500:
				continue
			print 'height:', height

			lines = [(0, 1), (1, 2), (2, 3), (3, 4), (1, 5), (5, 6), (6, 7), (1, 8), (8, 9), (9, 10), (1, 11), (11, 12), (12, 13)]

			for line in lines:
				pt0 = [x for x in person.body_part if x.part_id == line[0]]
				pt1 = [x for x in person.body_part if x.part_id == line[1]]

				if not len(pt0) or not len(pt1):
					continue

				pt0 = int(pt0[0].x * pose_msg.image_w), int(pt0[0].y * pose_msg.image_h)
				pt1 = int(pt1[0].x * pose_msg.image_w), int(pt1[0].y * pose_msg.image_h)
				cv2.line(canvas, pt0, pt1, (255, 0, 0), 5)

			for part in person.body_part:
				cv2.circle(canvas, (int(part.x * pose_msg.image_w), int(part.y * pose_msg.image_h)), 10, (0, 0, 255), -1)

			right_knee = [x for x in person.body_part if x.part_id == 10 and x.confidence > 0.3]
			left_knee = [x for x in person.body_part if x.part_id == 13 and x.confidence > 0.3]

			if len(right_knee) or len(left_knee):
				n = 0
				pos = numpy.float64([0, 0])
				if len(right_knee):
					n += 1
					pos += numpy.float64([right_knee[0].x, right_knee[0].y])
				if len(left_knee):
					n += 1
					pos += numpy.float64([left_knee[0].x, left_knee[0].y])

				pos = pos * numpy.float64([pose_msg.image_w, pose_msg.image_h]) / n
				cv2.circle(canvas, tuple(numpy.int32(pos)), 20, (255, 255, 255), -1)

				leg_pos.append(pos)

		camera_matrix = numpy.float64(camera_info_msg.K).reshape(3, 3)
		distortion = numpy.float64(camera_info_msg.D).flatten()

		undistorted = cv2.undistort(canvas, camera_matrix, distortion)

		try:
			self.tf_listener.waitForTransform('world', image_msg.header.frame_id, image_msg.header.stamp, rospy.Duration(2.0))
			trans = self.tf_listener.lookupTransform('world', image_msg.header.frame_id, image_msg.header.stamp)
			inv_trans = self.tf_listener.lookupTransform(image_msg.header.frame_id, 'world', image_msg.header.stamp)
		except:
			print 'failed to lookup'
			return

		self.camera_trajectory.append(trans[0])

		if len(leg_pos):
			leg_pos = numpy.float32(leg_pos).reshape(1, -1, 2)
			undistorted_leg_pos = cv2.undistortPoints(leg_pos, camera_matrix, distortion, P=camera_matrix)
			cv2.circle(undistorted, tuple(numpy.int32(undistorted_leg_pos[0, 0, :])), 40, (0, 255, 0), -1)

			R = pyquaternion.Quaternion(scalar=inv_trans[1][3], vector=inv_trans[1][:3]).rotation_matrix
			t = inv_trans[0]

			A_inv = numpy.linalg.inv(camera_matrix)
			R_inv = numpy.linalg.inv(R)

			uv1 = numpy.float64([undistorted_leg_pos[0, 0, 0], undistorted_leg_pos[0, 0, 1], 1])

			suv1 = R_inv.dot(A_inv.dot(uv1))
			rinv_t = R_inv.dot(t)

			s = rinv_t[2] / suv1[2]

			xyz = s * suv1 - rinv_t
			print s, xyz

			self.trajectory.append(xyz)

		output_msg = self.cv_bridge.cv2_to_imgmsg(undistorted, 'bgr8')
		self.image_pub.publish(output_msg)

	def spin(self):
		rospy.sleep(0.1)
		if not len(self.trajectory):
			return

		camtraj = numpy.float64(self.camera_trajectory)
		traj = numpy.float64(self.trajectory)
		pyplot.clf()
		pyplot.scatter(camtraj[:, 0], camtraj[:, 1])
		pyplot.scatter(traj[:, 0], traj[:, 1])
		pyplot.scatter(self.tracks[:, 0], self.tracks[:, 1])
		pyplot.legend()
		pyplot.axis('equal')
		pyplot.pause(0.1)

		self.save()

	def save(self):
		if len(self.camera_trajectory):
			camera_trajectory = numpy.float64(self.camera_trajectory)
			trajectory = numpy.float64(self.trajectory)
			numpy.savetxt('data/camera_trajectory.csv', camera_trajectory)
			numpy.savetxt('data/trajectory.csv', trajectory)


class OPTTrackRecorder:
	def __init__(self):
		self.tracks = []
		self.ids = [686]
		self.sub = rospy.Subscriber('/tracker/tracks_smoothed', TrackArray, self.callback)

	def callback(self, tracks_msg):
		for track in tracks_msg.tracks:
			if track.id not in self.ids:
				continue
			self.tracks.append((track.x, track.y))

	def spin(self):
		if not len(self.tracks):
			return

		tracks = numpy.float64(self.tracks)
		pyplot.clf()
		pyplot.scatter(tracks[:, 0], tracks[:, 1])
		pyplot.axis('equal')
		pyplot.pause(0.1)

	def save(self):
		if len(self.tracks):
			numpy.savetxt('data/tracks.csv', self.tracks)


class Plot:
	def __init__(self):
		self.tracks = numpy.loadtxt('data/tracks.csv')
		self.trajectory = numpy.loadtxt('data/trajectory.csv')
		self.camtraj = numpy.loadtxt('data/camera_trajectory.csv')

		traj = []
		self.trajectories = []
		for pt0, pt1 in zip(self.trajectory, self.trajectory[1:]):
			if numpy.linalg.norm(pt0 - pt1) > 1.8:
				self.trajectories.append(traj)
				traj = []
			traj.append(pt1)
		self.trajectories.append(traj)

		for i in range(len(self.trajectories)):
			traj = self.trajectories[i]
			avged = []
			for j in range(len(traj) - 5):
				mean = numpy.mean(traj[j:j+5], axis=0)
				avged.append(mean)

			self.trajectories[i] = avged

	def spin(self):
		pyplot.figure(figsize=(6, 3))
		# pyplot.plot(self.trajectory[:, 0], self.trajectory[:, 1], label='track(moving camera)')
		labeled = False
		for traj in self.trajectories:
			if not len(traj):
				continue

			traj = numpy.float64(traj)
			# pyplot.plot(traj[:, 0], traj[:, 1], c='orange', label='track(moving camera)')
			if not labeled:
				labeled = True
				pyplot.scatter(traj[:, 0], traj[:, 1], s=7.0, c='orange', label='track(moving camera)')
			else:
				pyplot.scatter(traj[:, 0], traj[:, 1], s=7.0, c='orange')

		pyplot.scatter(self.tracks[:, 0], self.tracks[:, 1], s=3.0, label='track(static camera)')

		pyplot.xlabel('X [m]')
		pyplot.ylabel('Y [m]')
		pyplot.xlim(xmin=-2, xmax=12)
		pyplot.ylim(ymin=-7, ymax=3)
		pyplot.legend()
		pyplot.savefig('trajectory.svg')
		pyplot.show()

		pyplot.figure(figsize=(6, 3))
		pyplot.plot(self.camtraj[:, 0], self.camtraj[:, 1], c='red', label='camera trajectory')
		pyplot.xlabel('X [m]')
		pyplot.ylabel('Y [m]')
		pyplot.xlim(xmin=-2, xmax=12)
		pyplot.ylim(ymin=-7, ymax=3)
		pyplot.legend()

		pyplot.savefig('camera_trajectory.svg')
		pyplot.show()
		exit(0)

	def save(self):
		pass


def main():
	rospy.init_node('person_tracking')
	node = Plot()

	while not rospy.is_shutdown():
		node.spin()

	node.save()


if __name__ == '__main__':
	main()
