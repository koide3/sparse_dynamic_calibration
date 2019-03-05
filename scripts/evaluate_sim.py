#!/usr/bin/python
import tf
import yaml
import numpy
import rospy
import rospkg
import scipy.optimize

import sophus
import pyquaternion


def record_camposes():
	rospy.init_node('evaluate_sim')
	tf_listener = tf.TransformListener()

	# template = 'kinect_%02d_optical_link'
	template = 'kinect_%02d'

	poses = []
	for i in range(1, 9):
		print template % i
		while True:
			try:
				tf_listener.waitForTransform('world', template % i, rospy.Time(0), rospy.Duration(5))
				break
			except:
				continue
		trans = tf_listener.lookupTransform('world', template % i, rospy.Time(0))
		pose = trans[0] + trans[1]
		print pose
		poses.append(pose)

	numpy.savetxt('camposes_cal.csv', poses)


def read_camposes():
	package_path = rospkg.RosPack().get_path('sparse_dynamic_calibration')

	with open(package_path + '/data/tag_camera_poses_refined.yaml', 'r') as f:
		data = yaml.load(f.read().replace('YAML:', 'YAML '))

		cameras = data['cameras']
		camera_poses = sorted([(x, cameras[x]['pose']) for x in cameras], key=lambda x: x[0])

		tags = data['tags']
		tag_poses = sorted([(x, tags[x]) for x in tags], key=lambda x: x[0])

	return camera_poses, tag_poses


def compare():
	camposes_gt = numpy.loadtxt('camposes_gt.csv')
	camposes_cal = read_camposes()[0]
	camposes_cal = numpy.array([x[1] for x in camposes_cal])

	def initial_guess(gt_mats, cal_mats):
		gt = numpy.array([x[:, 3] for x in gt_mats]).T
		cal = numpy.array([x[:, 3] for x in cal_mats]).T

		pseudo_cal_inv = cal.T.dot(numpy.linalg.inv(cal.dot(cal.T)))
		A = gt.dot(pseudo_cal_inv)

		R = scipy.linalg.orth(A[:3, :3])

		mat = numpy.eye(4)
		mat[:3, :3] = R
		mat[:3, 3] = A[:3, 3]

		return mat

	def pose2mat(pose):
		mat = numpy.eye(4)
		mat[:3, 3] = pose[:3]

		quat = pyquaternion.Quaternion(pose[6], pose[3], pose[4], pose[5])
		mat[:3, :3] = quat.rotation_matrix
		return mat

	def diff(lhs, rhs):
		t1 = lhs[:3, 3]
		t2 = rhs[:3, 3]
		q1 = pyquaternion.Quaternion(matrix=lhs[:3, :3])
		q2 = pyquaternion.Quaternion(matrix=rhs[:3, :3])

		dt = numpy.linalg.norm(t1 - t2)
		dq = abs((q1.inverse * q2).angle)

		return (dt, numpy.rad2deg(dq))

	def error(x):
		m = sophus.SE3.exp(x)

		err = 0
		for gt, cal in zip(gt_mats, cal_mats):
			err += sum(numpy.power(diff(gt, m.matrix().dot(cal)), 1))

		print err
		return err

	gt_mats = [pose2mat(x) for x in camposes_gt]
	cal_mats = [pose2mat(x) for x in camposes_cal]

	# x0 = numpy.zeros(6)
	initial_guess = initial_guess(gt_mats, cal_mats)
	x0 = sophus.SE3(initial_guess).log()

	result = scipy.optimize.minimize(error, x0, method='BFGS')

	m = sophus.SE3.exp(result.x)

	diffs = []
	for gt, cal in zip(gt_mats, cal_mats):
		d = diff(gt, m.matrix().dot(cal))
		diffs.append(d)
		print d

	print numpy.mean(diffs, axis=0)


if __name__ == '__main__':
	# record_camposes()
	compare()
