#!/usr/bin/python
import sys
import yaml
import numpy
import rospkg
import pyquaternion


def main():
	package_path = rospkg.RosPack().get_path('sparse_dynamic_calibration')
	filename = package_path + '/data/tag_camera_poses.yaml'

	with open(filename, 'r') as f:
		data = yaml.load(f.read().replace('YAML:', 'YAML '))
		cameras = data['cameras']

		def mul_vectors(vec1, vec2):
			t1 = numpy.float32(vec1[:3])
			q1 = pyquaternion.Quaternion(vec1[6], vec1[3], vec1[4], vec1[5])
			t2 = numpy.float32(vec2[:3])
			q2 = pyquaternion.Quaternion(vec2[6], vec2[3], vec2[4], vec2[5])

			t = t1 + q1.rotate(t2)
			q = (q1 * q2)

			return list(t) + list(q.vector) + [q.real]

		def inverse(vec):
			t = numpy.float32(vec[:3])
			q = pyquaternion.Quaternion(vec[6], vec[3], vec[4], vec[5])

			inv_q = q.inverse
			inv_t = q.inverse.rotate(-t)

			return list(inv_t) + list(inv_q.vector) + [inv_q.real]

		tag0 = data['tags']['tag_0']
		to_world = inverse(tag0['pose'])
		to_world = mul_vectors([0, 0, 0, 1, 0, 0, 0], to_world)

		cameras = [(camera_name, cameras[camera_name]['pose']) for camera_name in cameras]
		cameras = [(x[0], mul_vectors(to_world, x[1])) for x in cameras]
		cameras = [(x[0], x[1], inverse(x[1])) for x in cameras]
		cameras = sorted(cameras, key=lambda x: x[0])

	opt_package_path = rospkg.RosPack().get_path('opt_calibration')
	# opt_package_path = '/tmp'
	calibration_results_filename = opt_package_path + '/launch/opt_calibration_results.launch'
	camera_poses_filename = opt_package_path + '/conf/camera_poses.yaml'

	def print_pose(vec, file):
		print >> file, '    translation:'
		print >> file, '      x: %f' % vec[0]
		print >> file, '      y: %f' % vec[1]
		print >> file, '      z: %f' % vec[2]
		print >> file, '    rotation:'
		print >> file, '      w: %f' % vec[6]
		print >> file, '      x: %f' % vec[3]
		print >> file, '      y: %f' % vec[4]
		print >> file, '      z: %f' % vec[5]

	def print_camera_poses(cameras, file):
		print >> file, 'calibration_id: 0'
		print >> file
		print >> file, 'poses:'
		for camera in cameras:
			print >> file, '  %s:' % camera[0]
			print_pose(camera[1], file)
		print >> file
		print >> file, 'inverse_poses:'
		for camera in cameras:
			print >> file, '  %s:' % camera[0]
			print_pose(camera[2], file)

	with open(camera_poses_filename, 'w') as f:
		print_camera_poses(cameras, f)
		print_camera_poses(cameras, sys.stdout)

	def print_transform_publisher(camera, file):
		print >> file
		print >> file, '  <!-- sensor: %s -->' % camera[0]
		line = '  <node pkg="tf" type="static_transform_publisher" name="%s_broadcaster" ' % camera[0]
		line += 'args="%f %f %f %f %f %f %f ' % tuple(camera[1])
		line += '/world /%s 10" />' % camera[0]
		print >> file, line

		line = '  <node pkg="tf" type="static_transform_publisher" name="%s_broadcaster_2" args="0 0 0 1.57 -1.57 0 /%s /%s_link 10" />' % (camera[0], camera[0], camera[0])
		print >> file, line

	def print_calibration_results(cameras, file):
		print >> file, '<?xml version="1.0"?>'
		print >> file, '<launch>'
		for camera in cameras:
			print_transform_publisher(camera, file)
		print >> file, '</launch>'

	with open(calibration_results_filename, 'w') as f:
		print_calibration_results(cameras, f)
		print_calibration_results(cameras, sys.stdout)


if __name__ == '__main__':
	main()
