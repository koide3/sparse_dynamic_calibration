#!/usr/bin/python
import yaml
import rospkg
import subprocess


def main():
	package_path = rospkg.RosPack().get_path('sparse_dynamic_calibration')

	with open(package_path + '/data/camera_poses.yaml', 'r') as f:
		poses = yaml.load(f.read())['poses']

		for camera_name in poses:
			trans = poses[camera_name]['translation']
			rot = poses[camera_name]['rotation']

			t = trans['x'], trans['y'], trans['z']
			q = rot['x'], rot['y'], rot['z'], rot['w']

			src_filename = package_path + '/data/%s_points.pcd' % camera_name
			dst_filename = '/tmp/%s_transformed.pcd' % camera_name

			command = ['pcl_transform_point_cloud']
			command += [src_filename, dst_filename]
			command += ['-trans', '%f,%f,%f' % t]
			command += ['-quat', '%f,%f,%f,%f' % q]

			print command
			subprocess.call(command)


if __name__ == '__main__':
	main()
