#!/usr/bin/python
import yaml
import numpy
import rospkg
import scipy.optimize
from matplotlib import pyplot


def main():
	package_path = rospkg.RosPack().get_path('opt_calibration')

	cameras = []
	with open(package_path + '/conf/camera_poses.yaml', 'r') as f:
		cameras = yaml.load(f.read().replace('YAML:', 'YAML '))['poses']
		cameras = sorted([(x, cameras[x]['translation']) for x in cameras], key=lambda x: x[0])
		cameras = [(x[0], (x[1]['x'], -x[1]['y'])) for x in cameras]
		print cameras

	estimated = numpy.float64([x[1][:2] for x in cameras])

	cam2d_pos = numpy.loadtxt('cam2d_pos') * 0.05

	x0 = [0, 0, 0]

	def trans(x):
		cos = numpy.cos(x[2])
		sin = numpy.sin(x[2])
		R = numpy.float64([[cos, -sin], [sin, cos]])
		t = x[:2]

		est = numpy.float64([R.dot(p) + t for p in estimated])
		return est

	def error(x):
		est = trans(x)
		# est = R.dot(estimated.T).T + t
		diff = est - cam2d_pos
		print diff

		err = numpy.mean(numpy.abs(diff))
		print err
		return err

	result = scipy.optimize.minimize(error, x0)
	print result

	est = trans(result.x)
	diff = numpy.linalg.norm(est - cam2d_pos, axis=1)
	print diff
	print 'mean:%.3f std:%.3f min:%.3f max:%.3f' % (numpy.mean(diff), numpy.std(diff), numpy.min(diff), numpy.max(diff))

	pyplot.scatter(est[:, 0], est[:, 1])
	pyplot.scatter(cam2d_pos[:, 0], cam2d_pos[:, 1])
	pyplot.show()


if __name__ == '__main__':
	main()
