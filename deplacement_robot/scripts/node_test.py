#!/usr/bin/env python

import rospy

def dummy_check(*arg, **kwargs):
	print(arg)
	print(kwargs)
	print('[CHECK] %s', __name__)
	
	return True

def dummy_calibration(*arg, **kwargs):
	print(arg)
	print(kwargs)
	print('[CALIBRATION] %s', __name__)

	return True

def dummy_location(*arg, **kwargs):
	print(arg)
	print(kwargs)
	print('[LOCATION] %s', __name__)

	return True

def dummy_identification(*arg, **kwargs):
	print(arg)
	print(kwargs)
	print('[IDENTIFICATION] %s', __name__)

	return True

def dummy_quality(*arg, **kwargs):
	print(arg)
	print(kwargs)
	print('[QUALITY] %s', __name__)

	return True

if __name__ == '__main__':
	try:
		pass
	except rospy.ROSInterruptException as e:
		print(e)