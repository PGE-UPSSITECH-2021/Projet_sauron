#!/usr/bin/env python

import rospy
from motoman_hc10_moveit_config.msg import IHM_msg

import node_test as ndt

global ihm_data

ihm_msg = None

def shutdown():
	rospy.loginfo('%s is shutting down', rospy.get_caller_id())

def callback_ihmn(data):
	global ihm_msg
	rospy.loginfo(rospy.get_caller_id() + str(data))
	ihm_msg = data

def get_action(msg):
	if msg is not None:
		return msg.action

# To write into a config file, loaded via json.load()
TOPICS = {'IHM': {'name': 'ihm_topic', 'datatype': IHM_msg, 'callback': callback_ihmn}}

# Commands
'''
S_LOC : activate / reset location service
S_ID : activate / reset identification service
S_QUAL : activate / reset quality service
CLEAR : clear loc_ok, id_ok, qual_ok
PAUSE : pause process
SHUTDOWN : stop node

'''

def run():

	global ihm_msg

	'''

	###########################################

	def service_templace(func, *args, **kwargs):

		global ihm_msg
		finished = pause = False

		while not finished:

			# Extract command from ihm topic
			command = extract_data(ihm_msg)

			# Switch pause
			if command == 'PAUSE':
				pause = not pause

			# Case pause / not pause
			if not pause:
				finished, result = func()

			else:
				pass

			ihm_msg = None

			rospy.sleep(1)

		return result

	###########################################

	def test_pause(*args, **kwargs):

		global ihm_msg
		finished = pause = False

		rospy.init_node(name='TestPause', log_level=rospy.INFO, disable_signals=True)
		while not rospy.is_shutdown() and not finished:
			rospy.loginfo('Doing stuff ...')
			rospy.sleep(1)

			if finished:
				rospy.signal_shutdown('Shutdown: finished %s, pause %s', finished, pause)

			command = extract_data(ihm_msg)

			if command == 'END':
				finished = True

			elif command == 'PAUSE':
				pause = not pause

			rospy.loginfo('Internal state: finished %s, pause %s', finished, pause)

			ihm_msg = None

	###########################################

	'''

	all_sys_ok = calib_ok = loc_ok = id_ok = qual_ok = False

	rospy.init_node(name='RobotMoverController', log_level=rospy.INFO, disable_signals=True)

	for topic, params in TOPICS.items():
		rospy.Subscriber(params['name'], params['datatype'], params['callback'])

	rate = rospy.Rate(1)

	while not rospy.is_shutdown():

		command = None

		print(all_sys_ok, calib_ok, loc_ok, id_ok, qual_ok)
		
		# check if everything is fine
		while not all_sys_ok:
			# all_sys_ok = service_templace(dummy_check)
			# all_sys_ok = dummy_check()
			all_sys_ok = ndt.dummy_check()

		# launch calibration
		while not calib_ok:
			calib_ok = ndt.dummy_calibration()

		# Get message from ihm
		command = get_action(ihm_msg)

		###########################################

		def run_location(loc_ok, id_ok, qual_ok):

			# Reset all states
			loc_ok = id_ok = qual_ok = False

			# Run loc service and get loc state
			loc_ok = ndt.dummy_location()

			# Return states
			return loc_ok, id_ok, qual_ok

		def run_identification(loc_ok, id_ok, qual_ok):

			# Run loc service if previous loc state reset
			while not loc_ok:
				loc_ok, id_ok, qual_ok = run_location(loc_ok, id_ok, qual_ok)

			# Reset id, qual states
			id_ok = qual_ok = False

			# Check states conditions to run id
			assert loc_ok

			# Run id service and get id state
			id_ok = ndt.dummy_identification()

			# Return states
			return loc_ok, id_ok, qual_ok

		def run_quality(loc_ok, id_ok, qual_ok):

			# Run id service if previous id state reset
			# Will run loc service if previous loc state reset
			while not id_ok:
				loc_ok, id_ok, qual_ok = run_identification(loc_ok, id_ok, qual_ok)

			# Reset qual state
			qual_ok = False

			# Check states conditions to run qual
			assert loc_ok, id_ok

			# Run qual service and get qual state
			qual_ok = ndt.dummy_quality()

			# Return states
			return loc_ok, id_ok, qual_ok


		# Debug
		if command is not None:
			print(command)

		# Location service
		if command == 'S_LOC':
			loc_ok, id_ok, qual_ok = run_location(loc_ok, id_ok, qual_ok)

		# Identification service
		elif command == 'S_ID':
			loc_ok, id_ok, qual_ok = run_identification(loc_ok, id_ok, qual_ok)

		# Quality service
		elif command == 'S_QUAL':
			loc_ok, id_ok, qual_ok = run_quality(loc_ok, id_ok, qual_ok)

		# Pause service
		elif command == 'PAUSE':
			pass

		elif command is None:
			pass

		else:
			print(all_sys_ok, calib_ok, loc_ok, id_ok, qual_ok)
			print(command, type(command))
			raise NotImplementedError

		ihm_msg = None

		rate.sleep()

	rospy.spin()

if __name__ == '__main__':

	try:
		run()

	except rospy.ROSInterruptException as e:
		print(e)