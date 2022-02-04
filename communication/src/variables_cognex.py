from communication.msg import Image, Liste_points
from cv_bridge import CvBridge, CvBridgeError

class Variables:

	def __init__(self):
		self.pub_ok = None
		self.pub_originale = None
		self.pub_annotee = None
		self.pub_points = None
		self.bridge = CvBridge()
		
		self.originale = Image()
		self.annotee = Image()
		self.points = Liste_points()

		self.tn = None
		self.ftp = None
		self.ip = "192.168.1.100"
		self.user = 'admin'
		self.password = ''

		self.identification = False
		self.live = False
		self.opencv = 1
		self.render = True
		self.plaque = "Plate"

		self.imgNum = 0
		self.blur = 5
		self.dp = 1
		self.minDist = 150
		self.p1 = 100
		self.p2 = 30
		self.minR = 0
		self.maxR = 100
		self.gain = 1

		self.types = [(9, 13), (14, 19), (25, 31), (40, 47)]

		self.scale_percent = 0.4

	def get_type(self, rayon):
		i = 1
		for dim in self.types:
			if(rayon>= dim[0] and rayon<=dim[1]):
				return i
			i += 1
		return -1
