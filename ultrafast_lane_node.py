import os, sys
import rospy, cv2, torch
import numpy
from sensor_msg.msg import Image
from cv_bridge import CvBridge
from torchvision import transforms
from PIL import Image as PILImage

sys.path.append(os.path.join(os.path.dirname(__file__), "Ultra-Fast-Lane_detection"))

from model.model import parsingNet
from configs import test_parameters

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector_node', anonymous = False)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model.to(self.device)
        print('Lane Detection Model Ready')
        
    def load_model(self):