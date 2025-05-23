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
        cfg = test_parameters.cfg
        model = parsingNet(pretrained=False, backbone='18',
                           cls_dim=(cfg.num_lanes, cfg.num_row, cfg.num_col),
                           use_aux=False).eval()
        
        model_path = os.path.join(os.path.dirname(__file__),
                                  'Ultra-Fast-Lane-Detection', 'output', 'tusimple_18.pth')
        model.load_state_dict(torch.load(model_path, map_location='cpu'), strict=False)
        
        transform = transforms.Compose([
            transforms.Resize((288,800)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485,0.456,0.406],
                                 std=[0.229,0.224,0.225])
        ])
        return model, transforms
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        input_tensor = self.transform(PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))).unsqueeze(0).to(device)
        
        with torch.no_grad():
            out = self.model(input_tensor)[0].cpu().numpy()
            
        self.draw_lanes(frame, out)
        
    def draw_lanes(self, image, lane_data):
        h, w, _ = image.shape
        row_anchor = [64, 96, 128, 160, 192, 224, 256]
        color_set = [(255, 0, 0), (0, 255, 0), (0, 255, 255), (255, 0, 255)]
        
        for lane_idx, lane in enumerate(lane_data):
            xs = lane[0]
            ys = lane[1]
            
            points = []
            for x, y in zip(xs, ys):
                if x > 0:
                    x = int(x * w /800)
                    y = int(y * h /288)
                    points.append((x, y))
                    
            for j in range(1, len(points)):
                cv2.line(image, points[j - 1], points[j], color_set[lane_idx % 4], 2)
                
        cv2.imshow('Lane Detection', image)
        cv2.waitKey(1)