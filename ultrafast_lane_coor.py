#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

import rospy
import cv2
import torch
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from torchvision import transforms
from PIL import Image as PILImage

# ───────────────────────────────────────────────────────────────────
# (1) Ultra-Fast-Lane-Detection 폴더를 Python 모듈 경로에 추가합니다.
#     이 경로가 실제 폴더명과 정확히 일치해야 합니다.
sys.path.append(
    os.path.join(os.path.dirname(__file__), "Ultra-Fast-Lane-Detection")
)

# (2) UltraFast 모델 정의 및 설정값 가져오기
#     - model/model.py 에 parsingNet 클래스가 정의되어 있으며,
#     - configs/<dataset>.py 에 각종 하이퍼파라미터(cfg)가 정의되어 있습니다.
from model.model import parsingNet
from configs import test_parameters
# ───────────────────────────────────────────────────────────────────


class LaneCoordPublisher:
    """
    이 클래스는 다음을 수행합니다:
    1) /usb_cam/image_raw 토픽으로부터 카메라 이미지를 구독
    2) UltraFast 모델로 차선 추론 (generate_result)
    3) 각 차선의 (x_pixel, y_pixel) 좌표를 추출해서
       Float32MultiArray 메시지로 /lane_coords 토픽에 퍼블리시
    """

    def __init__(self):
        # 1) ROS 노드 초기화
        rospy.init_node("ultrafast_lane_coords_node", anonymous=False)
        self.bridge = CvBridge()

        # 2) /usb_cam/image_raw 이미지 토픽 구독
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw", Image, self.image_callback, queue_size=1
        )

        # 3) /lane_coords Float32MultiArray 퍼블리시
        #    Float32MultiArray.data 에 [lane0_x1, lane0_y1, lane0_x2, lane0_y2, …] 식으로 저장
        self.lane_pub = rospy.Publisher(
            "/lane_coords", Float32MultiArray, queue_size=1
        )

        # 4) UltraFast 모델 로드
        self.model, self.transform = self.load_model()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)
        
        cfg = test_parameters.cfg
        self.row_anchor = cfg.row_anchor

        rospy.loginfo("✅ UltraFast Lane Coord Publisher Ready")

    def load_model(self):
        """
        UltraFast 모델을 초기화하고, 사전학습된 가중치를 로드합니다.
        반환값:
          - model: parsingNet 인스턴스 (eval 모드)
          - transform: 이미지 전처리용 torchvision.transforms.Compose
        """
        # 1) test_parameters.cfg 안에 num_lanes, num_row, num_col, dataset 등이 정의되어 있음
        cfg = test_parameters.cfg

        # 2) parsingNet 생성
        model = parsingNet(
            pretrained=False,
            backbone=cfg.backbone,                           # 예: '18'
            cls_dim=(cfg.griding_num + 1, cfg.cls_num_per_lane, cfg.num_lanes),
            use_aux=False,
        ).eval()

        # 3) .pth 가중치 경로 설정
        #    (예: "Ultra-Fast-Lane-Detection/output/culane_18.pth")
        model_path = os.path.join(
            os.path.dirname(__file__),
            "Ultra-Fast-Lane-Detection",
            "output",
            os.path.basename(cfg.test_model),
        )
        if not os.path.isfile(model_path):
            rospy.logerr(f"❌ Cannot find model file: {model_path}")
            rospy.signal_shutdown("Model file missing")
            return None, None

        # 4) state_dict 로드 (DataParallel에서 저장된 경우 'module.' 접두어 제거)
        raw_state = torch.load(model_path, map_location="cpu")
        if "model" in raw_state:
            state_dict = raw_state["model"]
        else:
            state_dict = raw_state

        compatible_state = {}
        for k, v in state_dict.items():
            if k.startswith("module."):
                compatible_state[k[7:]] = v
            else:
                compatible_state[k] = v

        model.load_state_dict(compatible_state, strict=False)

        # 5) 이미지 전처리 (resize → tensor → normalize)
        transform = transforms.Compose(
            [
                transforms.Resize((288, 800)),
                transforms.ToTensor(),
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
                ),
            ]
        )
        return model, transform

    def image_callback(self, msg):
        """
        카메라 이미지 콜백: 
        1) ROS Image → OpenCV BGR 이미지 변환
        2) UltraFast 모델 추론 → generate_result → lane_data
        3) 좌표 리스트 flatten → Float32MultiArray 퍼블리시
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge error: %s" % str(e))
            return

        # 1) 이미지 전처리: OpenCV(BGR) → PIL(RGB) → Tensor → Device
        pil_img = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        input_tensor = self.transform(pil_img).unsqueeze(0).to(self.device)

        # 2) UltraFast 추론
        with torch.no_grad():
            raw_out = self.model(input_tensor)[0].cpu().numpy()
            # raw_out.shape = (griding_num+1, num_row, num_col)

        # 3) 후처리: raw_out → lane_data 형태로 변환
        #    lane_data: [(xs, ys), …]  len = num_lanes
        lane_data = self.generate_result(raw_out)

        # 4) (x, y) 좌표를 flatten해서 Float32MultiArray 로 퍼블리시
        flattened = self.flatten_lane_data(lane_data, frame.shape)
        msg_out = Float32MultiArray(data=flattened)
        self.lane_pub.publish(msg_out)
        
        anchor_vis = frame.copy()
        h, w = frame.shape[:2]
        for y in self.row_anchor:
            py = int(y * h /288.0)
            cv2.line(anchor_vis, (0, py), (w, py), (255, 0, 0), 1)
        cv2.imshow("Anchors", anchor_vis)
        cv2.waitKey(2)

    def generate_result(self, out):
        """
        UltraFast 모델 raw 출력 out을 “(num_lanes, 2, num_row)” 형태로 변환합니다.
        Args:
          - out: numpy array, shape = (griding_num+1, num_row, num_col)
        Returns:
          - lanes: list of tuples (xs, ys), 길이 = num_lanes
            · xs: length = num_row, “행(row_anchor)마다 예측된 x 픽셀(0 ~ 800 기준)”
            · ys: length = num_row, “row_anchor 픽셀 높이(288 기준)”
        """
        cfg = test_parameters.cfg
        griding_num = cfg.griding_num         # ex) 200
        num_row = cfg.cls_num_per_lane        # ex) 7(CULane) or 56(TuSimple)
        num_lanes = cfg.num_lanes             # ex) 4
        row_anchor = cfg.row_anchor           # ex) [64,96,128,160,192,224,256] or length 56

        # (1) out 배열 뒤집기: “y 인덱스(행) 순서를 뒤집어” bottom-to-top → top-to-bottom
        out = out[:, ::-1, :]  # shape = (griding_num+1, num_row, num_col)

        # (2) softmax 계산 (마지막 채널 제외)
        prob = scipy.special.softmax(out[:-1, :, :], axis=0)
        idx = np.arange(griding_num) + 1       # [1 ~ griding_num]
        idx = idx.reshape(-1, 1, 1)            # shape = (griding_num, 1, 1)

        # (3) loc: “Softmax 기반 연속 x 인덱스 값”
        loc = np.sum(prob * idx, axis=0)      # shape = (num_row, num_col)

        # (4) argmax: “각 (row, col)마다 가장 큰 로짓 인덱스”
        out_j = np.argmax(out, axis=0)        # shape = (num_row, num_col)

        # (5) 배경 인덱스(griding_num)인 경우 → 0으로 통일
        out_j[out_j == griding_num] = 0

        lanes = []
        for lane_idx in range(num_lanes):
            xs = []
            ys = []
            for i in range(num_row):
                # (6) 한 행(row=i)에 대해, “연속 x 좌표”를 구함
                x_index = int(loc[i, :] .argmax() if np.any(out_j[i, :] != 0) else 0)
                #    · loc[i, :]에서 out_j[i, :] != 0인 칸들의 인덱스 중 최대 확률 값
                #    · 만약 out_j[i, :] 모두 0이라면 “차선 없음”으로 간주하고 x_index = 0

                if x_index == 0:
                    # 이 행에 차선이 없다고 모델이 판단한 경우
                    xs.append(-1.0)
                else:
                    # “800 픽셀 기준 → 실제 800 픽셀 좌표”
                    # x_index는 [1~griding_num] 범위
                    # loc[i, x_index] 는 “continuous x 인덱스(1~griding_num)”
                    x_cont = loc[i, x_index]
                    x_pixel = x_cont * (800.0 / float(griding_num))
                    xs.append(x_pixel)
                # y 픽셀은 “288 기준 row_anchor[i]”
                ys.append(row_anchor[i])

            lanes.append((xs, ys))
        return lanes

    def flatten_lane_data(self, lane_data, image_shape):
        """
        lane_data = [(xs, ys), …], len = num_lanes
        image_shape = (height, width, channels)
        아래와 같은 1차원 리스트로 변환합니다:
          [lane0_x1, lane0_y1, lane0_x2, lane0_y2, …, laneN_xM, laneN_yM]
        """
        h, w, _ = image_shape
        num_lanes = len(lane_data)               # ex) 4
        num_row = len(lane_data[0][0])           # ex) 7 (CULane) or 56 (TuSimple)

        coords = []
        for lane_idx in range(num_lanes):
            xs, ys = lane_data[lane_idx]
            for i in range(num_row):
                x_val = xs[i]
                y_val = ys[i]

                if x_val < 0:
                    # 차선 없음 → [-1, -1]로 표시
                    coords.extend([-1.0, -1.0])
                else:
                    # (x_val, y_val)는 “800×288 기준 픽셀 좌표”이므로,
                    # 실제 카메라 이미지 해상도(w, h) 기준으로 스케일 보정
                    px = x_val * (w / 800.0)  # 실수형
                    py = y_val * (h / 288.0)  # 실수형
                    coords.extend([px, py])
        return coords


if __name__ == "__main__":
    node = LaneCoordPublisher()
    rospy.spin()
