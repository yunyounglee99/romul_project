## 소개

> 돌고돌아 정착한 자율주행대회,,, 그러나 정작 놓쳐버린 예선기한ㅠㅠ
> 

## 목표

아두이노 자동차를 제어하는 vlm 모델 개발 → 자율주행 대회 출전

- 아두이노 자동차 제작
- vlm 모델 설계
- 자율주행 대회 출전

## 데이터셋

## 모델링

제공된 unity 시뮬레이션 연동 파이썬 파일을 수정하여 조건을 충족하도록 도로 주행 시스템 개발

> 차량 주행 시뮬레이터 안에 있는 주행트랙에서 차선을 따라 안정적이고 정확하게 주행하는 자율 주행 SW 구현
> 
1. **camera_detector.py**
    - input : 카메라 이미지 센서
    - output : detected_objects
    - 세부사항
        - YOLO로 이미지 분석
        - 차선 감지
        - 신호등 감지

1. **lidar_preprocessing.py**
    - input : lidar points
    - ouput : clustered obstacle
    - 세부사항
        - 회피해야할 차량 및 라바콘 인식

1. **decision_maker.py**
    - input : sensors outputs
    - output : decision signs(=cmd_type) (STOP, AVOID, NORMAL 등)
    - 세부사항
        - 신호등 행동 판단
        - 장애물 행동 판단
        - 차량 회피 판단

1. **path_planner.py (3과 통합)**
    - input : sensors outputs, cmd_type
    - output : path
    - 세부사항
        - 라바콘 주행 판단 : 1순위 (라바콘 주행 true/false 받기)
        - 차도 주행 판단 : 2순위

1. **motion_controller.py**
    - input : path or cmd_type
    - output : motions
    - 세부사항
        - 각 cmd_type 또는 Path에 맞는 속도 및 조향 제어

## 훈련 및 평가

*실제 구현은 완성하지 못하여 제대로 된 성능을 평가하지 못하였습니다,,,,,*

## 결과

![image.png](attachment:aa48cdd1-a6dc-4876-9b4c-1f0ac1813c48:image.png)

## 한계 분석

- 처음 목표를 설정할때 생각보다 너무 어려운 것을 잡았다. 다음에는 실천 가능성에 대해서 면밀히 조사하자
- 하드웨어는 매우 어렵다 특히 용량은 항상 넉넉히 준비하자
- 목표가 더디게 진행된다면 빠른 시일내에 다른 방법을 모색하자 기존 목표에 너무 욕심 부리지 말자

## 후속 프로젝트

로봇 시뮬레이션 프로젝트 재도전
