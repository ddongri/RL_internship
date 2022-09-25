# RL_internship


구현한 함수<br/><br/>
rgb_image_export : Sensor class 매서드, rgb cam image 추출 (numpy 형식)<br/>
depth_image_export : Sensor class 매서드, depth cam image 추출 (numpy 형식)<br/>
vehicle_control : 전역 함수, vehicle control (throttle, steer, brake)<br/>
<br/><br/>
class<br/><br/>
Sensor Manager : sensor들 모아서 관리하는 용도<br/>
Sensor : sensor 설정, 센서 값 추출 (cam)<br/>
CustomTimer : sync 맞추는 용도(?)<br/>
<br/><br/>
참고한 코드<br/><br/>
visualize_multiple_sensors.py<br/>
manual_control<br/>
<br/><br/>
pygame 이용해 이미지들 시각화도 가능합니다. -> visualize_multiple_sensors.py
