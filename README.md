# vehicle_control_and_image_yonsei.py

- 구현한 함수<br/><br/>
rgb_image_export : Sensor class 매서드, rgb cam image 추출 (numpy 형식)<br/>
depth_image_export : Sensor class 매서드, depth cam image 추출 (numpy 형식)<br/>
vehicle_control : 전역 함수, vehicle control (throttle, steer, brake)<br/>
<br/>
- class<br/><br/>
Sensor Manager : sensor들 모아서 관리하는 용도<br/>
Sensor : sensor 설정, 센서 값 추출 (cam)<br/>
CustomTimer : sync 맞추는 용도(?)<br/>
<br/>
- 참고한 코드<br/><br/>
visualize_multiple_sensors.py<br/>
manual_control<br/>
<br/>
pygame 이용해 이미지들 시각화도 가능합니다. -> visualize_multiple_sensors.py
<br/><br/>

# visualize_multiple_sensors_yonsei.py

pygame 이용해 rgb 카메라 4개, depth 카메라 4개를 시각화 합니다.<br/>
visualize_multiple_sensors.py 예제 참고했습니다.<br/>
카메라는 차례대로 전, 후, 우, 좌 순서입니다.<br/>
vehicle_control 함수는 없습니다.
<br/><br/>

# cars_spawn_and_location_reset.py

- 구현한 함수<br/>
two_vehicle_random_spawner : 차량 두 대, 위치 랜덤 리스폰 함수 <br/>
vehicle_reset: 차량 위치 리셋 함수
<br/><br/>
