#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

#####################################################################################################################
# 구현한 함수
# rgb_image_export : Sensor class 매서드, rgb cam image 추출 (numpy 형식)
# depth_image_export : Sensor class 매서드, depth cam image 추출 (numpy 형식)
# vehicle_control : 전역 함수, vehicle control (throttle, steer, brake)

# class
# Sensor Manager : sensor들 모아서 관리하는 용도
# Sensor : sensor 설정, 센서 값 추출 (cam)
# CustomTimer : sync 맞추는 용도(?)

# 참고한 코드
# visualize_multiple_sensors.py
# manual_control
#####################################################################################################################

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import random
import time
import numpy as np

# cam image size
IMAGE_SIZE_X = 800
IMAGE_SIZE_Y = 600


#####################################################################################################################
# class
#####################################################################################################################


class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()
        
class SensorManager:
    def __init__(self):
        self.sensor_list = []

    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)

    def get_sensor_list(self):
        return self.sensor_list

    def destroy(self):
        for s in self.sensor_list:
            s.destroy()

class Sensor:
    def __init__(self, world, sensor_man, sensor_type, transform, attached, sensor_options):
        self.world = world
        self.sensor_man = sensor_man
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()

        self.time_processing = 0.0
        self.tics_processing = 0

        self.sensor_man.add_sensor(self)
        self.rgb_cam_image = None # image 추출용
        self.depth_cam_image = None # image 추출용

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', str(IMAGE_SIZE_X))
            camera_bp.set_attribute('image_size_y', str(IMAGE_SIZE_Y))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)

            return camera

        elif sensor_type == 'DepthCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
            camera_bp.set_attribute('image_size_x', str(IMAGE_SIZE_X))
            camera_bp.set_attribute('image_size_y', str(IMAGE_SIZE_Y))

            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])

            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_depth_image)

            return camera
        
        else:
            return None

    def get_sensor(self):
        return self.sensor

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        # carla image type -> numpy
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        
        self.rgb_cam_image = array # RGB image

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    def save_depth_image(self, image):
        t_start = self.timer.time()

        # carla image type -> numpy
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        
        self.depth_cam_image = array # Depth image

        # meter 변환법
        # self.normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
        # self.in_meters = 1000 * normalized

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1

    # RGBCamera image 반환 함수 (data: numpy)
    def rgb_image_export(self):
        return self.rgb_cam_image

    # DepthCamera image 반환 함수 (data: numpy)
    def depth_image_export(self):
        return self.depth_cam_image

    def destroy(self):
        self.sensor.destroy()
        
        
#####################################################################################################################
# global function
#####################################################################################################################
        
        
def vehicle_control(vehicle, throttle, steer, brake = 0.0, reverse = False):
    _control = carla.VehicleControl()
    
    _control.throttle = throttle    # 0.0 ~ 1.0
    _control.steer = steer          # -1.0 ~ 1.0
    _control.brake = brake          # 0.0 ~ 1.0
    _control.reverse = reverse      # True/False
    
    vehicle.apply_control(_control)
    

def run_simulation(args, client):
    
    sensor_manager = None
    vehicle = None
    vehicle_list = []
    timer = CustomTimer()

    try:

        # Getting the world and
        world = client.get_world()
        original_settings = world.get_settings()

        if args.sync:
            traffic_manager = client.get_trafficmanager(8000)
            settings = world.get_settings()
            traffic_manager.set_synchronous_mode(True)
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)


        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter('charger_2020')[0]
        vehicle = world.spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
        vehicle_list.append(vehicle)
        
        vehicle_control(vehicle_list[0], 0.5, 0.5) # vehicle_control 함수 테스트용
        # vehicle.set_autopilot(True) # 활성화 시 트랙 알아서 돌음

        # cam_image 추출용, front, rear, right, left cam 순서
        rgb_cam_image = [None, None, None, None]
        depth_cam_image = [None, None, None, None]

        # sensorplay Manager organize all the sensors an its sensorplay in a window
        # If can easily configure the grid and the total window size
        sensor_manager = SensorManager()

        # Then, SensorManager can be used to spawn RGBCamera, LiDARs and SemanticLiDARs as needed
        # and assign each of them to a grid position, 
        Sensor(world, sensor_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), vehicle, {})
        Sensor(world, sensor_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)), vehicle, {})
        Sensor(world, sensor_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)), vehicle, {})
        Sensor(world, sensor_manager, 'RGBCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)), vehicle, {})
        
        Sensor(world, sensor_manager, 'DepthCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+00)), vehicle, {})
        Sensor(world, sensor_manager, 'DepthCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=180)), vehicle, {})
        Sensor(world, sensor_manager, 'DepthCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=+90)), vehicle, {})
        Sensor(world, sensor_manager, 'DepthCamera', carla.Transform(carla.Location(x=0, z=2.4), carla.Rotation(yaw=-90)), vehicle, {})
        

        #Simulation loop
        call_exit = False
        time_init_sim = timer.time()
        while True:
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()

            # image 추출 테스트
            for idx, cam in enumerate(sensor_manager.sensor_list):
                if idx < 4:
                    rgb_cam_image[idx] = cam.rgb_image_export()
                else:
                    depth_cam_image[idx-4] = cam.depth_image_export()
                    
            if call_exit:
                break

    finally:
        if sensor_manager:
            sensor_manager.destroy()

        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])

        world.apply_settings(original_settings)


#####################################################################################################################
# main
#####################################################################################################################


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Sensor tutorial')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--async',
        dest='sync',
        action='store_false',
        help='Asynchronous mode execution')
    argparser.set_defaults(sync=True)
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(5.0)

        run_simulation(args, client)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
