#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

#####################################################################################################################
# 구현한 함수
# 
# two_vehicle_random_spawner : 차량 두 대, 위치 랜덤 리스폰 함수
# vehicle_reset: 차량 위치 리셋 함수
#
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
        
        
#####################################################################################################################
# global function
#####################################################################################################################


def two_vehicle_random_spawner(world, bp, vehicle_list):
    vehicle_1_spawn_transform = random.choice(world.get_map().get_spawn_points())
    vehicle_2_spawn_transform = random.choice(world.get_map().get_spawn_points())
    
    vehicle_1 = world.spawn_actor(bp, vehicle_1_spawn_transform)
    vehicle_list.append(vehicle_1) # shallow copy
    
    vehicle_2 = world.spawn_actor(bp, vehicle_2_spawn_transform)
    vehicle_list.append(vehicle_2) # shallow copy
    
    return vehicle_1, vehicle_2, vehicle_1_spawn_transform, vehicle_2_spawn_transform


# Transform 데이터 타입: carla.Transform -> carla.Location, carla.Rotation 포함
#   -> set_transform 안에 초기 location과 rotation 설정해주면 될 듯합니다.
def vehicle_reset(vehicle, transform): # transform : 차량 리셋할 위치
    vehicle.set_transform(transform)
    

def run_simulation(args, client):
    
    vehicle_1 = None
    vehicle_2 = None
    vehicle_1_spawn_transform = None
    vehicle_2_spawn_transform = None
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


        '''
        The map object contains recommended spawn points for the creation of vehicles. 
        You can get a list of these spawn points, each one containing a carla.
        Transform, using the method below. 
        Bear in mind that the spawn points may be occupied already, resulting in failed creation of vehicles due to collisions.
        
        참고: https://carla.readthedocs.io/en/latest/core_map/
        '''
        # Instanciating the vehicle to which we attached the sensors
        bp = world.get_blueprint_library().filter('charger_2020')[0]

        # 차량 두 대 랜덤 리스폰
        vehicle_1, vehicle_2, vehicle_1_spawn_transform, vehicle_2_spawn_transform = two_vehicle_random_spawner(world, bp, vehicle_list)
        
        # Vehicle autopilot mode
        vehicle_1.set_autopilot(True) 
        vehicle_2.set_autopilot(True)
        
        # Simulation loop
        call_exit = False
        time_init_sim = timer.time()
        while True:
            # Carla Tick
            if args.sync:
                world.tick()
            else:
                world.wait_for_tick()
    
            if call_exit:
                break
            
            vehicle_reset(vehicle_1, vehicle_1_spawn_transform) # vehicle_reset 함수 대충 테스트 해봤는데 vehicle_1은 계속 제자리에서 부들거리는 것을 보니 잘 되는 듯합니다. 

    finally:

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
