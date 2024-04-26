# this code references https://dkqhzm2.tistory.com/entry/CARLA-double-monitor-manualcontrol 
# as help in making the triple monitors work. I made tweaks of my own, but this link helped
# alot.
# I used this code to solve my problem, I changed the cameras to regular cameras, and then changed the
# angle of these cameras so that the 3 monitor view was more seamless

from __future__ import print_function

import argparse
from audioop import cross
from bdb import GENERATOR_AND_COROUTINE_FLAGS
import collections
import datetime
from distutils.command.build_scripts import first_line_re
from distutils.spawn import spawn
import glob
import logging
import math
import numbers
from operator import is_
import os
import numpy.random as random
import re
import sys
import weakref
import time
import cv2


try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error

# from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
# from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error
#from carla.agents.navigation.behavior_agent import BehavoirAgent

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================
walkers_list = []
all_id = []
all_actors = []
cross_walks = []
walkers = []
vehicles_list = []
VIEW_WIDTH = 1920
VIEW_HEIGHT = 1080
VIEW_FOV = 90

def find_weather_presets():
    """Method to find weather presets"""
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

def set_destinations(spawn_points):
        different_end_destinations = []
        #0
        end_destination = random.choice(spawn_points).location
        end_destination.x = 110.800049
        end_destination.y = 72.599747
        different_end_destinations.append(end_destination)

        # 1
        end_destination = random.choice(spawn_points).location
        end_destination.x = 90.432556
        end_destination.y = 12.643750
        different_end_destinations.append(end_destination)

        # 2
        end_destination = random.choice(spawn_points).location
        end_destination.x = 52.143875
        end_destination.y = 106.947296
        different_end_destinations.append(end_destination)

        # 4
        #end_destination = random.choice(spawn_points).location
        #end_destination.x = 68.927277
        #end_destination.y = 27.830568
        #different_end_destinations.append(end_destination)

        # 3
        end_destination = random.choice(spawn_points).location
        end_destination.x = 113.648468
        end_destination.y = 4.688451
        different_end_destinations.append(end_destination)

        # 4
        end_destination = random.choice(spawn_points).location
        end_destination.x = 68.927277
        end_destination.y = 27.830568
        different_end_destinations.append(end_destination)

        #5
        end_destination = random.choice(spawn_points).location
        end_destination.x = 82.522911
        end_destination.y = 70.302856
        different_end_destinations.append(end_destination)

        #6
        end_destination = random.choice(spawn_points).location
        end_destination.x = 85.100761
        end_destination.y = 16.689871
        different_end_destinations.append(end_destination)

        # 7
        end_destination = random.choice(spawn_points).location
        end_destination.x = 87.605782
        end_destination.y = 130.068909
        different_end_destinations.append(end_destination)

        return different_end_destinations

# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

# Spawning Pedestrians givent the world, client and amount of pedestrians
def spawn_pedestrians(world, client, type_pedestrian, x, y):
    # add pedestrians to the world
    print(f"Attempting to spawn a pedestrians.")
    walker_bp = world.get_blueprint_library().filter(type_pedestrian)[0]
    spawn_point = carla.Transform()
    spawn_point.location = cross_walks[0]
    spawn_point.location.x = x
    spawn_point.location.y = y
    spawn_point.location.z = 1.0

    batch = []
    batch.append(carla.command.SpawnActor(walker_bp, spawn_point))

    # Spawn the walker
    results = client.apply_batch_sync(batch, True)

    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})

    #spawn walker controller
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))

    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
            walkers.append(results[i]) #changed this from results[i].actor_id so we can simply destroy the actor later

    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)

    for i in range(0, len(all_actors), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # random max speed
        all_actors[i].set_max_speed(1 + random.random())

    #for i in range(0, len(all_actors), 2):
        # start walker
        #all_actors[i].start()
        
        # set destination to the other end of the crosswalk
        #destination = carla.Location(x=84.37948, y=1.86154, z=1.0)
        #all_actors[i].go_to_location(destination)

        # set max speed
        #all_actors[i].set_max_speed(1 + random.random())


class World(object):
    """ Class representing the surrounding environment """
    def __init__(self, carla_world, args, client):
        """Constructor method"""
        self._args = args
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

        # ------------ attributes -------------    
        self.player = None
        self.client = client

        # ----- traffic managers for different vehicles -----
        self.tm = self.client.get_trafficmanager()
        self.tm.set_random_device_seed(9)
        self.tm.set_respawn_dormant_vehicles(True)
        self.tm.set_boundaries_respawn_dormant_vehicles(20,500)

        

        # traffic manager for cautius vehicles
        self.tm2 = self.client.get_trafficmanager(5000)
        self.tm2.set_random_device_seed(9)
        self.tm2.set_respawn_dormant_vehicles(True)
        self.tm2.set_boundaries_respawn_dormant_vehicles(20, 500)
        # traffic mangaer for aggressiv vehicles
        self.tm3 = self.client.get_trafficmanager(5050)
        self.tm3.set_random_device_seed(9)
        self.tm3.set_respawn_dormant_vehicles(True)
        self.tm3.set_boundaries_respawn_dormant_vehicles(20, 500)

        # ----- cameras and sensors ---------
        self.camera = None
        self.right_camera = None
        self.left_camera = None

        #displays
        self.display = None
        self.right_display = None
        self.left_display = None

        #images
        self.image = None
        self.right_image = None
        self.left_image = None

        # captures
        self.capture = True
        self.right_capture = True
        self.left_capture = None

        #render arrays
        self.right = None
        self.left = None

        self.restart(args)

    def restart(self, args):
        """Restart the world"""
        # get tesla vehicle from blueprint
        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.filter('model3')[0]

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.player.set_autopilot(False)

            # spawn_point = self.map.get_waypoint(carla.Location(x=0, y=30, z=10))
            spawn_point = self.map.get_waypoint(carla.Location(x=-86.347275, y=24.404694, z=1.0))
            spawn_point.location.x = -86.347275
            spawn_point.location.y = 24.404694
            spawn_point.location.z = 1.0

            self.destroy()
            self.player = self.world.try_spawn_actor(bp, spawn_point.transform)
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()

            spawn_point = spawn_points[0]
            spawn_point.location.x = 50
            spawn_point.location.y = 25
            spawn_point.location.z = 1.0
            
            # print(spawn_point.location)
            self.player = self.world.try_spawn_actor(bp, spawn_point)
            
            self.modify_vehicle_physics(self.player)

        # Set up the sensors.
        self.setup_middle_camera()
        self.setup_left_camera()
        self.setup_right_camera()

        if self._args.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def spawn_vehicles_straight(self, x, y, type_car, color):
        # spawns one car, current simulation makes these cars red
        vehicle_list = []  # keep the spawned vehicle in vehicle_list, because we need to link them with traffic_manager
        bps = self.world.get_blueprint_library()
        vehicle_bp = bps.filter(type_car)[0]
        if color:
            vehicle_bp.set_attribute('color', color)

        point = carla.Transform(carla.Location(x=x, y=y, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.840790, roll=0.000000))
        try:
            # print(point)
            print("Spawning a straight vehicle.")
            vehicle = self.world.spawn_actor(vehicle_bp, point)
            if vehicle:
                vehicle_list.append(vehicle)
                vehicles_list.append(vehicle.id)
            else:
                print(f"Vehicle: {vehicle_bp} could not be printed at {point}")
        except Exception as e:
            print("--- Error trying to print straight vehilce -----")
            print(e)
            print('failed')  # if failed, print the hints.
            
        #add these vehicles into the traffice manager
        self.tm.global_percentage_speed_difference(10.0)
        tm_port = self.tm.get_port()
        for v in vehicle_list:
            v.set_autopilot(False, tm_port)
            self.tm.ignore_lights_percentage(v, 0)
            # self.tm.ignore_stop_signs(False)
            self.tm.distance_to_leading_vehicle(v, 0.8)
            self.tm.vehicle_percentage_speed_difference(v, -15)
            self.tm.set_synchronous_mode(False)

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception as e:
            print("----Error when modifying vehicle physics.----")
            print(e)
            print("\n")

    def spawn_vehicles_around_ego_vehicles(self):
        vehicle_list = [] #store spawned vehicles to manage with traffic manger(autopilot)
        
        accessible_points = [
            #transform = Transform(Location(x=230, y=195, z=40), Rotation(yaw=180))
            carla.Transform(carla.Location(x=83.075226, y=13.414804, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.840790, roll=0.000000)),
            carla.Transform(carla.Location(x=74.798752, y=28.343533, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000)),
            #1 coord
            #carla.Transform(carla.Location(x=-71.903824, y=28.343533, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000)),    
            #2 coord
            #carla.Transform(carla.Location(x=47.612568, y=28.343533, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000)),    
            #3 coord
            #carla.Transform(carla.Location(x=23.106254, y=28.343533, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000)),
            #4 coord
            #carla.Transform(carla.Location(x=-28.402511, y=28.343533, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000)),         
            #5 coord
            #carla.Transform(carla.Location(x=-3.671235, y=28.343533, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000)),
            carla.Transform(carla.Location(x=99.384415, y=-6.305729, z=0.600000), carla.Rotation(pitch=0.000000, yaw=90.390709, roll=0.000000)),
            carla.Transform(carla.Location(x=109.502762, y=53.293152, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-89.609253, roll=0.000000)),
            carla.Transform(carla.Location(x=99.078560, y=42.141800, z=0.600000), carla.Rotation(pitch=0.000000, yaw=90.390709, roll=0.000000)),
            carla.Transform(carla.Location(x=67.659744, y=69.822777, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.073273, roll=0.000000)),
            carla.Transform(carla.Location(x=43.373123, y=16.909227, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.840790, roll=0.000000)),
            carla.Transform(carla.Location(x=29.235720, y=16.765228, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.840790, roll=0.000000)),
            carla.Transform(carla.Location(x=59.812996, y=24.850224, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000)),
            carla.Transform(carla.Location(x=102.566177, y=43.965668, z=0.600000), carla.Rotation(pitch=0.000000, yaw=90.390709, roll=0.000000))
        ]
        bp = self.world.get_blueprint_library()
        vehicle_bps = [
            bp.filter('vehicle.chevrolet.impala')[0],
            bp.filter('vehicle.audi.a2')[0],
            bp.filter('vehicle.ford.mustang')[0],
            bp.filter('vehicle.audi.etron')[0],
            bp.filter('vehicle.bmw.grandtourer')[0],
            bp.filter('vehicle.citroen.c3')[0],
            bp.filter('vehicle.audi.a2')[0],
            bp.filter('vehicle.dodge.charger_2020')[0],
            bp.filter('vehicle.ford.crown')[0],
            bp.filter('vehicle.ford.mustang')[0]
            #bp.filter('vehicle.dodge.charger_2020')[0],
            #bp.filter('vehicle.bmw.grandtourer')[0],
            #bp.filter('vehicle.ford.mustang')[0],
            #bp.filter('vehicle.ford.mustang')[0],
            #bp.filter('vehicle.chevrolet.impala')[0],
        ]

        #make all normal vehicles around ego vehicle to be black and blue
        for i, v in enumerate(vehicle_bps):
            if i % 2:
                v.set_attribute('color', '0,0,255')
            else:
                v.set_attribute('color', "0,0,0")

        numbers_of_vehicles = len(accessible_points)
        
        for i in range(0, numbers_of_vehicles):
            point = accessible_points[i]
            vehicle_bp = vehicle_bps[i]
            try:
                vehicle = self.world.spawn_actor(vehicle_bp, point)
                vehicle_list.append(vehicle)
                vehicles_list.append(vehicle.id)

            except Exception as e:
                print('\n--failed trying to spawn vehicle around ego vehicle--')
                print(i, vehicle_bp)
                print(point)
                print(e)
                print("\n")
                pass
        
        #add these vehicles into the traffice manager
        self.tm.global_percentage_speed_difference(10.0)
        tm_port = self.tm.get_port()
        for v in vehicle_list:
            v.set_autopilot(False, tm_port)
            self.tm.ignore_lights_percentage(v, 0)
            self.tm.distance_to_leading_vehicle(v, 0.8)
            # self.tm.ignore_stop_signs(False)
            self.tm.vehicle_percentage_speed_difference(v, -15)
            self.tm.set_synchronous_mode(False)
        print(f"A total of {len(vehicle_list)} vehicles were spawned around ego vehicle.")     

    def spawn_agro_vehicles(self):
        """ Spawns 1 aggressive vehicle (fire truck) around the ego vehicle in a hardcoded spawn point """
        vehicle_list = []
        accessible_points = [ 
            carla.Transform(carla.Location(x=85.982246, y=66.358490, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.926727, roll=0.000000))
        ]
        bps = [self.world.get_blueprint_library().filter('vehicle.carlamotors.firetruck')[0]]
        numbers_of_vehicles = len(accessible_points)
        for i in range(numbers_of_vehicles):
            point = accessible_points[i]
            vehicle_bp = bps[i]
            try:
                vehicle = self.world.spawn_actor(vehicle_bp, point)
                vehicle_list.append(vehicle)
                vehicles_list.append(vehicle.id)
                # self.agro_vehicles.append(vehicle)
            except Exception as e:
                print('\n--failed trying to spawn agro vehicles--')
                print(e)
                print("\n")
        
        self.tm3.global_percentage_speed_difference(10.0)
        tm_port = self.tm3.get_port()
        for v in vehicle_list:
            v.set_autopilot(False, tm_port)
            self.tm3.ignore_lights_percentage(v, 100)
            self.tm3.distance_to_leading_vehicle(v, 1)
            self.tm3.vehicle_percentage_speed_difference(v, -20)
            self.tm3.update_vehicle_lights(v, True)
            self.tm3.set_synchronous_mode(False)
        print(f"Successfuly spawned {len(vehicle_list)} aggressive vehicles.")
                
    def spawn_cautious_vehicles(self):
        """ spawns 8 vehicles close to the ego vehicle """
        vehicle_list = []
        accessible_points = [
            carla.Transform(carla.Location(x=-45.317440, y=-11.645325, z=0.600002), carla.Rotation(pitch=0.000000, yaw=-90.161217, roll=0.000000)),
            carla.Transform(carla.Location(x=-45.311253, y=-1.694395, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-90.161217, roll=0.000000)),
            carla.Transform(carla.Location(x=-20.115120, y=16.749100, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.840790, roll=0.000000)),
            carla.Transform(carla.Location(x=-64.581863, y=-65.167366, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.403244, roll=0.000000)),
            carla.Transform(carla.Location(x=-41.833862, y=-16.555164, z=0.600002), carla.Rotation(pitch=0.000000, yaw=-90.161217, roll=0.000000)),
            carla.Transform(carla.Location(x=-24.336779, y=-57.785625, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.596735, roll=0.000000)),
            carla.Transform(carla.Location(x=-48.839951, y=-17.213200, z=0.600000), carla.Rotation(pitch=0.000000, yaw=90.432327, roll=0.000000)),
            carla.Transform(carla.Location(x=-52.330811, y=-14.039614, z=0.600000), carla.Rotation(pitch=0.000000, yaw=90.432327, roll=0.000000))
        ]

        bp = self.world.get_blueprint_library()
        vehicle_bps = [ 
            bp.filter('vehicle.dodge.charger_police')[0],
            bp.filter('vehicle.dodge.charger_police_2020')[0],
            bp.filter('vehicle.dodge.charger_police_2020')[0],
            bp.filter('vehicle.ford.ambulance')[0],
            bp.filter('vehicle.mercedes.coupe_2020')[0],
            bp.filter('vehicle.mercedes.sprinter')[0],
            bp.filter('vehicle.mini.cooper_s_2021')[0],
            bp.filter('vehicle.volkswagen.t2')[0]
        ]

        # orange and white
        for i,v in enumerate(vehicle_bps[4:], 3):
            if i % 2:
                v.set_attribute('color', '255,87,51')
            else:
                v.set_attribute('color', '255,255,255')

        numbers_of_vehicles = len(accessible_points)
        for i in range(numbers_of_vehicles):
            point = accessible_points[i]
            vehicle_bp = vehicle_bps[i]
            try:
                vehicle = self.world.spawn_actor(vehicle_bp, point)
                vehicle_list.append(vehicle)
                vehicles_list.append(vehicle.id)
            except:
                print('failed trying to spawn cautious vehicles')
                print(f"failed vehicle: {vehicle_bp}")
        
        self.tm3.global_percentage_speed_difference(10.0)
        tm_port = self.tm3.get_port()
        for v in vehicle_list:
            v.set_autopilot(False, tm_port)
            self.tm2.distance_to_leading_vehicle(v, 10.0)
            self.tm2.vehicle_percentage_speed_difference(v, 60)
            self.tm2.update_vehicle_lights(v, True)
            self.tm2.auto_lane_change(v, False)
            self.tm2.set_synchronous_mode(False)
        print(f"Successfuly spawned {len(vehicle_list)} cautious vehicles.")

    def spawn_bikes(self):
        vehicle_list = [] #store spawned vehicles to manage with traffic manger(autopilot)
        accessible_points = [
            carla.Transform(carla.Location(x=-64.644844, y=24.471010, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000)),
            carla.Transform(carla.Location(x=-67.254570, y=27.963758, z=0.600000), carla.Rotation(pitch=0.000000, yaw=0.159198, roll=0.000000)),
            carla.Transform(carla.Location(x=-87.623032, y=12.967159, z=0.600000), carla.Rotation(pitch=0.000000, yaw=-179.840790, roll=0.000000))
        ]

        bp = self.world.get_blueprint_library()
        vehicle_bps = [
           bp.filter('vehicle.diamondback.century')[0], # this is a bike
           bp.filter('vehicle.kawasaki.ninja')[0],
           bp.filter('vehicle.harley-davidson.low_rider')[0]
        ]

        #make all normal vehicles around ego vehicle to be black
        for i, v in enumerate(vehicle_bps):
            if i % 2:
                v.set_attribute('color', '0,0,255')
            else:
                v.set_attribute('color', "0,0,0")

        numbers_of_vehicles = len(accessible_points)
        
        for i in range(0, numbers_of_vehicles):
            point = accessible_points[i]
            vehicle_bp = vehicle_bps[i]
            try:
                vehicle = self.world.spawn_actor(vehicle_bp, point)
                vehicle_list.append(vehicle)
                vehicles_list.append(vehicle.id)

            except Exception as e:
                print('\n--failed trying to spawn vehicle around ego vehicle--')
                print(i, vehicle_bp)
                print(point)
                print(e)
                print("\n")
                pass
        print(f"Successfully spawned {len(vehicle_list)} bikes!\n")

        #add these vehicles into the traffice manager
        self.tm.global_percentage_speed_difference(10.0)
        tm_port = self.tm3.get_port()
        for v in vehicle_list:
            v.set_autopilot(False, tm_port)
            self.tm.ignore_lights_percentage(v, 0)
            self.tm.distance_to_leading_vehicle(v, 0.8)
            self.tm.vehicle_percentage_speed_difference(v, -15)
            self.tm.set_synchronous_mode(False)

    # ======= camera functions =======
    def camera_blueprint(self):
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(1920))
        camera_bp.set_attribute('image_size_y', str(1080))
        camera_bp.set_attribute('fov', str(90))
        return camera_bp
    
    def setup_middle_camera(self):
        camera_transform = carla.Transform(carla.Location(x = 0.2, z = 1.10), carla.Rotation(pitch = 0))
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=self.player)
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))

        calibration = np.identity(3)
        calibration[0,2] = 1920 // 2.0
        calibration[1, 2] = 1080 / 2.0
        calibration[0, 0] = calibration[1, 1] = 1920 / (2.0 * np.tan(90 * np.pi / 360.0))
        self.camera.calibration = calibration
    
    # to remove car peripherals, just set x and y to zero
    def setup_right_camera(self):
        right_camera_transform = carla.Transform(carla.Location(x = 0.2, y = 0.80, z = 1.10), carla.Rotation(pitch=0, yaw = 90.0))
        self.right_camera = self.world.spawn_actor(self.camera_blueprint(), right_camera_transform, attach_to=self.player)
        weak_right_self = weakref.ref(self)
        self.right_camera.listen(lambda right_image: weak_right_self().set_right_image(weak_right_self, right_image))

        calibration = np.identity(3)
        calibration[0,2] = 1920 // 2.0
        calibration[1, 2] = 1080 / 2.0
        calibration[0, 0] = calibration[1, 1] = 1920 / (2.0 * np.tan(90 * np.pi / 360.0))
        self.right_camera.calibration = calibration
    
    def setup_left_camera(self):
        left_camera_transform = carla.Transform(carla.Location(x = 0.2, y = -0.8, z = 1.10), carla.Rotation(pitch=0, yaw = -90.0))
        self.left_camera = self.world.spawn_actor(self.camera_blueprint(), left_camera_transform, attach_to=self.player)
        weak_left_self = weakref.ref(self)
        self.left_camera.listen(lambda left_image: weak_left_self().set_left_image(weak_left_self, left_image))

        calibration = np.identity(3)
        calibration[0,2] = 1920 // 2.0
        calibration[1, 2] = 1080 / 2.0
        calibration[0, 0] = calibration[1, 1] = 1920 / (2.0 * np.tan(90 * np.pi / 360.0))
        self.left_camera.calibration = calibration
    
    @staticmethod
    def set_image(weak_self, img):
        self = weak_self()
        if self.capture:
            self.image = img
            self.capture = False

    @staticmethod
    def set_right_image(weak_right_self, right_img):
        self = weak_right_self()
        if self.right_capture:
            self.right_image = right_img
            self.right_capture = False

    @staticmethod
    def set_left_image(weak_left_self, left_img):
        self = weak_left_self()
        if self.left_capture:
            self.left_image = left_img
            self.left_capture = False

    def render(self, display):
        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (self.image.height, self.image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))

    def right_render(self, right_display):
        if self.right_image is not None:
            # self.depth_image.convert(cc.CityScapesPalette) #original
            i = np.array(self.right_image.raw_data)
            i2 = i.reshape((VIEW_HEIGHT, VIEW_WIDTH, 4))
            i3 = i2[:, :, :3]
            self.right = i3
            cv2.imshow("right_image", self.right)

    def left_render(self, left_display):
        if self.left_image is not None:
            i = np.array(self.left_image.raw_data)
            i2 = i.reshape((VIEW_HEIGHT, VIEW_WIDTH, 4))
            i3 = i2[:, :, :3]
            self.left = i3
            cv2.imshow("left_image", self.left)    

    def destroy_sensors(self):
        self.camera.destroy()
        self.left_camera.destroy()
        self.right_camera.destroy()
        print("All cameras should be destroyed.")

    def destroy_cars(self):
        """Destroys all actors"""
        print("\n\n----- Destroying vehicles in the scene -----")       

        #destroy ego vehicle
        self.player.destroy()

        print('\ndestroying %d vehicles' % len(vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        print("All vehicles should now be destroyed.")
    
    def destroy_pedestrians(self):
        print("\n----- Destroying Pedestrians -----")
        print('\ndestroying %d walkers' % len(walkers_list))
        walker_ids = []
        con_ids = []

        #seperate controllers and walkers
        for item in walkers_list:
            # print(item)
            con_ids.append(item['con'])
            walker_ids.append(item['id'])

        # stop the controllers
        cons = self.world.get_actors(con_ids)
        # print(cons)
        for con in cons:
            con.stop()

        # print(walker_ids)
        # destroy the pedestrians
        self.client.apply_batch([carla.command.DestroyActor(x) for x in walker_ids])

        print("All pedestrians should now be destroyed.")
        

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world):
        if isinstance(world.player, carla.Vehicle):
            pass
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0

    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
            steer_increment = 0.01
            if event.type == pygame.JOYAXISMOTION:
                #if event.a6xis == 0:
                self._steer_cache = max(-1, min(1, joystick.get_axis(0) * 1))
                self._control.steer = self._steer_cache

    @staticmethod
    def _is_quit_shortcut(key):
        """Shortcut for quitting"""
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)

# ==============================================================================
# -- Game Loop ---------------------------------------------------------
# ==============================================================================
loc = None
def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        if args.seed:
            random.seed(args.seed)

        client = carla.Client(args.host, args.port)
        client.set_timeout(4.0)

        sim_world = client.get_world()

        if args.sync:
            settings = sim_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)

  
        world = World(sim_world, args, client)
        controller = KeyboardControl(world)

        if args.agent == "Basic":  
            print("Vehicle behavior will be Basic")                                                                                                                                                     
            agent = BasicAgent(world.player)
            agent.ignore_stop_signs(False)
        else:
            print(f"Vehicle behavior will be {args.behavior}")
            agent = BehaviorAgent(world.player, behavior=args.behavior)
            agent.ignore_stop_signs(False)


        # camera stuff
        # works best, goes fullscreen but you have to open it using powershell in the monitor you want it to display on
        world.display = pygame.display.set_mode((pygame.display.Info().current_w, pygame.display.Info().current_h), pygame.SCALED)
        world.right_display = cv2.namedWindow('right_image')
        world.left_display = cv2.namedWindow('left_image')
      
                                                                                                                                                                                                                                                                                                                                                                                                                
        # Set the agent destination
        spawn_points = world.map.get_spawn_points()
        destination = random.choice(spawn_points).location
        # print(f"First print of destination: {destination}")
        destination.x = 110.800049
        destination.y = 72.599747
        print(f"\nStart Destination: {destination}")

        end_destination = random.choice(spawn_points).location
        end_destination.x = 106.416290
        end_destination.y = -12.711931
        print(f"End Destination: {end_destination}")
        #tried swapping parameters to follow CARLA docs, maybe will fix crashing situation
        agent.set_destination(end_location=end_destination, start_location=destination)
        
        clock = pygame.time.Clock()

        first_crosswalk = world.player.get_location()
        first_crosswalk.x = 84
        first_crosswalk.y = 25
        first_crosswalk.z = 0.004

        cross_walks.append(first_crosswalk)

        clock = pygame.time.Clock()

        oldTime = time.time()

        index = 0
        # Getting different end destinations ##########################
        different_end_destinations = set_destinations(spawn_points)
        spawn_vehicle_at_spawn = False

        semi = False
        ####################################################
        print("\n---- Entering Game Loop ----")
        while True:
            clock.tick()
            if args.sync:
                sim_world.tick()
                # world.world.tick()
            else:
                world.world.wait_for_tick()
            if controller.parse_events():
                return
            
            # === camera stuff here ====
            world.capture = True
            world.right_capture = True
            world.left_capture = True

            # renders
            world.render(world.display)
            pygame.display.flip()
            world.right_render(world.right_display)
            world.left_render(world.left_display)
            cv2.waitKey(1)
            # =====================


            velocity = world.player.get_velocity()
            speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) # in m/s

            ##### SCENARIOS #####
            loc = world.player.get_location()
            #1  SPAWN VEHICLES LEFT AND RIGHT OF SPAWNED PLAYER
            if spawn_vehicle_at_spawn == False:
                print("\n----Entering the first scenario...----")
                spawn_vehicle_at_spawn = True
                world.spawn_vehicles_straight(x=-52.498489, y=-11.581840, color=None, type_car="vehicle.carlamotors.carlacola")
                world.spawn_vehicles_straight(x=-45, y=63, color="255,0,0", type_car="vehicle.chevrolet.impala")
                spawn_vehicle_at_spawn = True
                print("----Exiting the first scenario----\n")

            #2 SPAWN PEDESTRIAN AT CROSSWALK and VEHICLES
            if loc.x < 55 and loc.x > 50 and loc.y < 25 and loc.y > 22 and len(walkers) == 0:
                print("\n----Entering the second scenario...----")
                spawn_pedestrians(world=world.world, client=client, type_pedestrian="walker.pedestrian.0018", x=86,y=31)
                # randTest()
                world.spawn_vehicles_straight(x=106, y=52, color="255,0,0", type_car="vehicle.jeep.wrangler_rubicon")
                world.spawn_vehicles_straight(x=99,y=-22, color="255,0,0", type_car="vehicle.lincoln.mkz_2017")
                print("----Exiting the second scenario----\n")

            #3 SPAWN PEDESTRIAN AT NEXT CROSSWALK AFTER #2
            if loc.x < 88 and loc.x > 87 and loc.y < 25 and loc.y > 24 and len(walkers) == 1:
                print("\n----Entering the third scenario...----")
                spawn_pedestrians(world=world.world, client=client, type_pedestrian="walker.pedestrian.0009", x=111.590347, y=2.08)
                print("----Exiting the third scenario----\n")
            #####################


            # END script if time limit reaches (15 min) or it has reached it's last destination
            if time.time() - oldTime >= (59*3) and time.time() - oldTime < (59*16):
                print("It has been 5 minutes, scenario ending")
                print(f"A total of {index}/7 destinations were reached.")
                break

            if index == 3 and agent.done():
                print("All  destinations have been reached.")
                total_time = (time.time() - oldTime) /59
                print(f"A total of {round(total_time)}/15 minutes have passed.")
                break
                # print("index: " + str(index))
                # print(f"Total time elapsed: {time.time()-oldTime/59}")

            if agent.done():
                if args.loop:
                    agent.set_destination(random.choice(spawn_points).location)
                    print("searching for another target")
                else:
                    print(f"Target {index + 1} has been reached, sending new target")
                    agent.set_destination(end_location=different_end_destinations[index])
                    index += 1

            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:  # Check if a key is pressed down
                    if event.key == pygame.K_d:
                        print("testttttttt")
                        semi = True
            if semi == False:
                world.player.apply_control(agent.run_step())

    finally:

        if world is not None:
            # change the weather back to defualt weather here
            settings = sim_world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.tm.set_synchronous_mode(False)
            world.tm2.set_synchronous_mode(False)
            world.tm3.set_synchronous_mode(False)
            world.world.apply_settings(settings)

            """ Memory Management """
            world.destroy_sensors() #destroys cameras
            world.destroy_cars()
            #world.destroy_pedestrians()

        pygame.quit()
        cv2.destroyAllWindows()

# ==============================================================================
# -- main() --------------------------------------------------------------
# ==============================================================================

def main():
    """Main method"""

    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='Print debug information')
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
        '--res',
        metavar='WIDTHxHEIGHT',
        default='600x400',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='Actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')
    argparser.add_argument(
        "-a", "--agent", type=str,
        choices=["Behavior", "Basic"],
        help="select which agent to run",
        default="Behavior")
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')
    argparser.add_argument(
        '-s', '--seed',
        help='Set seed for repeating executions (default: None)',
        default=None,
        type=int)

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)
    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()