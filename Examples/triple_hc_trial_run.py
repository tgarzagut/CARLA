#!/usr/bin/env python
# referenced code from 
# Copyright (c) 2019 Aptiv
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# this code references https://dkqhzm2.tistory.com/entry/CARLA-double-monitor-manualcontrol 
# I used this code to solve my problem, I changed the cameras to regular cameras, and then changed the
# angle of these cameras to that the 3 monitor view was more seamless

"""
An example of client-side bounding boxes with basic car controls.

Controls:

    W			 : throttle
    S			 : brake
    AD			 : steer
    Space		 : hand-brake

    ESC			 : quit
"""

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


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



# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import time
start = time.time()

import carla
from carla import ColorConverter as cc

import weakref
import random
import cv2

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_SPACE
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_m
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

VIEW_WIDTH = 1920
VIEW_HEIGHT = 1080
VIEW_FOV = 90

pygame.init()
# Initialize joystick subsystem
pygame.joystick.init()

# Check how many joysticks are connected
joystick_count = pygame.joystick.get_count()
print("Number of joysticks:", joystick_count)

joystick = pygame.joystick.Joystick(0)
joystick.init()





# ==============================================================================
# -- BasicSynchronousClient ----------------------------------------------------
# ==============================================================================


class BasicSynchronousClient(object):
    """
    Basic implementation of a synchronous client.
    """

    def __init__(self):
        self.client = None
        self.world = None
        self.car = None

        # cameras 
        self.camera = None
        self.right_camera = None
        self.left_camera = None

        # displays
        self.display = None
        self.right_display = None
        self.left_display = None

        # images
        self.image = None
        self.right_image = None
        self.left_image = None

        # captures
        self.capture = True
        self.right_capture = True
        self.left_capture = None

        # render arrays
        self.counter = 0
        self.right = None
        self.left = None

        # logging
        self.pose = []
        self.log = False
        


    def camera_blueprint(self):
        """
        Returns camera blueprint.
        """

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        return camera_bp

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """

        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        self.world.apply_settings(settings)

    def setup_car(self):
        """
        Spawns actor-vehicle to be controled.
        """

        car_bp = self.world.get_blueprint_library().filter('model3')[0]
        location = random.choice(self.world.get_map().get_spawn_points())
        self.car = self.world.spawn_actor(car_bp, location)

    def setup_camera(self):
        """
        Spawns actor-camera to be used to render view.
        Sets calibration for client-side boxes rendering.
        """

        camera_transform = carla.Transform(carla.Location(x=0.0, z=2.0), carla.Rotation(pitch=0))
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=self.car)
        weak_self = weakref.ref(self)
        self.camera.listen(lambda image: weak_self().set_image(weak_self, image))

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration
        
    def setup_right_camera(self):
        """
        Spawns actor-camera to be used to render view.
        Sets calibration for client-side boxes rendering.
        """

        right_camera_transform = carla.Transform(carla.Location(x=0, z=2.0), carla.Rotation(pitch=0, yaw = 90.0))
        self.right_camera = self.world.spawn_actor(self.camera_blueprint(), right_camera_transform, attach_to=self.car)
        weak_right_self = weakref.ref(self)
        self.right_camera.listen(lambda right_image: weak_right_self().set_right_image(weak_right_self, right_image))

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.right_camera.calibration = calibration
    
    def setup_left_camera(self):
        """
        Spawns actor-camera to be used to render view.
        Sets calibration for client-side boxes rendering.
        """

        left_camera_transform = carla.Transform(carla.Location(x=0, z=2.0), carla.Rotation(pitch=0, yaw = -90.0))
        self.left_camera = self.world.spawn_actor(self.camera_blueprint(), left_camera_transform, attach_to=self.car)
        weak_left_self = weakref.ref(self)
        self.left_camera.listen(lambda left_image: weak_left_self().set_left_image(weak_left_self, left_image))

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.left_camera.calibration = calibration

    class JoystickControl(object):
        """Class that handles joystick input."""
        def __init__(self, world):
            self._world = world
            if isinstance(world.car, carla.Vehicle):
                pass
            else:
                raise NotImplementedError("Actor type not supported")
            self._steer_cache = 0.0
            pygame.joystick.init()
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
        def control(self):
            clock = pygame.time.Clock()
            control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False)
            """Applies control to main car based on joystick input."""
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True 
                normalized_throttle = max(0.0, (1 - self._joystick.get_axis(1)) / 2)
                _brake = (1 - self._joystick.get_axis(1))
                control.throttle = min(0, normalized_throttle - _brake)

                control.reverse = self._joystick.get_button(5)

                steer_axis_value = self._joystick.get_axis(0)  # X-axis
                control.steer = steer_axis_value

                
                
                # control.reverse = _throttle < 0  # Set reverse if throttle is negative

                self._world.car.apply_control(control)

            return False

    @staticmethod
    def set_image(weak_self, img):
        """
        Sets image coming from camera sensor.
        The self.capture flag is a mean of synchronization - once the flag is
        set, next coming image will be stored.
        """

        self = weak_self()
        if self.capture:
            self.image = img
            self.capture = False

    @staticmethod
    def set_right_image(weak_right_self, right_img):
        """
        Sets image coming from camera sensor.
        The self.capture flag is a mean of synchronization - once the flag is
        set, next coming image will be stored.
        """

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
        """
        Transforms image from camera sensor and blits it to main pygame display.
        """

        if self.image is not None:
            array = np.frombuffer(self.image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (VIEW_HEIGHT, VIEW_WIDTH, 4))
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



    def log_data(self):
            global start
            freq = 1/(time.time() - start)

    #		sys.stdout.write("\rFrequency:{}Hz		Logging:{}".format(int(freq),self.log))
            sys.stdout.write("\r{}".format(self.car.get_transform().rotation))

            sys.stdout.flush()
            if self.log:
                name ='log/' + str(self.counter) + '.png'
                self.depth_image.save_to_disk(name)
                position = self.car.get_transform()
                pos=None
                pos = (int(self.counter), position.location.x, position.location.y, position.location.z, position.rotation.roll, position.rotation.pitch, position.rotation.yaw)
                self.pose.append(pos)
                self.counter += 1
            start = time.time()
        
            
        

    def game_loop(self):
        """
        Main program loop.
        """

        try:
            pygame.init()

            self.client = carla.Client('127.0.0.1', 2000)
            self.client.set_timeout(2.0)
            self.world = self.client.get_world()

            self.setup_car()
            
            self.setup_right_camera()
            self.setup_left_camera()
            self.setup_camera()

            # self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.SCALED)
            self.right_display = cv2.namedWindow('right_image')
            self.left_display = cv2.namedWindow("left_image")
            self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.SCALED)

            pygame_clock = pygame.time.Clock()

            self.set_synchronous_mode(True)
            vehicles = self.world.get_actors().filter('vehicle.*')
            joystick_control = BasicSynchronousClient.JoystickControl(self)

            while True:
                self.world.tick()
                self.capture = True
                self.right_capture = True
                self.left_capture = True
                pygame_clock.tick_busy_loop(30)
                # self.render(self.display)
                pygame.display.flip()
                pygame.event.pump()
                self.right_render(self.right_display)
                # pygame.display.flip()
                # pygame.event.pump()
                self.left_render(self.left_display)
                self.render(self.display)

                self.log_data()
                cv2.waitKey(1)
                if joystick_control.control():  # Apply joystick control
                    return
                
        #except Exception as e: print(e)
        finally:

            self.set_synchronous_mode(False)
            self.camera.destroy()
            self.right_camera.destroy()
            self.left_camera.destroy()
            self.car.destroy()
            pygame.quit()
            cv2.destroyAllWindows()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    Initializes the client-side bounding box demo.
    """

    try:
        client = BasicSynchronousClient()
        client.game_loop()
    finally:
        print('EXIT')


if __name__ == '__main__':
    main()
