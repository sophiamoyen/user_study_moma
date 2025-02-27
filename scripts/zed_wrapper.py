# Author: Sophie Lueth
# Date: June 2024

import sys
import queue
import time
from copy import deepcopy
from threading import Thread, Lock
# from multiprocessing import Process as Thread
import numpy as np
import cv2 as cv

import pyzed.sl as sl

# debug
# import matplotlib
# matplotlib.use('Agg')
# import matplotlib.pyplot as plt


class ZedWrapper:
    def __init__(self,
                 serial_number,
                 resolution=sl.RESOLUTION.HD720,
                 fps=30,
                 depth=True):
        """
        Args:
            serial_number (int): serial number of the camera to open
            resolution (sl.RESOLUTION): resolution to set the camera
            fps (int): frames per second to set the camera
        """
        self.resource_lock = Lock()
        self.zed = sl.Camera()
        camera_found = False
        for device in self.zed.get_device_list():
            if device.serial_number == serial_number:
                camera_found = True
        if not camera_found:
            print(f'ZED camera with Serial Number {serial_number} not detected. \nPlease check connection and try again.')
            sys.exit(0)

        # HD720 resolution 1280x720 (per camera)

        config = sl.InitParameters(camera_resolution=resolution,  # camera_resolution=sl.RESOLUTION.VGA,
                                   camera_fps=fps,
                                   depth_mode=sl.DEPTH_MODE.ULTRA,
                                   coordinate_units=sl.UNIT.METER,
                                   depth_minimum_distance=0.2)
        config.set_from_serial_number(serial_number)

        err = self.zed.open(config)
        if err != sl.ERROR_CODE.SUCCESS:
            print('Could not open connection to ZED Camera... Please check permissions')
            sys.exit(0)

        self.serial_number = serial_number
        self.resolution = self.zed.get_camera_information().camera_configuration.resolution
        self.fps = fps

        # put reading out Camera into an extra thread
        self.q_bgr = queue.Queue(maxsize=1)
        self.q_depth = queue.Queue()
        self.t = Thread(target=self._update, daemon=True)
        self.t.start()

        # record video
        self.video_buffer = []
        self.video_writer = None

        # record depth
        self.depth_buffer = []
        self.depth_path = None

        print(f"=== Camera {serial_number} is ready! ===")

    def _update(self):
        while True:
            with self.resource_lock:
                image = sl.Mat()
                depth_map = sl.Mat()
                runtime_parameters = sl.RuntimeParameters()
                if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                    self.zed.retrieve_image(image, sl.VIEW.LEFT)
                    # self.zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)

                    bgra = image.get_data()
                    # depth = depth_map.get_data()

                    if not self.q_bgr.empty():
                        try:
                            self.q_bgr.get_nowait()
                        except queue.Empty:
                            pass

                    self.q_bgr.put(bgra[:, :, :3])

                    # if not self.q_depth.empty():
                    #     try:
                    #         self.q_depth.get_nowait()
                    #     except queue.Empty:
                    #         pass
                    #
                    # self.q_depth.put(depth)

                else:
                    break

            time.sleep(0.001)

    def open_video_writer(self, path, fps=None):
        with self.resource_lock:
            if fps is None:
                fps = self.fps
            self.video_writer = cv.VideoWriter(path, cv.VideoWriter_fourcc(*'mp4v'), fps, (self.resolution.width, self.resolution.height))

    def add_current_frame_to_video(self):
        with self.resource_lock:
            if self.video_writer is None:
                print(f'Warning: VideoWriter for SN {self.serial_number} was not initialized, but you try to add frames!')
                return

            if not self.q_bgr.empty():
                bgr = self.q_bgr.get()
                bgr = deepcopy(bgr)
                self.video_buffer.append(bgr)

                # # TO test
                # rgb = cv.cvtColor(bgr, cv.COLOR_BGR2RGB)
                # cv.imshow('image', rgb)
                # cv.waitKey(1)

    def close_video_writer(self):
        with self.resource_lock:
            if self.video_writer is None:
                print(f'Warning: VideoWriter for SN {self.serial_number} was not initialized, but you try to close it!')
                return

            print(f'=== Exporting Video for ZED {self.serial_number}...')

            for bgr in self.video_buffer:
                self.video_writer.write(bgr)

            self.video_writer.release()
            self.video_writer = None
            self.video_buffer = []

    def open_depth_recording(self, path):
        with self.resource_lock:
            self.depth_path = path
            self.depth_buffer = []

    def add_current_depth_to_recording(self):
        with self.resource_lock:
            if self.depth_path is None:
                print(f'Warning: Depth Recording for SN {self.serial_number} was not initialized, but you try to add frames!')
                return

            depth = self.get_depth()
            if depth is None:
                self.depth_buffer.append(self.depth_buffer[-1])
            else:
                self.depth_buffer.append(self.get_depth())

    def close_depth_recording(self):
        with self.resource_lock:
            if self.depth_path is None:
                print(f'Warning: Depth Recording for SN {self.serial_number} was not initialized, but you try to close it!')
                return

            print(f'=== Exporting Depth for ZED {self.serial_number}...')
            breakpoint()
            np.save(self.depth_path, self.depth_buffer)

            self.depth_buffer = []
            self.depth_path = None

    def get_rgbd(self):
        with self.resource_lock:
            rgbd = np.concatenate((self.get_rgb(), self.get_depth()[:, :, np.newaxis]), axis=2)

            return rgbd

    def get_rgb(self):
        with self.resource_lock:
            if not self.q_bgr.empty():
                rgb = cv.cvtColor(self.q_bgr.get(), cv.COLOR_BGR2RGB)
                return rgb

    def get_depth(self):
        with self.resource_lock:
            if not self.q_depth.empty():
                return self.q_depth.get()

    def close(self):
        with self.resource_lock:
            self.zed.close()
            self.video_writer.release()
