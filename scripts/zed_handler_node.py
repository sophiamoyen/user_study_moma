# Author: Sophie Lueth
# Date: July 2024

import numpy as np
import signal
import sys
import matplotlib.pyplot as plt

import rospy

from std_msgs.msg import String

from zed_wrapper import ZedWrapper

ZED_SERIAL_NUMBERS = np.array([
                               22559299,
                               24285872,
                               ])

zeds = []

"""
possible commands:
- 'save_rgb <sn> <path>'
- 'save_depth <sn> <path>'
- 'start_video <sn> <path>'
- 'grab_frame <sn>'
- 'release_video <sn>'
- 'start_depth_recording <sn> <path>'
- 'grab_depth_frame <sn>'
- 'release_depth_recording <sn> <path>'
"""


def zed_trigger_cb(msg):
    global zeds
    
    command = msg.data
    cmds = command.split()
    invalid_command = False
    if cmds[0] == 'save_rgb':
        try:
            sn = int(cmds[1])
            path = cmds[2]
            index = np.argwhere(ZED_SERIAL_NUMBERS == sn)[0, 0]
            rgb = zeds[index].get_rgb()
            plt.imsave(path, rgb)
            print(f'Successfully saved imaged from camera {sn} to {path}')
        except Exception as e:
            print(e)
            invalid_command = True
    
    elif cmds[0] == 'save_depth':
        try:
            sn = int(cmds[1])
            path = cmds[2]
            index = np.argwhere(ZED_SERIAL_NUMBERS == sn)[0, 0]
            rgb = zeds[index].get_depth()
            np.save(path, rgb)
            print(f'Successfully saved depthmap from camera {sn} to {path}')
        except Exception as e:
            print(e)
            invalid_command = True
    
    elif cmds[0] == 'start_video':
        try:
            sn = int(cmds[1])
            path = cmds[2]
            index = np.argwhere(ZED_SERIAL_NUMBERS == sn)[0, 0]
            zeds[index].open_video_writer(path, fps=15)
            print(f'Successfully started video recording from camera {sn} to {path}')
        except Exception as e:
            print(e)
            invalid_command = True
            
    elif cmds[0] == 'grab_frame':
        try:
            sn = int(cmds[1])
            index = np.argwhere(ZED_SERIAL_NUMBERS == sn)[0, 0]
            zeds[index].add_current_frame_to_video()
            # print(f'Successfully added frame to video recording from camera {sn}')
        except Exception as e:
            print(e)
            invalid_command = True
            
    elif cmds[0] == 'release_video':
        try:
            sn = int(cmds[1])
            index = np.argwhere(ZED_SERIAL_NUMBERS == sn)[0, 0]
            zeds[index].close_video_writer()
            print(f'Successfully video recording from camera {sn}')
        except Exception as e:
            print(e)
            invalid_command = True
    
    elif cmds[0] == 'start_depth_recording':
        try:
            sn = int(cmds[1])
            index = np.argwhere(ZED_SERIAL_NUMBERS == sn)[0, 0]
            path = cmds[2]
            zeds[index].open_depth_recording(path)
            print(f'Successfully opened depth recording for camera {sn}')
        except Exception as e:
                print(e)
                invalid_command = True
        
    elif cmds[0] == 'grab_depth_frame':
        try:
            sn = int(cmds[1])
            index = np.argwhere(ZED_SERIAL_NUMBERS == sn)[0, 0]
            zeds[index].add_current_depth_to_recording()
            print(f'Successfully added depth frame to recording from camera {sn}')
        except Exception as e:
            print(e)
            invalid_command = True
            
    elif cmds[0] == 'release_depth_recording':
        try:
            sn = int(cmds[1])
            index = np.argwhere(ZED_SERIAL_NUMBERS == sn)[0, 0]
            zeds[index].close_depth_recording()
            print(f'Successfully saved depth recording from camera {sn}')
        except Exception as e:
            print(e)
            invalid_command = True
                
    else:
        invalid_command = True
    
    if invalid_command:
        print(f"Received invalid command: {command}")
        print("""possible commands:
            - 'save_rgb <sn> <path>'
            - 'save_depth <sn> <path>'
            - 'start_video <sn> <path>'
            - 'grab_frame <sn>' : add current image to video recording
            - 'release_video <sn>'
            - 'start_depth_recording <sn> <path>'
            - 'grab_depth_frame <sn>': add current depth image to video recording
            - 'release_depth_recording <sn> <path>'""")


if __name__ == '__main__':
    def close(signum, frame):
        for zed in zeds:
            zed.close()
        sys.exit(0)

    signal.signal(signal.SIGTSTP, close)
    signal.signal(signal.SIGINT, close)
    rospy.init_node('zed_node')
    
    print(f'Initializing ZED cameras with Serial Numbers: {[sn for sn in ZED_SERIAL_NUMBERS]}')
    
    zeds = [ZedWrapper(sn, fps=30) for sn in ZED_SERIAL_NUMBERS]
    
    trigger_sub = rospy.Subscriber('/zed_trigger', String, zed_trigger_cb, queue_size=10)
    
    rospy.spin()
    
