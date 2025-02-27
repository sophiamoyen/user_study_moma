#!/usr/bin/env python3

"""Node to record a rosbag with start/stop/pause control through service calls.
Example call:
    rosrun utilities rosbag_controlled_recording.py _rosbag_command:="rosbag record -o /home/foo/test_bag /bar_topic" _record_from_startup:=false
Then start/pause/resume/stop can be controlled through:
    rosservice call /rosbag_controlled_recording/start
    rosservice call /rosbag_controlled_recording/pause_resume
    rosservice call /rosbag_controlled_recording/pause_resume
    rosservice call /rosbag_controlled_recording/stop
Note that pausing does not modify the recorded time of messages, i.e. the bag's total length is unaffected. A list of
  pause-resume times is logged when stopping, in case the paused period needs to be (manually) removed afterwards.
If this node is killed recording is also stopped. If recording was paused, it is momentarily resumed before stopping.
"""

import psutil
import subprocess
import shlex
import signal

import rospy
from std_srvs.srv import Empty, EmptyResponse


def signal_process_and_children(pid, signal_to_send, wait=False):
    process = psutil.Process(pid)
    for children in process.children(recursive=True):
        if signal_to_send == 'suspend':
            children.suspend()
        elif signal_to_send == 'resume':
            children.resume()
        else:
            children.send_signal(signal_to_send)
    if wait:
        process.wait()


def format_to_columns(input_list, cols):
    """Adapted from https://stackoverflow.com/questions/171662/formatting-a-list-of-text-into-columns"""
    max_width = max(map(len, input_list))
    justify_list = map(lambda x: x.ljust(max_width + 4), input_list)
    lines = (''.join(justify_list[i:i + cols]) for i in range(0, len(justify_list), cols))
    return '\n'.join(lines)


class RosbagControlledRecorder(object):
    """Record a rosbag with service calls to control start, stop  and pause"""

    def __init__(self, rosbag_command_):
        self.rosbag_command = shlex.split(rosbag_command_)
        self.recording_started = False
        self.recording_paused = False
        self.recording_stopped = False
        self.pause_resume_times = []
        self.process_pid = None
        

    def start_recording(self):
        if self.recording_started:
            rospy.logwarn("Recording has already started - nothing to be done")
        else:
            process = subprocess.Popen(self.rosbag_command)
            self.process_pid = process.pid
            self.recording_started = True
            rospy.logwarn("Started recording rosbag")
            

    def pause_resume_recording(self):
        if self.recording_started:
            if self.recording_paused:
                signal_process_and_children(self.process_pid, 'resume')
                self.recording_paused = False
                rospy.loginfo("Recording resumed")
            else:
                signal_process_and_children(self.process_pid, 'suspend')
                self.recording_paused = True
                rospy.loginfo("Recording paused")
            self.pause_resume_times.append(rospy.get_time())
        else:
            rospy.logwarn("Recording not yet started - nothing to be done")

    def stop_recording(self):
        if self.process_pid is not None:
            if self.recording_paused:  # need to resume process in order to cleanly kill it
                self.pause_resume_recording()
            if self.pause_resume_times:  # log pause/resume times for user's reference
                pause_resume_str = map(str, self.pause_resume_times)
                pause_resume_str[0:0] = ['PAUSE', 'RESUME']
                rospy.logwarn("List of pause and resume times:\n%s\n", format_to_columns(pause_resume_str, 2))
            signal_process_and_children(self.process_pid, signal.SIGINT, wait=True)
            self.process_pid = None
            rospy.logwarn("Stopped recording rosbag")
        self.recording_started = False
        self.recording_stopped = True


if __name__ == '__main__':
    rospy.init_node('rosbag_controlled_recording')

    # Get parameters
    rosbag_command = rospy.get_param('~rosbag_command','rosbag record -o /home/hydra/sophia_ws/ros_ws/src/data_collection/data/ /rosout')  # str with rosbag command line command to be issued
    record_from_startup = rospy.get_param('~record_from_startup', False)  # whether to start node already recording

    # Start recorder object
    recorder = RosbagControlledRecorder(rosbag_command, record_from_startup)

    

    # Recording is also stopped on node shutdown. This allows stopping to be done via service call or regular Ctrl-C
    rospy.on_shutdown(recorder.stop_recording_srv)

    while not rospy.is_shutdown():
        #if recorder.recording_stopped:  # stop main node if recording has finished
            #break
        rospy.sleep(1.0)
