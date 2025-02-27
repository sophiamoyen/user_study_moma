#!/usr/bin/env python3

import yaml

import rospy
import rospkg
import rosparam
from controller_manager_msgs.srv import SwitchController, LoadController, ListControllers


def call_service(proxy, num_tries, *args):
    """Calls ServiceProxy safelty num_tries times or infinitely until exception-free execution.
    Requires rosnode to be set up before calling.
    Args:
        proxy (rospy.ServiceProxy): Service Proxy to call
        num_tries (int): number of tries to call Service in case it throws Exception. if -1, will try indefinetely
        *args : arguments to call Service with
    """
    rospy.wait_for_service(proxy.resolved_name)

    success = False
    iter = 0
    resp = None
    while (not success and iter != num_tries):
        try:
            resp = proxy(*args)
            success = True
        except rospy.ServiceException as e:
            print(f"Service call to {proxy.resolved_name} failed: {e}")
            iter += 1
            rospy.sleep(1)

    return resp

def load_yaml_files_onto_server(paths, package=None, ns='', verbose=False):
    """
    Args:
        paths (list of str): Path to load yamls from; if package is given, it will be relative to `rospack find <package>`
        package (str): name of package to look for yaml
        ns (str): namespace †o load parameters onto param server
        verbose (bool): whether to print success statement
    """
    if package is not None:
        rospack = rospkg.RosPack()

        try:
            package_path = rospack.get_path(package)
        except rospkg.common.ResourceNotFound:
            print(f"Error: ROS can't find package `{package}`. Make sure it's sourced!")
            return False
    else:
        package_path = ''

    for path in paths:
        if ns == '':
            print(f'Warning: loading params of file {package_path + path} onto param_sever without namespace!!')

        with open(package_path + path, 'r') as f:
            file_content = yaml.load(f, yaml.Loader)
            rosparam.upload_params(ns, file_content)

        if verbose:
            print(f'File {package_path + path} has successfully been loaded onto the param server!')

    return True



def load_imp_files_for_side(side, gripper, mode):
    """
    Args:
        side (str): 'left' or 'right'
        gripper (str): 'robotiq-2f-85' or 'pal-gripper'
        mode (str): 'sim' or 'robot'
    Returns:
        bool: package_found
    """
    file_list = []
    file_list.append(f'/config/tiago_dual_arm_{side}_impedance_controller_basic_config.yaml')
    file_list.append(f'/config/tiago_dual_arm_{side}_{gripper}_kin_chain.yaml')
    file_list.append(f'/config/tiago_dual_arm_{side}_motor_params_{mode}.yaml')

    return load_yaml_files_onto_server(file_list, package='joint_trajectory_impedance_controller', ns=f'iRosa_arm_{side}_controller')

def load_controller(side):
    """
    Args:
        side (str): 'left' or 'right'
    Returns:
        package_found: whether the necessary package with config files was found, i.e. whether it makes sense to try to load the other side's controller
    """
    gripper = 'pal-gripper'
    mode = 'robot'

    # load config
    package_found = load_imp_files_for_side(side, gripper, mode)
    if not package_found:
        print("Impedance Control got disabled for your `TiagoDual`, because package `joint_trajectory_impedance_controller` couldn't be found.")
        return False

    # load controller
    load_controller_proxy = rospy.ServiceProxy("controller_manager/load_controller", LoadController)

    resp = call_service(load_controller_proxy, 10, f'iRosa_arm_{side}_controller')
    if resp is not None:
            ok = resp.ok

    if not ok:
        print(f"Failed to initialize 'iRosa_arm_{side}_controller'.")
    else:
        print(f"'iRosa_arm_{side}_controller' successfully initialized!")
        
def activate_arm_compliance(side):
    """
    Args:
        side (str): can be 'left' or 'right'
    Returns:
        success (bool)
    """
    switch_controller = rospy.ServiceProxy("controller_manager/switch_controller", SwitchController)

    
    
    resp = call_service(switch_controller, 10, [f'iRosa_arm_{side}_controller'], [f"arm_{side}_controller"], 0)
    if not resp.ok:
        print(f"Failed to switch from position controller 'arm_{side}_controller to impedance controller 'iRosa_arm_{side}_controller'.")
        return False
    else:
        print(f"Successfully switched from position controller 'arm_{side}_controller' to impedance controller 'iRosa_arm_{side}_controller'. \
        You can now send commands to it.")
        return True
        
if __name__ == '__main__':
    rospy.init_node('load_imp_controller')
    load_controller('left')
    load_controller('right')
    activate_arm_compliance('left')
    activate_arm_compliance('right')