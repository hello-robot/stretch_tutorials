from __future__ import print_function
import stretch_body.hello_utils
import stretch_body.robot_params
import argparse
from os.path import exists
import sys

model_name=stretch_body.robot_params.RobotParams()._robot_params['robot']['model_name']

if model_name=='RE2V0':
    dex_wrist_yaml = {
    'robot': {'use_collision_manager': 1, 'tool': 'tool_stretch_gripper'},
    'params': [],
    'stretch_gripper': {'range_t': [0, 8022], 'zero_t': 5212}, 
    'lift': {'i_feedforward': 1.2},
    'hello-motor-lift': {'gains': {'i_safety_feedforward': 1.2}}}
if model_name == 'RE1V0':
    dex_wrist_yaml = {
    'robot': {'use_collision_manager': 1, 'tool': 'tool_stretch_gripper'},
    'params': [],
    'stretch_gripper': {'range_t': [0, 8022], 'zero_t': 5212}, 
    'lift': {'i_feedforward': 0.54},
    'hello-motor-lift': {'gains': {'i_safety_feedforward': 0.4}}}

if not exists(stretch_body.hello_utils.get_fleet_directory()+'stretch_configuration_params.yaml'):
    print('Please run tool RE1_migrate_params.py before continuing. For more details, see https://forum.hello-robot.com/t/425')
    sys.exit(1)

configuration_yaml=stretch_body.hello_utils.read_fleet_yaml('stretch_configuration_params.yaml')
stretch_body.hello_utils.overwrite_dict(overwritee_dict=configuration_yaml, overwriter_dict=dex_wrist_yaml)
stretch_body.hello_utils.write_fleet_yaml('stretch_configuration_params.yaml', configuration_yaml,
                                          header=stretch_body.robot_params.RobotParams().get_configuration_params_header())
