from launch import LaunchDescription
from launch_ros.actions import Node

# def generate_launch_description():
    
#     app_controller_node = Node(
#         package     = 'adisha_controllers',
#         executable  = 'app_controller',
#         name        = f'{ROBOT_ID}_app_controller',
#         parameters  = [
#             {'id': ROBOT_ID},
#             {'dxl_baudrate': DXL_BAUDRATE},
#             {'dxl_u2d2_port': DXL_U2D2_PORT},
#             {'dxl_num': DXL_NUM},
#             {'dxl_id': DXL_ID},
#             {'dxl_type': DXL_TYPE},
#             {'joint_name': JOINT_NAME},
#             {'master_clock': MASTER_CLOCK}
#         ]
#     )

#     app_launcher_node = Node(
#         package     = 'adisha_interfaces',
#         executable  = 'app_launcher',
#         name        = f'{ROBOT_ID}_app_launcher'
#     )

#     return LaunchDescription([
#         app_controller_node,
#         app_launcher_node
#     ])

