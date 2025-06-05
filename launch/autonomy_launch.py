import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
  pkg_amr_bt = get_package_share_directory('amr_bt')

  autonomy_node_cmd = Node(
      package="amr_bt",
      executable="autonomy_node",
      name="autonomy_node",
      parameters=[{
          "location_file": os.path.join(pkg_amr_bt, "config", "station_locations.yaml")
      }]
  )

  battery_sim_node = Node(
      package="amr_bt",
      executable="battery_sim_node",
      name="battery_sim_node",
  )

  ld = LaunchDescription()

  ld.add_action(autonomy_node_cmd)
  ld.add_action(battery_sim_node)

  return ld