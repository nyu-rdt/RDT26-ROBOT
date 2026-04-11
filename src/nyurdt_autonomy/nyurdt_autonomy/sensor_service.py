# This is dead code???
# Seems to be copied from a previous year

from nyurdt_interfaces.srv import SensorData
from nyurdt_command_handler.i2c_comms import get_data_dictionary

import rclpy
from rclpy.node import Node


class SensorService(Node):

    def __init__(self):
        super().__init__('sensor_service')
        self.srv = self.create_service(SensorData, 'get_sensor_data', self.sensor_data_callback)

    def sensor_data_callback(self, request, response: SensorData) -> SensorData:
        """
        Updates the SensorData.srv database to current robot status
        sensor dictionary has to be updated later from EE 

        :param request: Incoming request information (NOT USED, NEEDED FOR SYNTAX)
        :param response: The outgoing response to the request
        :return: SensorData
        """
        ### Get the data dictionary from EE
        sensor_data_dict = get_data_dictionary()
        
        # Log incoming request
        self.get_logger().info("Incoming sensor data request")

        response.at_locomotion_position = sensor_data_dict["at_locomotion_position"]
        response.at_excavation_position = sensor_data_dict["at_excavation_position"]
        
        return response
    


def main(args=None):
    rclpy.init(args=args)

    sensor_service = SensorService()

    rclpy.spin(sensor_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
