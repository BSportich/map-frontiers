#!/usr/bin/env python3
# coding: utf-8

import rospy
from std_srvs.srv import Empty

class NodeLauncher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('node_launcher', anonymous=True)
        self._rate = rospy.Rate(0.1)
        self._has_called_services = False

        # Get parameters
        # Expecting topics as follow. /droneName_NUMBER/serviceName
        # FIXME: this architecture forces the param /drone_name to be already loaded to the rosparams
        drone_name = rospy.get_param('/drone_name')
        self._topic_namespace = "/" + drone_name + "_"
        self._service_name = rospy.get_param('~service_name', "start_NBV_selector")
        self._number_of_drones = rospy.get_param('/number_of_drones')

        # Service server
        self._launch_service = rospy.Service('/start_voxblox_nodes', Empty, self._LaunchVoxbloxNodeCallback)

        # Service clients
        self._service_clients = list()

        # Add voxblox to the service client list
        voxblox_service_name = "/voxblox_node/activate_node"
        rospy.wait_for_service(voxblox_service_name)
        self._service_clients.append(rospy.ServiceProxy(voxblox_service_name, Empty))

        # Should work for N number of service
        for drone_id in range(self._number_of_drones):
            full_service_name = self._topic_namespace + str(drone_id) + '/' + self._service_name
            # Wait for the service to be available
            rospy.wait_for_service(full_service_name)
            rospy.loginfo("[node_launcher] Added service %s to the list of services to call", full_service_name)
            self._service_clients.append(rospy.ServiceProxy(full_service_name, Empty))
            

    def _LaunchVoxbloxNodeCallback(self, req):
        self._CallServiceClients()
        rospy.loginfo("[node_launcher] Done calling services")
        return []
    
    def _CallServiceClients(self):
        for client in self._service_clients:
            try:
                client() # Call the service
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

    def Spin(self):
        while not rospy.is_shutdown():
            if self._has_called_services:
                rospy.signal_shutdown("Shutdown initiated from signal.")
            else:
                self._rate.sleep()

if __name__ == '__main__':
    try:
        node_laucher = NodeLauncher()
        node_laucher.Spin()
    except rospy.ROSInterruptException:
        pass
