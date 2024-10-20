#!/usr/bin/env python3

import rospy
import logging

class ErrorHandler:
    def __init__(self):
        self.logger = logging.getLogger('error_handler')
        self.logger.setLevel(logging.ERROR)
        
        rospy.loginfo("Error Handler initialized")

    def handle_error(self, error_type, error):
        self.logger.error(f"{error_type}: {error}")

if __name__ == '__main__':
    try:
        handler = ErrorHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
      
