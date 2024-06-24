import numpy as np
import copy
import math
import rclpy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class ReactiveFollowGap:
    BUBBLE_RADIUS = 1
    PREPROCESS_CONV_SIZE = 5
    BEST_POINT_CONV_SIZE = 80
    STRAIGHTS_STEERING_ANGLE = np.pi / 12  # 10 degrees
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_SPEED = 5.0
    CORNERS_SPEED = 3.0

    def __init__(self):
        self.node = rclpy.create_node("reactive_node")

        self.lidar_subscription = self.node.create_subscription(
            LaserScan, '/scan', self.scan_callback, 1)
        self.odom_subscription = self.node.create_subscription(
            Odometry, '/pf/pose/odom', self.odom_callback, 1)
        self.drive_publisher = self.node.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.ackermann_data = AckermannDriveStamped()
        self.ackermann_data.drive.acceleration = 0.0
        self.ackermann_data.drive.jerk = 0.0
        self.ackermann_data.drive.steering_angle = 0.0
        self.ackermann_data.drive.steering_angle_velocity = 0.0
        self.ackermann_data.drive.speed = 0.0

        self.current_odom_x = 0.0
        self.current_odom_y = 0.0

    def odom_callback(self, odom_msg):
        self.current_odom_x = odom_msg.pose.pose.position.x
        self.current_odom_y = odom_msg.pose.pose.position.y
      
    def scan_callback(self, scan_msg):
        self.radians_per_elem = (2 * np.pi) / len(scan_msg.ranges)
        proc_ranges = np.array(scan_msg.ranges[135:-135])
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)

        left = scan_msg.ranges[720]
        right = scan_msg.ranges[380]
        step = scan_msg.ranges[540]
        
        closest = proc_ranges.argmin()
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0:
            min_index = 0
        if max_index >= len(proc_ranges):
            max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        gap_start, gap_end = self.find_max_gap(proc_ranges)
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        angle = self.get_angle(best, len(proc_ranges)) - (0.15 * (0.4 / left)) + (0.15 * (0.4 / right))
        
        
        
        xgoal= 3.840
        ygoal=-4.696
        a=math.atan((xgoal-self.current_odom_x)/(ygoal-self.current_odom_y))
        LFD=1.9
        true_angle=math.atan((2*0.25*math.sin(a))/LFD)
        
        
        xgoal1= -0.355
        ygoal1=-1.490
        b=math.atan((xgoal1-self.current_odom_x)/(ygoal1-self.current_odom_y))
        LFD1=20.0
        true_angle1=math.atan((2*0.25*math.sin(b))/LFD1)
            # Check if the current position is within the specified range
        if 0.508<= self.current_odom_x <=4.303 and -6.556 <= self.current_odom_y <= -4.819:
          velocity=5.0
          angle=true_angle
          print(true_angle)

 
        
            # Check if the current position is within the specified range
        elif -3.459<= self.current_odom_x <=-0.884 and -2.604 <= self.current_odom_y <= -1.368:
          velocity=4.0
          angle=true_angle1
          print(true_angle1)
     
      #  elif  -5.5<= self.current_odom_x <= 3.8 and 2.2<= self.current_odom_y <= 3.2 :
      #  
      #    if step >= 5.0:
      #        if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
      #            velocity = 4.5
      #        else:
      #            velocity = 10.0
      #    elif 5.0 > step >= 3.5:
      #        if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
      #            velocity = 4.5
      #        else:
      #            velocity = 7.0 * (step / 5.0)
      #            if velocity > 7.0:
      #                velocity = 7.0
      #    elif 3.5 > step >= 0.4:
      #        if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
      #            velocity = 2.5
      #        else:
      #            velocity = 2.5 * (step / 3.5) + 1.5
      #            if velocity > 3.5:
      #                velocity = 3.5
        
        else :
        
          if step >= 8.0:
              if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
                  velocity = 5.0
              else:
                  velocity = 9.0
          elif 8.0 > step >= 5.0:
              if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
                  velocity = 5.0
              else:
                  velocity = 7.5 * (step / 8.0)
                  if velocity > 7.5:
                      velocity = 7.5
          elif 5.0 > step >= 2.0:
              if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
                  velocity = 5.0
              else:
                  velocity = 6.0 * (step / 5.0)
                  if velocity > 6.0:
                      velocity = 6.0
          elif 2.0 > step >= 0.0:
              if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
                  velocity = 2.0
              else:
                  velocity = 2.0 * (step / 2.0) 
                  if velocity > 2.0:
                      velocity = 2.0
         
        self.ackermann_data.drive.speed = velocity 
        self.ackermann_data.drive.steering_angle = angle
        self.ackermann_data.drive.steering_angle_velocity = 0.0
        self.ackermann_data.drive.acceleration = 0.0
        self.ackermann_data.drive.jerk = 0.0
        self.drive_publisher.publish(self.ackermann_data)

    def find_max_gap(self, free_space_ranges):
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        averaged_max_gap = np.convolve(
            ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE), 'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2
        return steering_angle

if __name__ == '__main__':
    rclpy.init()
    print("FGM Initialized")
    fgm_node = ReactiveFollowGap()
    rclpy.spin(fgm_node.node)
    rclpy.shutdown()
