#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, atan2, sqrt, fabs, pi
import random
import time

# Hàm tính sự khác biệt giữa hai góc a và b, với góc được chuẩn hóa trong khoảng [-pi, pi].
def angle_diff(a, b):
    a = atan2(sin(a), cos(a))
    b = atan2(sin(b), cos(b))
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if(d1 > 0):
        d2 *= -1.0
    if(fabs(d1) < fabs(d2)):
        return d1
    else:
        return d2 

# Lớp OdometryMotionModel kế thừa từ Node, mô hình chuyển động dựa trên dữ liệu odometry.
class OdometryMotionModel(Node):

    def __init__(self):
        super().__init__("odometry_motion_model")
        self.is_first_odom = True # Biến cờ xác định lần đầu tiên nhận dữ liệu 
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0

        #khai báo mức độ ảnh hưởng của nhiễu trong mô hình
        self.declare_parameter("alpha1", 0.1)
        self.declare_parameter("alpha2", 0.1)
        self.declare_parameter("alpha3", 0.1)
        self.declare_parameter("alpha4", 0.1)
        #số lượng mẫu (samples) sẽ được tạo ra
        self.declare_parameter("nr_samples", 300)

        #lấy giá trị các tham số trước đó
        self.alpha1 = self.get_parameter("alpha1").get_parameter_value().double_value
        self.alpha2 = self.get_parameter("alpha2").get_parameter_value().double_value
        self.alpha3 = self.get_parameter("alpha3").get_parameter_value().double_value
        self.alpha4 = self.get_parameter("alpha4").get_parameter_value().double_value
        self.nr_samples = self.get_parameter("nr_samples").get_parameter_value().integer_value


        if self.nr_samples >= 0:
            #tạo 1 mảng số lượng Pose cùng với số lượng mẫu
            self.samples = PoseArray()
            self.samples.poses = [Pose() for _ in range(self.nr_samples)]
        else:
            self.get_logger().fatal("Invalid number of samples requested: %d", self.nr_samples)
            return
            

        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom_noisy", self.odomCallback, 10)
        # khai báo hàm callback để nhận odometry từ dom_noisy

        self.pose_array_pub_ = self.create_publisher(PoseArray, "odometry_motion_model/samples", 10)
        #Tạo publisher

    def odomCallback(self, odom): #xử lý khi nhận được tín hiệu odometry
        # Lấy giá trị quaternion từ odometry
        q = [odom.pose.pose.orientation.x, 
             odom.pose.pose.orientation.y, 
             odom.pose.pose.orientation.z, 
             odom.pose.pose.orientation.w]
        
        # Chuyển đổi quaternion sang góc Euler (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_quaternion(q)

        # Kiểm tra nếu đây là lần đầu tiên nhận odometry
        if self.is_first_odom:
            # Lưu vị trí và góc hiện tại
            self.last_odom_x = odom.pose.pose.orientation.x
            self.last_odom_y = odom.pose.pose.orientation.y
            self.last_odom_theta = yaw

            # Gán frame_id cho samples
            self.samples.header.frame_id = odom.header.frame_id
            self.is_first_odom =False
            return
        
        # Tính toán các gia số thay đổi trong odometry
        odom_x_increment = odom.pose.pose.position.x - self.last_odom_x
        odom_y_increment = odom.pose.pose.position.y - self.last_odom_y
        odom_theta_increment = angle_diff(yaw, self.last_odom_theta)

        # Nếu khoảng cách di chuyển rất nhỏ, đặt delta_rot1 là 0
        if sqrt(pow(odom_y_increment, 2) + pow(odom_x_increment, 2)) < 0.01:
            delta_rot1 = 0.0
        else:
            # Tính delta_rot1 (góc quay đầu) dựa trên thay đổi vị trí
            delta_rot1 = angle_diff(atan2(odom_y_increment, odom_x_increment), yaw)

        #Tính toán khoảng cách di chuyển và góc quay còn lại
        delta_trasl = sqrt(pow(odom_y_increment, 2) + pow(odom_x_increment, 2))
        delta_rot2 = angle_diff(odom_theta_increment, delta_rot1)

        # Tính toán độ biến thiên (variance) cho từng phần
        rot1_variance = self.alpha1 * delta_rot1+ self.alpha2 * delta_trasl
        trasl_variance = self.alpha3 * delta_trasl + self.alpha4 * (delta_rot1 + delta_rot2)
        rot2_variance = self.alpha1 * delta_rot2 + self.alpha2 * delta_trasl

        # Gán seed cho random để tạo nhiễu
        random.seed(int(time.time()))
        # Tạo mẫu (samples) dựa trên biến động
        for sample in self.samples.poses:
            # Tạo nhiễu cho từng mẫu
            rot1_noise = random.gauss(0.0, rot1_variance)
            trasl_noise = random.gauss(0.0, rot1_variance)
            rot2_noise = random.gauss(0.0, rot1_variance)

            # Tính toán các giá trị đã rút trừ cho từng mẫu
            delta_rot1_draw = angle_diff(delta_rot1, rot1_noise)
            delta_trans_draw = delta_trasl - trasl_noise
            delta_rot2_draw = angle_diff(delta_rot2, rot2_noise)

            # Lấy quaternion của mẫu hiện tại
            sample_q = [sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w]

            # Chuyển đổi quaternion của mẫu thành góc Euler
            sample_roll, sample_pitch, sample_yaw = euler_from_quaternion(sample_q)
            
            # Cập nhật vị trí cho mẫu
            sample.position.x += delta_trans_draw * cos(sample_yaw + delta_rot1_draw)
            sample.position.y += delta_trans_draw * sin(sample_yaw + delta_rot1_draw)
            
            # Cập nhật quaternion cho mẫu
            quaternion_from_euler(0.0, 0.0, sample_yaw + delta_rot1_draw + delta_rot2_draw)
            sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w = q

        # Cập nhật các giá trị odometry cuối cùng
        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw

        # Xuất các mẫu đã tạo ra
        self.pose_array_pub_.publish(self.samples)



def main():
    rclpy.init()

    odometry_motion_model = OdometryMotionModel()
    rclpy.spin(odometry_motion_model)
    
    odometry_motion_model.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()