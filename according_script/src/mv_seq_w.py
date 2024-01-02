#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations as tf_trans

from math import *

import sys
sys.path.append('/home/sangpham/catkin_ws/src/ros_mqtt/src')  # Path to the directory containing define_mqtt.py
from define_mqtt import DefineMqtt


# Dữ liệu AI gửi đến
AI_data = None
distance_AI = None
angle_AI = None

interrupt_flag = False  # Cờ ngắt Topic subscriber AI
position_flag = None    # Cờ ngắt mảng landmarks

# Khai báo mảng các điểm mốc
landmarks = [
    {'land': 'nhà ăn', 'pos': {'x':  0.56, 'y': -0.78}, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 0.707, 'r4': 0.707}},
    {'land': 'nhà ngủ', 'pos': {'x':  0.04, 'y':  2.70}, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 1.000, 'r4': -1.000}},
    {'land': 'nhà nghỉ', 'pos': {'x': -3.30, 'y': 1.220}, 'quat': {'r1': 0.000, 'r2': 0.000, 'r3': 1, 'r4': 1}},
]

# Hàm subscribe dữ liệu AI gửi đến
def callback(data):
    global interrupt_flag, AI_data, distance_AI, angle_AI
    if not interrupt_flag:
        AI_data = data.data
        AI_data_dict = eval(AI_data)

        distance_AI = AI_data_dict['distance']
        angle_AI = AI_data_dict['angle']

        print("hihi")
        interrupt_flag = True

# Hàm sắp xếp lại mảng các cột mốc
def shift_array_elements(arr, first_element):
    if first_element in arr:
        index = arr.index(first_element)

        # Sử dụng slicing để tạo mảng mới với phần tử đầu tiên là first_element
        shifted_array = arr[index:] + arr[:index]

        # Gán giá trị của shifted_array vào mảng ban đầu
        arr[:] = shifted_array  # arr[:] có thể được hiểu là một con trỏ
    else:
        print("The first element does not exist in the array.")


class GoToPose():
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server()

        self.define = DefineMqtt()
        
        self.mqtt_sub_signal = rospy.get_param('mqtt_sub_signal', 'user/request')

        self.define.define()
        self.define.mqtt_client.on_message = self.on_mqtt_message
        self.define.mqtt_client.subscribe(self.mqtt_sub_signal)
        self.define.mqtt_client.loop_start()
        self.run_request = None # Các yêu cầu bắt đầu/ kết thúc tương tác được nhận từ MQTT

        self.amcl_yaw_euler = None
        self.amcl_position_x = None
        self.amcl_position_y =None

        self.amcl_flag = False  # Cờ ngắt topic /amcl_pose


    # Decode giá trị MQTT
    def on_mqtt_message(self, client, userdata, msg):
        self.run_request = msg.payload.decode()

    # Subscribe /amcl_pose
    def amcl_callback(self, msg):
        if self.amcl_flag:
            amcl_data = msg
            # Biến đổi từ quaternion sang euler
            amcl_angle_euler = tf_trans.euler_from_quaternion([amcl_data.pose.pose.orientation.x, amcl_data.pose.pose.orientation.y,
                                                                    amcl_data.pose.pose.orientation.z, amcl_data.pose.pose.orientation.w])
            
            self.amcl_yaw_euler = amcl_angle_euler[2]
            self.amcl_position_x = amcl_data.pose.pose.position.x
            self.amcl_position_y = amcl_data.pose.pose.position.y

            self.amcl_flag = False
            return
        
    # Chờ đợi yêu cầu khi robot đến vị trí người dùng
    def duration(self):
        for i in range(15): # Chờ 15 giây sau khi đến vị trí người dùng
            if (int(i) % 2) == 1:
                print("Chờ tương tác")
            if self.run_request is not None and self.run_request.lower() == "interact":
                print("Có tương tác, đứng đợi tiếp")
                while True:
                    if self.run_request is not None and self.run_request.lower() == "end":
                        self.run_request = None
                        rospy.sleep(1)
                        print("Hết chuyện, én")
                        break
                break
            rospy.sleep(1)
        else:
            print("Lâu vãi nồi, thôi đi")
            main()
        main()

    # Tạo điểm goal
    def create_goal(self, pos, quat):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.0),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        return goal
    
    # Thực hiện hành động di chuyển
    def send_goal(self, goal):
        global interrupt_flag
        if not interrupt_flag:
            self.sub = rospy.Subscriber('ex_pub', String, callback)
        self.move_base.send_goal(goal)

    # Chờ đợi kết quả di chuyển
    def wait_for_result(self):
        while not rospy.is_shutdown() and not interrupt_flag:
            if self.move_base.get_state() == actionlib.GoalStatus.ACTIVE:
                if self.move_base.wait_for_result(rospy.Duration(1.0)):
                    break

    def cancel_goal(self):
        if self.move_base.get_state() == actionlib.GoalStatus.ACTIVE:
            self.move_base.cancel_goal()
            rospy.sleep(1)
            self.goto_user()   

    # Hàm thực hiện di chuyển robot đến vị trí người dùng khi phát hiện yêu cầu tương tác
    def goto_user(self):
        self.amcl_flag = True
        rospy.sleep(0.1)

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        rospy.sleep(0.1)    # cần có để đảm bảo nhận được dữ liệu từ topic /amcl_pose
        
        if self.amcl_yaw_euler is not None and angle_AI is not None:
            # Đảm bảo tất cả giá trị góc có đơn vị là radian
            goal_angle = self.amcl_yaw_euler + radians(angle_AI)
            print("hohohoho")
            quat_angle_AI = tf_trans.quaternion_from_euler(0, 0, goal_angle)
            # Công thức tính vị trị của người yêu cầu tương tác
            new_position_x = self.amcl_position_x + (distance_AI - 0.4) * cos(goal_angle)   # 0.4 Là một hằng số để đảm bảo robot đứng cách người tương tác một khoảng an toàn
            new_position_y = self.amcl_position_y + (distance_AI - 0.4) * sin(goal_angle)
            
            person_pose = {'x':  new_position_x, 
                        'y': new_position_y, 
                        'quat': {'r1': quat_angle_AI[0], 'r2': quat_angle_AI[1], 
                                'r3': quat_angle_AI[2], 'r4': quat_angle_AI[3]}}

            print(f"Phát hiện con hàng ở tọa độ: {round(new_position_x, 2), round(new_position_y, 2)}")
            print("Gooooo!!!!!!!!!")

            goal = self.create_goal(person_pose, person_pose['quat'])
            self.send_goal(goal)
            result = self.move_base.wait_for_result()
            if result:
                global interrupt_flag, landmarks
                print("It's delicious")
                shift_array_elements(landmarks, position_flag)
                # interrupt_flag = False
                self.duration()

    # Hàm trả về kết quả action move_base
    def get_result(self):
        rospy.sleep(0.1)
        return self.move_base.get_result()
    
    # Thực hiện kịch bản chương trình
    def navigate_to(self, pos, quat):
        goal = self.create_goal(pos, quat)
        self.send_goal(goal)
        self.wait_for_result()
        # if interrupt_flag:
        self.cancel_goal()
        return self.get_result()

def main():
    global position_flag, interrupt_flag
    navigator = GoToPose()
    interrupt_flag = False

    while not interrupt_flag:
        for position in landmarks:
            rospy.loginfo(f"Đang đến {position['land']}")
            position_flag = position
            success = navigator.navigate_to(position['pos'], position['quat'])

            if interrupt_flag:
                rospy.loginfo("Received cancel signal. Stopping the robot.")
                break 

            if success:
                rospy.loginfo(f"Đã đến {position['land']}")
            else:
                rospy.loginfo(f"Đếu đến {position['land']}")
                break
            rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('nav_test', anonymous=False)
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException. Quitting")
