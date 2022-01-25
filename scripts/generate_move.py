#!/usr/bin/env python
from geopy import quart_to_rpy, euler_to_quaternion, euclid_distance
import geopy as gp
import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from water_msgs.msg import ShipState
import os
import _thread
import math
import time
import numpy as np
basedir = os.path.abspath(os.path.dirname(__file__))

pier_free_to_send_ship_statistics = dict()


class Ship:
    def __init__(self, name, ship_type):
        self.name = name
        self.type = ship_type
        self.g_set_state = None
        self.g_get_state = None
        self.ros_srv_init()

        self.park_state = ModelState()
        self.latest_state = ShipState()

        self.route = None
        self.route_point_idx = 0
        self.shipping = False
        self.ending = False

        self.distance_to_target_route_point = None
        self.rad_bet_ori_and_route = None

        self.latest_twist_linear_x = 0
        self.latest_twist_angular_z = 0

        # control params
        self.kp_linear_velocity_1 = 1
        self.kp_linear_velocity_2 = 1
        self.kp_linear_velocity_3 = 1
        self.kp_angular_velocity_1 = 1
        self.kp_angular_velocity_2 = 1
        self.initialize_ship_param(self.name)

    def ros_srv_init(self):
        self.g_set_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        self.g_get_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)

        # global set_state_pub
        # set_state_pub = rospy.Publisher(
        #     '/gazebo/set_model_state', ModelState, queue_size=20)

    def initialize_ship_param(self, name):
        park_x = rospy.get_param(f"{name}/x")
        park_y = rospy.get_param(f"{name}/y")
        print(f"loading {self.name}, park position: ", park_x, ",", park_y)
        self.park_state.model_name = name
        self.park_state.pose.position.x = park_x
        self.park_state.pose.position.y = park_y

        self.kp_linear_velocity_1 = rospy.get_param(
            f"ships/{self.type}/ships_param/kp_linear_velocity_1")
        self.kp_linear_velocity_2 = rospy.get_param(
            f"ships/{self.type}/ships_param/kp_linear_velocity_2")
        self.kp_linear_velocity_3 = rospy.get_param(
            f"ships/{self.type}/ships_param/kp_linear_velocity_3")
        self.kp_angular_velocity_1 = rospy.get_param(
            f"ships/{self.type}/ships_param/kp_angular_velocity_1")
        self.kp_angular_velocity_2 = rospy.get_param(
            f"ships/{self.type}/ships_param/kp_angular_velocity_2")

    def update_latest_state(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        try:
            model_state = self.g_get_state(model_name=self.name)
        except Exception as e:
            rospy.logerr('Error on calling service: %s', str(e))
            return
        latest_state = ShipState()
        latest_state.header = header
        latest_state.modelstate = model_state
        self.latest_state = latest_state

    def update_latest_route_state(self):
        px = self.latest_state.modelstate.pose.position.x
        py = self.latest_state.modelstate.pose.position.y
        tx = self.route.route_points[self.route_point_idx+1][0]
        ty = self.route.route_points[self.route_point_idx+1][1]
        qx = self.latest_state.modelstate.pose.orientation.x
        qy = self.latest_state.modelstate.pose.orientation.y
        qz = self.latest_state.modelstate.pose.orientation.z
        qw = self.latest_state.modelstate.pose.orientation.w
        self.distance_to_target_route_point = euclid_distance(px, py, tx, ty)

        orientation_yaw = quart_to_rpy(qx, qy, qz, qw)[2]
        route_yaw = math.atan2(ty-py, tx-px)
        self.rad_bet_ori_and_route = orientation_yaw - route_yaw
        if abs(self.rad_bet_ori_and_route) > 3.1414926:
            if self.rad_bet_ori_and_route > 0:
                self.rad_bet_ori_and_route = - \
                    (2 * 3.1414926 - abs(self.rad_bet_ori_and_route))
            else:
                self.rad_bet_ori_and_route = 2 * 3.1414926 - \
                    abs(self.rad_bet_ori_and_route)

    def to_target_point(self):
        motion_state_msg = ModelState()
        motion_state_msg.model_name = self.name
        if abs(self.rad_bet_ori_and_route) > 0.1:
            motion_state_msg.twist.linear.x = 0.3*self.kp_linear_velocity_1
            motion_state_msg.twist.angular.z = -180 / \
                180 * 3.14 * self.rad_bet_ori_and_route * self.kp_angular_velocity_1
        else:
            set_velocity = 7.5 * self.kp_linear_velocity_2
            if self.distance_to_target_route_point > 5:
                set_velocity = 15 * self.kp_linear_velocity_3
            motion_state_msg.twist.linear.x = set_velocity
            motion_state_msg.twist.angular.z = -180 / \
                180 * 3.14 * self.rad_bet_ori_and_route * self.kp_angular_velocity_2
        self.latest_twist_linear_x = motion_state_msg.twist.linear.x
        self.latest_twist_angular_z = motion_state_msg.twist.angular.z
        motion_state_msg.reference_frame = self.name
        try:
            resp = self.g_set_state(motion_state_msg)
        except rospy.ServiceException as e:
            print(f"[{self.name}] Service call failed: {e}")
        # pub.publish(motion_state_msg)

    def route_main(self):
        global pier_free_to_send_ship_statistics
        set_time = 0
        while True:
            self.update_latest_state()
            self.update_latest_route_state()
            # print(f"[{self.route_point_idx}] target point:({self.route.route_points[self.route_point_idx+1][0]:.2f}, {self.route.route_points[self.route_point_idx+1][1]:.2f})")
            # print(
            #     f"    current position:({self.latest_state.modelstate.pose.position.x:.2f}, {self.latest_state.modelstate.pose.position.y:.2f})")
            # print(
            #     f"    control state: target distance: {self.distance_to_target_route_point:.2f}m, target angle: {self.rad_bet_ori_and_route:.4f}rad")
            # print(
            #     f"    control ouput: twist angular x: {self.latest_twist_linear_x:.2f}, twist_angular_z: {self.latest_twist_angular_z:.4f}\n")
            if self.route_point_idx < 2:
                pier_free_to_send_ship_statistics[self.route.name] = False
            elif set_time == 0:
                pier_free_to_send_ship_statistics[self.route.name] = True
                set_time += 1
            if self.distance_to_target_route_point < 3.5:
                # print(
                #     f"[{self.route.name}] {self.name} has comed to {self.route_point_idx+1} idx.")
                self.route_point_idx += 1
                if self.route_point_idx >= self.route.route_length - 1:
                    break
            self.to_target_point()
            time.sleep(0.2)

        # print(f"[{self.route.name}] {self.name} finished one trip {self.route.name}！")
        try:
            resp = self.g_set_state(self.park_state)
        except rospy.ServiceException as e:
            print(f"[{self.name}] Service call failed: {e}")
        self.route_point_idx = 0
        self.shipping = False

    def set_route(self, route):
        self.route = route

    def put_object_to_start_pier(self):
        state_msg = ModelState()
        state_msg.model_name = self.name
        state_msg.pose.position.x = self.route.start_peir_position[0]
        state_msg.pose.position.y = self.route.start_peir_position[1]
        state_msg.pose.position.z = 0.1
        q = euler_to_quaternion(
            self.route.start_park_orietation[2], self.route.start_park_orietation[1], self.route.start_park_orietation[0])  # yaw, pitch, roll
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]
        try:
            resp = self.g_set_state(state_msg)
        except rospy.ServiceException as e:
            print(f"[{self.name}] Service call failed: {e}")

    def run_route(self):
        self.shipping = True
        _thread.start_new_thread(self.route_main, ())


class AutoShip:
    def __init__(self, name, ship_type):
        self.name = name
        self.type = ship_type
        self.g_set_state = None
        self.g_get_state = None
        self.ros_srv_init()

        self.park_state = ModelState()
        self.latest_state = ShipState()

        self.route = None
        self.route_point_idx = 0
        self.shipping = False
        self.ending = False

        self.distance_to_target_route_point = None
        self.rad_bet_ori_and_route = None

        self.latest_twist_linear_x = 0
        self.latest_twist_angular_z = 0

        # control params
        self.kp_linear_velocity_1 = 1
        self.kp_linear_velocity_2 = 1
        self.kp_linear_velocity_3 = 1
        self.kp_angular_velocity_1 = 1
        self.kp_angular_velocity_2 = 1
        self.initialize_ship_param(self.name)

        # new
        self.allship_state = dict()

    def ros_srv_init(self):
        self.g_set_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        self.g_get_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)

        # global set_state_pub
        # set_state_pub = rospy.Publisher(
        #     '/gazebo/set_model_state', ModelState, queue_size=20)

    def initialize_ship_param(self, name):
        park_x = rospy.get_param(f"{name}/x")
        park_y = rospy.get_param(f"{name}/y")
        print(f"loading {self.name}, park position: ", park_x, ",", park_y)
        self.park_state.model_name = name
        self.park_state.pose.position.x = park_x
        self.park_state.pose.position.y = park_y

        self.kp_linear_velocity_1 = rospy.get_param(
            f"ships/{self.type}/ships_param/kp_linear_velocity_1")
        self.kp_linear_velocity_2 = rospy.get_param(
            f"ships/{self.type}/ships_param/kp_linear_velocity_2")
        self.kp_linear_velocity_3 = rospy.get_param(
            f"ships/{self.type}/ships_param/kp_linear_velocity_3")
        self.kp_angular_velocity_1 = rospy.get_param(
            f"ships/{self.type}/ships_param/kp_angular_velocity_1")
        self.kp_angular_velocity_2 = rospy.get_param(
            f"ships/{self.type}/ships_param/kp_angular_velocity_2")

    def update_latest_state(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        try:
            model_state = self.g_get_state(model_name=self.name)
        except Exception as e:
            rospy.logerr('Error on calling service: %s', str(e))
            return
        latest_state = ShipState()
        latest_state.header = header
        latest_state.modelstate = model_state
        self.latest_state = latest_state

    def update_latest_route_state(self):
        px = self.latest_state.modelstate.pose.position.x
        py = self.latest_state.modelstate.pose.position.y
        tx = self.route.route_points[self.route_point_idx+1][0]
        ty = self.route.route_points[self.route_point_idx+1][1]
        qx = self.latest_state.modelstate.pose.orientation.x
        qy = self.latest_state.modelstate.pose.orientation.y
        qz = self.latest_state.modelstate.pose.orientation.z
        qw = self.latest_state.modelstate.pose.orientation.w
        self.distance_to_target_route_point = euclid_distance(px, py, tx, ty)

        orientation_yaw = quart_to_rpy(qx, qy, qz, qw)[2]
        route_yaw = math.atan2(ty-py, tx-px)
        self.rad_bet_ori_and_route = orientation_yaw - route_yaw
        if abs(self.rad_bet_ori_and_route) > 3.1414926:
            if self.rad_bet_ori_and_route > 0:
                self.rad_bet_ori_and_route = - \
                    (2 * 3.1414926 - abs(self.rad_bet_ori_and_route))
            else:
                self.rad_bet_ori_and_route = 2 * 3.1414926 - \
                    abs(self.rad_bet_ori_and_route)

    def to_target_point(self):
        motion_state_msg = ModelState()
        motion_state_msg.model_name = self.name
        if abs(self.rad_bet_ori_and_route) > 0.1:
            motion_state_msg.twist.linear.x = 0.3*self.kp_linear_velocity_1
            motion_state_msg.twist.angular.z = -180 / \
                180 * 3.14 * self.rad_bet_ori_and_route * self.kp_angular_velocity_1
        else:
            set_velocity = 7.5 * self.kp_linear_velocity_2
            if self.distance_to_target_route_point > 5:
                set_velocity = 15 * self.kp_linear_velocity_3
            motion_state_msg.twist.linear.x = set_velocity
            motion_state_msg.twist.angular.z = -180 / \
                180 * 3.14 * self.rad_bet_ori_and_route * self.kp_angular_velocity_2
        # Auto Breaking System control-------------
        abs_trigger = self.auto_breaking_system()
        if abs_trigger == True:
            print("Collision warning!")
            motion_state_msg.twist.linear.x = 0
            motion_state_msg.twist.angular.z = 0.5 * self.kp_angular_velocity_2  # 0
        # -----------------------------------------------------
        self.latest_twist_linear_x = motion_state_msg.twist.linear.x
        self.latest_twist_angular_z = motion_state_msg.twist.angular.z
        motion_state_msg.reference_frame = self.name
        try:
            resp = self.g_set_state(motion_state_msg)
        except rospy.ServiceException as e:
            print(f"[{self.name}] Service call failed: {e}")
        # pub.publish(motion_state_msg)

    # new------------------------------------
    def auto_breaking_system(self):
        if len(self.allship_state) == 0:
            return False
        else:
            mx_world = self.latest_state.modelstate.pose.position.x
            my_world = self.latest_state.modelstate.pose.position.y
            mqx = self.latest_state.modelstate.pose.orientation.x
            mqy = self.latest_state.modelstate.pose.orientation.y
            mqz = self.latest_state.modelstate.pose.orientation.z
            mqw = self.latest_state.modelstate.pose.orientation.w
            myaw = quart_to_rpy(mqx, mqy, mqz, mqw)[2]
            wmR = gp.euler_to_Rmatrix(myaw, 0, 0)
            wmT = gp.transform_from_rot_trans(
                wmR, np.array([mx_world, my_world, 0]))  # 4x4
            wmT = wmT[0:3, :]  # 3x4
            mwT = gp.inverse_rigid_trans(wmT)
            for target_name, target_state in self.allship_state.items():
                if target_name == self.name:
                    continue
                tx_world = target_state.modelstate.pose.position.x
                ty_world = target_state.modelstate.pose.position.y
                tp_world = np.array(
                    [tx_world, ty_world, 0, 1]).reshape((4, 1))  # 4x1
                tp_self = np.dot(mwT, tp_world)
                dis = math.sqrt(
                    math.pow(tp_self[0], 2) + math.pow(tp_self[1], 2))
                angle_degree = np.arctan2(
                    tp_self[1], tp_self[0]) / 3.1415926 * 180
                # print(
                #     f"{target_name}, {tx_world}, {ty_world}: {dis}, {angle_degree}")
                if dis < 15 and abs(angle_degree) < 35:
                    print(
                        f"[{self.name}]: {target_name}, ({float(tp_self[0])}m, {float(tp_self[1])}m), dis-{dis}m, angle-{float(angle_degree)}°")
                    return True
        return False
    # -------------------------------------------

    def route_main(self):
        global pier_free_to_send_ship_statistics
        set_time = 0
        while True:
            self.update_latest_state()
            self.update_latest_route_state()
            # print(f"[{self.route_point_idx}] target point:({self.route.route_points[self.route_point_idx+1][0]:.2f}, {self.route.route_points[self.route_point_idx+1][1]:.2f})")
            # print(
            #     f"    current position:({self.latest_state.modelstate.pose.position.x:.2f}, {self.latest_state.modelstate.pose.position.y:.2f})")
            # print(
            #     f"    control state: target distance: {self.distance_to_target_route_point:.2f}m, target angle: {self.rad_bet_ori_and_route:.4f}rad")
            # print(
            #     f"    control ouput: twist angular x: {self.latest_twist_linear_x:.2f}, twist_angular_z: {self.latest_twist_angular_z:.4f}\n")
            if self.route_point_idx < 2:
                pier_free_to_send_ship_statistics[self.route.name] = False
            elif set_time == 0:
                pier_free_to_send_ship_statistics[self.route.name] = True
                set_time += 1
            if self.distance_to_target_route_point < 3.5:
                print(
                    f"[{self.route.name}] {self.name} has comed to {self.route_point_idx+1} idx.")
                self.route_point_idx += 1
                if self.route_point_idx >= self.route.route_length - 1:
                    break
            self.to_target_point()
            time.sleep(0.2)

        print(f"[{self.route.name}] {self.name} finished one trip {self.route.name}！")
        try:
            resp = self.g_set_state(self.park_state)
        except rospy.ServiceException as e:
            print(f"[{self.name}] Service call failed: {e}")
        self.route_point_idx = 0
        self.shipping = False

    def set_route(self, route):
        self.route = route

    def put_object_to_start_pier(self):
        state_msg = ModelState()
        state_msg.model_name = self.name
        state_msg.pose.position.x = self.route.start_peir_position[0]
        state_msg.pose.position.y = self.route.start_peir_position[1]
        state_msg.pose.position.z = 0.1
        q = euler_to_quaternion(
            self.route.start_park_orietation[2], self.route.start_park_orietation[1], self.route.start_park_orietation[0])  # yaw, pitch, roll
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]
        try:
            resp = self.g_set_state(state_msg)
        except rospy.ServiceException as e:
            print(f"[{self.name}] Service call failed: {e}")

    def run_route(self):
        self.shipping = True
        _thread.start_new_thread(self.route_main, ())


class Route:
    def __init__(self, name):
        self.name = name
        self.route_length = None

        self.route_points = list()
        self.start_peir_position = None
        self.start_park_orietation = None
        self.end_peir_position = None

        self.load_ship_route(self.name)

    def load_ship_route(self, name):
        self.route_length = rospy.get_param(f"{name}/route_points_length")
        self.start_park_orietation = rospy.get_param(
            f"{name}/position_1_orietation")
        for i in range(self.route_length):
            route_point = rospy.get_param(f"{name}/position_{i+1}")
            self.route_points.append(route_point)
        self.start_peir_position = self.route_points[0]
        self.end_peir_position = self.route_points[-1]


class DispatchCenter():
    def __init__(self):
        self.route = dict()
        self.ship = dict()
        self.ship_and_route_initialization()
        self.allship_state = dict()

    def ship_and_route_initialization(self):
        self.route["route_1"] = Route("route_1")
        self.route["route_2"] = Route("route_2")
        self.route["route_3"] = Route("route_3")
        self.route["route_auto"] = Route("route_auto")
        global pier_free_to_send_ship_statistics
        pier_free_to_send_ship_statistics["route_1"] = True
        pier_free_to_send_ship_statistics["route_2"] = True
        pier_free_to_send_ship_statistics["route_3"] = True
        pier_free_to_send_ship_statistics["route_auto"] = True

        ships_type = rospy.get_param(f"ships/total_ships_type")
        for ship_type in ships_type:
            self.total_ships_name = rospy.get_param(
                f"ships/{ship_type}/ships_name")
            for ship_name in self.total_ships_name:
                self.ship[ship_name] = AutoShip(ship_name, ship_type)
                # define self auto ship
                if (ship_name == "wamv"):
                    self.ship[ship_name] = AutoShip(ship_name, ship_type)

        self.route_1_ships_name = rospy.get_param(f"route_1/ships")
        self.route_2_ships_name = rospy.get_param(f"route_2/ships")
        self.route_3_ships_name = rospy.get_param(f"route_3/ships")
        self.route_auto_ships_name = rospy.get_param(f"route_auto/ships")

    def allship_state_update(self):
        self.allship_state = dict()
        for ship_name, ship_object in self.ship.items():
            if ship_object.shipping == True:
                self.allship_state[ship_name] = ship_object.latest_state
                # print(
                #     f"{ship_name}, {ship_object.latest_state.modelstate.pose.position.x}, {ship_object.latest_state.modelstate.pose.position.y}")

    def distribute_messages(self):
        for ship_name, ship_state in self.allship_state.items():
            #     print(
            #         f"{ship_name}, {ship_state.modelstate.pose.position.x}, {ship_state.modelstate.pose.position.y}")
            self.ship[ship_name].allship_state = self.allship_state

    def dispatch(self):
        ## dispatch example ##
        # self.assign_route(self.ship["wamv_1_1"], self.route["route_1"])
        # self.ship_parking_in_pier(self.ship["wamv_1_1"])
        # self.ship["wamv_1_1"].run_route()

        while True:
            self.allship_state_update()
            self.distribute_messages()

            global pier_free_to_send_ship_statistics
            ### route_auto dispatch ###
            if pier_free_to_send_ship_statistics["route_auto"] == True:
                for ship_name in self.route_auto_ships_name:
                    if not ship_name in self.ship:
                        continue
                    ship_object = self.ship[ship_name]
                    if ship_object.shipping is False:
                        print(f"[route_auto] {ship_name} is set to sail!")
                        self.assign_route(
                            ship_object, self.route["route_auto"])
                        self.ship_parking_in_pier(ship_object)
                        pier_free_to_send_ship_statistics["route_auto"] = False
                        ship_object.run_route()
                        break
            ### route_1 dispatch ###
            if pier_free_to_send_ship_statistics["route_1"] == True:
                for ship_name in self.route_1_ships_name:
                    if not ship_name in self.ship:
                        continue
                    ship_object = self.ship[ship_name]
                    if ship_object.shipping is False:
                        print(f"[route_1] {ship_name} is set to sail!")
                        self.assign_route(ship_object, self.route["route_1"])
                        self.ship_parking_in_pier(ship_object)
                        pier_free_to_send_ship_statistics["route_1"] = False
                        ship_object.run_route()
                        break

            ### route_2 dispatch ###
            if pier_free_to_send_ship_statistics["route_2"] == True:
                for ship_name in self.route_2_ships_name:
                    if not ship_name in self.ship:
                        continue
                    ship_object = self.ship[ship_name]
                    if ship_object.shipping is False:
                        print(f"[route_2] {ship_name} is set to sail!")
                        self.assign_route(ship_object, self.route["route_2"])
                        self.ship_parking_in_pier(ship_object)
                        pier_free_to_send_ship_statistics["route_2"] = False
                        ship_object.run_route()
                        break

            ### route_3 dispatch ###
            # if pier_free_to_send_ship_statistics["route_3"] is False:
            #     continue
            # else:
            #     for ship_name in self.route_3_ships_name:
            #         if not ship_name in self.ship:
            #             continue
            #         ship_object = self.ship[ship_name]
            #         if ship_object.shipping is False:
            #             print(f"[route_3] {ship_name} is set to sail!")
            #             self.assign_route(ship_object, self.route["route_3"])
            #             self.ship_parking_in_pier(ship_object)
            #             pier_free_to_send_ship_statistics["route_3"] = False
            #             ship_object.run_route()
            #             break

            time.sleep(0.2)

    def assign_route(self, ship, route):
        ship.set_route(route)

    def ship_parking_in_pier(self, ship):
        ship.put_object_to_start_pier()


if __name__ == '__main__':

    rospy.init_node('set_route')
    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.wait_for_service("/gazebo/get_model_state")
    # ros_init()

    TechBrain = DispatchCenter()
    TechBrain.dispatch()
    print("Ending?")
    while True:
        a = 1
