#! /usr/bin/env python
# encoding: utf-8
import numpy as np
import cv2 as cv
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi


class navigation_demo:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(3))
        print("server start!")

    def set_pose(self, p):
        if self.move_base is None:
            return False

        [x, y, th] = p

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        self.set_pose_pub.publish(pose)
        return True

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        # rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)
        pass

    def goto(self, p,temp):
        rospy.loginfo("[Navi] goto %s"%p)
        
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, 0/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        time_temp = temp
        if time_temp:
             result = self.move_base.wait_for_result(rospy.Duration(3))
             self.move_base.cancel_goal()
        else:
            result = self.move_base.wait_for_result(rospy.Duration(3))

        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

if __name__ == "__main__":
    rospy.init_node('navigation_demo',anonymous=True)

    use_debug = 0  # 是否开启可视化
    use_dilate = 0  # 是否开启膨胀
    diameter = 2  # 每像素代表0.05米，预设小车直径为0.3m，即6像素
    start_x = 30  # 默认起点
    start_y = 20
    path = [[start_x, start_y]]  # 路径存储
    current_point = [start_x, start_y]  # 当前点
    obstacle_distance = [0, 0, 0, 0]  # 四个方向的距离存储，记得用完清零
    direct = 0  # 方向存储
    shortest_dis = 0  # 最短距离
    shortest_index = 0  # 最短距离的索引
    radius = 2  # 半径计数

    # 地图预处理
    img = cv.imread("/home/michealjs/catkin_ws/src/ccpp/src/map_rot.pgm")
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    img_copy = gray.copy()
    # 截取
    ret, thresh = cv.threshold(gray, 127, 255, cv.THRESH_BINARY)
    if use_dilate:
        canny = cv.Canny(thresh, 50, 150)
        kernel = np.ones((15, 15), np.uint8)
        dilation = cv.dilate(canny, kernel, iterations=1)
        cv.imshow("canny", canny)
        cv.imshow("dilation", dilation)
        thresh = dilation
    _, contours, hierarchy = cv.findContours(thresh, 1, 2)
    cnt = contours[-2]  # 倒数第二个框符合要求
    x, y, w, h = cv.boundingRect(cnt)  # 左上角坐标，宽高
    resize_w = w // diameter
    resize_h = h // diameter
    img = cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
    # print("有效地图范围：x:{},y:{}".format(x + w, y + h))
    cut = img_copy[y:y + h, x:x + w]  # 高度范围、宽度范围截取,仅保留有效地图区域
    recovery_path = [[x + start_x * diameter,
                    y + start_y * diameter]]  # 转换到ROS中的路径坐标
    if use_debug:
        cv.imshow("cut", cut)
        cv.waitKey(0)

    # 栅格化
    res = cv.resize(cut, (w // diameter, h // diameter),
                    cv.INTER_AREA)  # 缩小使用插值法INTER_AREA
    ret1, thresh1 = cv.threshold(res, 250, 255, cv.THRESH_BINARY)
    map_BGR = cv.cvtColor(thresh1, cv.COLOR_GRAY2BGR)  # 可视化辅助
    map_bin = thresh1.copy()  # 用于走过的路径标记
    map_BGR1 = map_BGR.copy()
    # cv.imshow("thresh1", thresh1)
    # cv.imwrite("thresh1.png", thresh1)
    # cv.waitKey(0)

    # 从默认起点开始找路径
    factor = [[1, 0],  # 右
            [0, 1],  # 下
            [-1, 0],  # 左
            [0, -1]]  # 上
    try:
        while True:
            # 方向判定
            for direct in range(4):  # 从四个方向找，顺序为右下左上
                for dis in range(1, w):  # 一般w比h大，如有不同这里可以改。为了效率这里默认w
                    if map_bin[current_point[0] + dis * factor[direct][0], current_point[1] + dis * factor[direct][1]] == 0:
                        if dis == 1:  # 下一个就是墙（撞墙了），
                            obstacle_distance[direct] = w
                        else:
                            obstacle_distance[direct] = dis
                        break  # 找到黑点即跳出，停止查找
                    elif dis == w:  # 如果找到最远了，则跳出，并且标记距离为最长
                        obstacle_distance[direct] = w
                    else:  # 如果不是黑点（障碍），则继续搜索
                        continue

            shortest_dis = min(obstacle_distance)  # 判断距离最短方向
            shortest_index = obstacle_distance.index(shortest_dis)  # 最小值的索引

            # 开始移动
            map_bin[current_point[0], current_point[1]] = 0  # 上一个路径标黑（用于下次路径查找）
            map_BGR[current_point[0], current_point[1]] = (
                0, 0, 255)  # 上一个路径标红（用于可视化）
            current_point = [current_point[0] + factor[shortest_index]
                            [0], current_point[1] + factor[shortest_index][1]]
            map_BGR[current_point[0], current_point[1]] = (
                0, 255, 0)  # 当前点标绿（用于可视化）
            path.append(current_point)  # 存储路径

            # 放大助于可视化
            if use_debug:
                # cv.imwrite("map_BGR.png", map_BGR)
                map_BGR_expand = cv.resize(
                    map_BGR, (w * 3, h * 3), cv.INTER_LINEAR)
                cv.imshow("map_BGR", map_BGR_expand)
                cv.waitKey(3)

            # 进入死角
            if shortest_dis == w:
                # 结束判定
                minVal, maxVal, minLoc, maxLoc = cv.minMaxLoc(map_bin)  # 寻找最亮点
                # print(maxLoc)
                if maxVal == 0:  # 全图黑，真的扫描完了
                    # cv.imwrite("map_BGR.png", map_BGR)
                    print("Over")
                    # cv.waitKey(0)
                    break
                else:  # 还有亮点，没扫完
                    # 7.9 暴力搜索  复杂度为n^2
                    # print("开始暴力搜索")
                    nearest_distance = {
                        "pose_x": 0,
                        "pose_y": 0,
                        "distance": resize_w ** 2 + resize_h ** 2  # 默认最大
                    }
                    for i in range(resize_h):  # 矩阵XY坐标反于图像坐标
                        for j in range(resize_w):
                            if map_bin[i, j] == 255:  # 只有白点才可以进入距离计算
                                distance_temp = (
                                    i - current_point[0]) ** 2 + (j - current_point[1]) ** 2
                                # √x^2+y^2平方和(求距离本应该开根号，但只是为了比较大小)
                                # print(i, " ", j, " ", "计算距离", distance_temp, "最短距离：", nearest_distance["distance"])
                                if distance_temp < nearest_distance["distance"]:
                                    nearest_distance["pose_x"] = i
                                    nearest_distance["pose_y"] = j
                                    nearest_distance["distance"] = distance_temp
                    # 直接挪动到最近的白点,但是没有考虑隔墙的问题
                    current_point = [nearest_distance["pose_x"],
                                    nearest_distance["pose_y"]]
                    # print("搜索完毕")
    except:
        pass

    # 坐标转换s
    print("len of path:", len(path))
    for cnt in range(len(path)):
        register = [9.6-(x + path[cnt][0] * diameter)*0.05,9.6-( y + path[cnt][1] * diameter)*0.05]
        # register = [9.6-( y + path[cnt][1] * diameter)*0.05,9.6-(x + path[cnt][0] * diameter)*0.05]
        recovery_path.append(register)

    # goals = [[1.0,2.0,0],[2.0,1.0,0.0],[1.0,2.0,0],[0.0,0.0,0.0]] #注意X正方向是垂直上方向
    navi = navigation_demo()

   
    # navi.set_pose([-2.0,-1.5,0.0])
    # for goal in goals:
    #     navi.goto(goal)
    k = 0
    for tempg in range(len(recovery_path)):
        if tempg == 0:
            time_temp = 1
        else:
            time_temp = 0
        print("recovery_path[tempg]:",recovery_path[tempg])
        print("k:",k)
        map_BGR1[path[k][0]][path[k][1]]=(0,0,255)
        k = k + 1
        map_BGR1_expand = cv.resize(map_BGR1, (w * 3, h * 3), cv.INTER_LINEAR)
        cv.imshow("map_BGR1", map_BGR1_expand)
        cv.waitKey(3)
        navi.goto(recovery_path[tempg],time_temp)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        r.sleep()



