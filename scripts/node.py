#!/usr/bin/env python
import math as m
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool
from kamaz_test_task.msg import KamazPath
from kamaz_test_task.srv import StartRecord, StopRecord, ClearTrajectory
from kamaz_test_task.srv import StartRecordResponse, StopRecordResponse, ClearTrajectoryResponse


class NodeClass:
    def __init__(self):
        rospy.init_node('kamaz_test_task_node')

        self.path = Path()
        self.path_len = float(0)

        self.input = rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odomCB)
        self.output = rospy.Publisher('/path', KamazPath, queue_size=10)
        self.start_record = rospy.Service('start_record', StartRecord, self.startRecord)
        self.stop_record = rospy.Service('stop_record', StopRecord, self.stopRecord)
        self.clear_trajectory = rospy.Service('clear_trajectory', ClearTrajectory, self.clearTrajectory)

        self.err_req = "You've sent false request. You should send true value in the request msg!"
        self.record_started = False
        self.first_iter = True
        self.x, self.y, self.z = float(0), float(0), float(0)

        self.max_buf = rospy.get_param("~max_buf", 1000)

        rospy.loginfo('Node kamaz_test_task_node started')

    def odomCB(self, new_data):
        if self.record_started:
            pose = PoseStamped()
            pose.header = new_data.header
            pose.pose = new_data.pose.pose
            x_new, y_new, z_new = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z

            if self.first_iter:
                self.first_iter = False
                delta_path = float(0)
            else:
                delta_path = m.sqrt((x_new - self.x)**2 + (y_new - self.y)**2 + (z_new - self.z)**2)
                if self.x == x_new and self.y == y_new and self.z == z_new:
                    return
            self.path_len += delta_path
            self.x, self.y, self.z = x_new, y_new, z_new

            self.path.header = new_data.header
            if (len(self.path.poses) > self.max_buf):
                self.path.poses.pop(0)
            self.path.poses.append(pose)

        k_path = KamazPath()
        k_path.header = new_data.header
        k_path.path = self.path
        k_path.path_len = Float64(self.path_len)

        self.output.publish(k_path)

    def startRecord(self, req):
        res = StartRecordResponse()
        if req.start == True:
            s = ""
            if not self.record_started:
                s = "Path recording started"
                self.record_started = True
                self.first_iter = True
            else:
                s = "Path recording is already started"
            res.res = s
            rospy.loginfo(s)
        else:
            rospy.logwarn(self.err_req)
            res.res = self.err_req
        return res

    def stopRecord(self, req):
        res = StopRecordResponse()
        if req.stop == True:
            s = ""
            if self.record_started:
                s = "Path recording stopped"
                self.record_started = False
            else:
                s = "Path recording is already stopped"
            res.res = s
            rospy.loginfo(s)
        else:
            rospy.logwarn(self.err_req)
            res.res = self.err_req
        return res

    def clearTrajectory(self, req):
        res = ClearTrajectoryResponse()
        if req.clear == True:
            s = ""
            if self.path_len > 0 and self.path.poses:
                s = "Path recording cleared"
                self.path_len = float(0)
            else:
                s = "Path recording is already equal to zero"
            res.res = s
            rospy.loginfo(s)
        else:
            rospy.logwarn(self.err_req)
            res.res = self.err_req
        return res


if __name__ == "__main__":
    try:
        NC = NodeClass()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        print('Something wrong with kamaz rosnode!')
    rospy.loginfo('kamaz_test_task_node stop')
