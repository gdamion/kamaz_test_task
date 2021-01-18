#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path, PoseStamped
from std_msgs.msg import Float64, Bool
from kamaz_test_task.msg import KamazPath
from kamaz_test_task.srv import StartRecord, StopRecord, ClearTrajectory
from kamaz_test_task.srv import StartRecordResponse, StopRecordResponse, ClearTrajectoryResponse


class NodeClass:
    def __init__(self):
        rospy.init_node('kamaz_test_task_node')

        self.input_odom = rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odomCB)
        self.output_path = rospy.Publisher('/carla/ego_vehicle/path', KamazPath, queue_size=10)
        self.start_record = rospy.Service('start_record', StartRecord, self.startRecord)
        self.stop_record = rospy.Service('stop_record', StopRecord, self.stopRecord)
        self.clear_trajectory = rospy.Service('clear_trajectory', ClearTrajectory, self.clearTrajectory)

        self.err_req = "You've sent false request. You should send true value in the request msg!"
        self.record_started = False
        self.path = 0.0

        rospy.loginfo('Node kamaz_test_task_node started')

    def odomCB(self, data):
        # self.path += 0.3
        k_path = KamazPath()

        pose = PoseStamped()
        pose.header.frame_id = "main"
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.position.z = float(data.pose.pose.position.z)
        pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose.pose.orientation.w = float(data.pose.pose.orientation.w)
        
        # k_path.path =
        # k_path.length =
        self.output_path.publish()

    def startRecord(self, req):
        res = StartRecordResponse()
        if req.start == True:
            s = ""
            if not self.record_started:
                s = "Path recording started"
                self.record_started = True
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
            if self.path > 0.0:
                s = "Path recording cleared"
                self.path = 0.0
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
