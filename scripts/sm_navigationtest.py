#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#RCJ2016 Navigation Test用ステートマシンのROSノード
#
#author: Yutaro ISHIDA
#date: 16/03/12
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

from common_import import *
from common_function import *


rospy.sleep(5) #paramノードが立ち上がるまで待つ


#--------------------------------------------------
#ステートマシン設計規則
#--------------------------------------------------
#ステートを跨ぐデータはパラメータ(/param/以下)に保存する


#--------------------------------------------------
#--------------------------------------------------
class init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
class CamModeChange1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        if rospy.get_param('/param/dbg/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:        
            commonf_speech_single('カメラモード切り替え中。')

            call(['rosnode', 'kill', '/camera/camera_nodelet_manager'])    
            call(['rosnode', 'kill', '/camera/depth_metric'])
            call(['rosnode', 'kill', '/camera/depth_metric_rect'])
            call(['rosnode', 'kill', '/camera/depth_points'])
            call(['rosnode', 'kill', '/camera/depth_rectify_depth'])
            call(['rosnode', 'kill', '/camera/depth_registered_rectify_depth'])
            call(['rosnode', 'kill', '/camera/points_xyzrgb_hw_registered'])
            call(['rosnode', 'kill', '/camera/rectify_color'])
            rospy.sleep(5)

            os.system('yes | rosnode cleanup')
            os.system('echo horihori|sudo -S service udev reload')
        
            Popen(['rosrun', 'openni_tracker', 'openni_tracker'])
            rospy.sleep(2)

        commonf_dbg_sm_stepout()
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
class WaitStartSig(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        raw_input('#####Type enter key to start#####')

        commonf_dbg_sm_stepout()
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
class DetectDoorOpen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        call(['rosrun', 'common_pkg','detect_dooropen.py'])

        commonf_dbg_sm_stepout()
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
class SLAM_GoToWaypoint1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        commonf_speech_single('ウェイポイント１に移動します。')

        waypoint_pos = rospy.get_param('/param/waypoint/pos')

        commonf_actionf_move_base(waypoint_pos[0]['x'], waypoint_pos[0]['y'], waypoint_pos[0]['yaw'])

        commonf_speech_single('ウェイポイント１に移動しました。')

        commonf_dbg_sm_stepout()
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
class SLAM_GoToWaypoint2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1', 'exit2'])
        self._move_base_action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._move_base_action_client.wait_for_server()


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        commonf_speech_single('ウェイポイント２に移動します。')

        proc_detect_parson = Popen(['rosrun', 'common_pkg', 'detect_person.py'])

        waypoint_pos = rospy.get_param('/param/waypoint/pos')

        rospy.loginfo('Goal pos x: ' + str(waypoint_pos[1]['x']) + ' y: ' + str(waypoint_pos[1]['y']) + ' yaw: ' + str(waypoint_pos[1]['yaw']))

        quaternion = quaternion_from_euler(0.0, 0.0, waypoint_pos[1]['yaw'])        

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(waypoint_pos[1]['x'], waypoint_pos[1]['y'], 0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        self._move_base_action_client.send_goal(goal)

        #self._move_base_action_client.wait_for_result()

        rospy.set_param('/param/slam/goal/pos/x', float(waypoint_pos[1]['x'])) #float
        rospy.set_param('/param/slam/goal/pos/y', float(waypoint_pos[1]['y'])) #float
        rospy.set_param('/param/slam/goal/pos/yaw', float(waypoint_pos[1]['yaw'])) #float
                
        proc_slam_goto = Popen(['rosrun', 'common_pkg', 'slam_goto.py'])


        while not rospy.is_shutdown():
            proc_detect_parson.poll()
            if proc_detect_parson.returncode == 0 and rospy.get_param('/param/waypoint2/cnt') == 0:
                #proc_detect_parson.kill()
                commonf_node_killer('detect_person')
                #proc_slam_goto.kill()
                commonf_node_killer('slam_goto')                
                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)
                self._move_base_action_client.cancel_goal()
                rospy.set_param('/param/waypoint2/cnt', rospy.get_param('/param/waypoint2/cnt') + 1)
                rospy.sleep(2)            

                commonf_dbg_sm_stepout()
                return 'exit1'

            proc_slam_goto.poll()                
            if proc_slam_goto.returncode == 0:
                #proc_detect_parson.kill()
                commonf_node_killer('detect_person')
                #proc_slam_goto.kill()
                commonf_node_killer('slam_goto')      
                commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)
                self._move_base_action_client.cancel_goal()
                rospy.set_param('/param/waypoint2/cnt', rospy.get_param('/param/waypoint2/cnt') + 1)       

                commonf_speech_single('ウェイポイント２に移動しました。')         

                commonf_dbg_sm_stepout()
                return 'exit2'            


#--------------------------------------------------
#--------------------------------------------------
class SSyn_AskToMoveOut(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        commonf_speech_single('邪魔になっているので避けてください。')

        commonf_dbg_sm_stepout()
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
class CamModeChange2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        if rospy.get_param('/param/dbg/sm/flow') == 0 and rospy.get_param('/param/dbg/speech/onlyspeech') == 0:        
            commonf_speech_single('カメラモード切り替え中。')        

            commonf_node_killer('openni_tracker')
            rospy.sleep(5)
        
            os.system('yes | rosnode cleanup')
            os.system('echo horihori|sudo -S service udev reload')

            Popen(['roslaunch', 'openni2_launch', 'openni2.launch'])
            rospy.sleep(2)

        commonf_dbg_sm_stepout()
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
class SLAM_GoToWaypoint3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        commonf_speech_single('ウェイポイント３に移動します。')

        waypoint_pos = rospy.get_param('/param/waypoint/pos')

        commonf_actionf_move_base(waypoint_pos[2]['x'], waypoint_pos[2]['y'], waypoint_pos[2]['yaw'])

        commonf_speech_single('ウェイポイント３に移動しました。')

        commonf_dbg_sm_stepout()
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
class SRec_WaitOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1', 'err_in_speech_rec'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()
        
        if commonf_actionf_speech_rec(self.__class__.__name__) == True: #音声認識ノードに現在のステートに対する処理が記述されていた時
            commonf_dbg_sm_stepout()            
            return 'exit1'
        else: #音声認識ノードに現在のステートに対する処理が記述されていない時
            commonf_dbg_sm_stepout()            
            return 'err_in_speech_rec'


#--------------------------------------------------
#--------------------------------------------------
class SRec_InstructOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1', 'err_in_speech_rec'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()
        
        if commonf_actionf_speech_rec(self.__class__.__name__) == True: #音声認識ノードに現在のステートに対する処理が記述されていた時
            commonf_dbg_sm_stepout()            
            return 'exit1'
        else: #音声認識ノードに現在のステートに対する処理が記述されていない時
            commonf_dbg_sm_stepout()            
            return 'err_in_speech_rec'


#--------------------------------------------------
#--------------------------------------------------
class Img_MemoOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        commonf_speech_single('あなたを記憶します。１メートル前に立ってください。')
        commonf_speech_single('あなたを記憶しました。')

        commonf_dbg_sm_stepout()
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
class FollowOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1', 'err_in_speech_rec'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        Popen(['rosrun', 'common_pkg', 'img_followparson'])
        rospy.sleep(10)
                
        if commonf_actionf_speech_rec(self.__class__.__name__) == True: #音声認識ノードに現在のステートに対する処理が記述されていた時
            commonf_node_killer('img_followparson')
            commonf_pubf_cam_pan(0)
            commonf_pubf_cam_tilt(0)
            commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)

            commonf_speech_single('ウェイポイント４に到着しました。')

            commonf_dbg_sm_stepout()
            return 'exit1'
        else: #音声認識ノードに現在のステートに対する処理が記述されていない時
            commonf_dbg_sm_stepout()            
            return 'err_in_speech_rec'


#--------------------------------------------------
#--------------------------------------------------
class SLAM_GoBackToWaypoint3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        commonf_speech_single('ウェイポイント３に移動します。')

        waypoint_pos = rospy.get_param('/param/waypoint/pos')

        commonf_actionf_move_base(waypoint_pos[2]['x'], waypoint_pos[2]['y'], waypoint_pos[2]['yaw'])

        commonf_speech_single('ウェイポイント３に移動しました。')

        commonf_dbg_sm_stepout()
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
class SLAM_LeaveArena(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit1'])


    def execute(self, userdata):
        commonf_dbg_sm_stepin()

        commonf_speech_single('退場します。')

        commonf_actionf_move_base(8.57, -3.64, -1.57)

        commonf_speech_single('退場しました。')

        commonf_dbg_sm_stepout()
        return 'exit1'


#--------------------------------------------------
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])


    sm = smach.StateMachine(outcomes=['exit'])


    #起動音を再生する
    commonf_actionf_sound_effect_single('launch')

   
    commonf_pubf_scan_mode('lrf')

    commonf_pubf_cam_pan(0.523)
    commonf_pubf_cam_tilt(0.523)
    commonf_pubf_mic_pan(-0.523)
    commonf_pubf_mic_tilt(-0.523)
    rospy.sleep(0.5)
    commonf_pubf_cam_pan(0)
    commonf_pubf_cam_tilt(0)
    commonf_pubf_mic_pan(0)
    commonf_pubf_mic_tilt(-0.349)
    rospy.sleep(0.5)

    commonf_pubf_cmd_vel(0, 0, 0, 0, 0, 0)

    commonf_actionf_cam_lift(0.555)


    #commonf_speech_single('タスク、ナビゲーションテストをスタート。')
    #commonf_speech_single('スタートステートを指定して下さい。')
    rospy.loginfo('タスク、ナビゲーションテストをスタート')
    rospy.loginfo('スタートステートを指定して下さい')

    print '#####If you want to start from first state, please type enter key#####'
    start_state = raw_input('#####Please Input First State Name##### >> ')
    if not start_state:
        start_state = 'CamModeChange1'

    #commonf_speech_single('ステートマシンをスタート。')
    rospy.loginfo('ステートマシンをスタート')


    with sm:
        smach.StateMachine.add('init', init(), 
                               transitions={'exit1':start_state})
        smach.StateMachine.add('CamModeChange1', CamModeChange1(), 
                               transitions={'exit1':'WaitStartSig'})
        smach.StateMachine.add('WaitStartSig', WaitStartSig(), 
                               transitions={'exit1':'DetectDoorOpen'})
        smach.StateMachine.add('DetectDoorOpen', DetectDoorOpen(), 
                               transitions={'exit1':'SLAM_GoToWaypoint1'})
        smach.StateMachine.add('SLAM_GoToWaypoint1', SLAM_GoToWaypoint1(), 
                               transitions={'exit1':'SLAM_GoToWaypoint2'})
        smach.StateMachine.add('SLAM_GoToWaypoint2', SLAM_GoToWaypoint2(), 
                               transitions={'exit1':'SSyn_AskToMoveOut',
                                            'exit2':'CamModeChange2'})
        smach.StateMachine.add('SSyn_AskToMoveOut', SSyn_AskToMoveOut(), 
                               transitions={'exit1':'SLAM_GoToWaypoint2'})
        smach.StateMachine.add('CamModeChange2', CamModeChange2(), 
                               transitions={'exit1':'SLAM_GoToWaypoint3'})
        smach.StateMachine.add('SLAM_GoToWaypoint3', SLAM_GoToWaypoint3(), 
                               transitions={'exit1':'SRec_WaitOperator'})
        smach.StateMachine.add('SRec_WaitOperator', SRec_WaitOperator(), 
                               transitions={'exit1':'SRec_InstructOperator',
                                            'err_in_speech_rec':'exit'})
        smach.StateMachine.add('SRec_InstructOperator', SRec_InstructOperator(), 
                               transitions={'exit1':'Img_MemoOperator',
                                            'err_in_speech_rec':'exit'})
        smach.StateMachine.add('Img_MemoOperator', Img_MemoOperator(), 
                               transitions={'exit1':'FollowOperator'})
        smach.StateMachine.add('FollowOperator', FollowOperator(), 
                               transitions={'exit1':'SLAM_GoBackToWaypoint3',
                                            'err_in_speech_rec':'exit'})
        smach.StateMachine.add('SLAM_GoBackToWaypoint3', SLAM_GoBackToWaypoint3(), 
                               transitions={'exit1':'SLAM_LeaveArena'})
        smach.StateMachine.add('SLAM_LeaveArena', SLAM_LeaveArena(), 
                               transitions={'exit1':'exit'})


    sis = smach_ros.IntrospectionServer('sm', sm, '/SM_ROOT')
    sis.start()


    outcome = sm.execute()


    commonf_speech_single('タスクを終了します。')
    raw_input('#####Type Ctrl + c key to end#####')


    while not rospy.is_shutdown():
        rospy.spin()
