#!/usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#RCJ2016 Navigation Test用パラメータのROSノード
#
#author: Yutaro ISHIDA
#date: 16/03/17
#--------------------------------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

from common_import import *
from common_function import *
from common_param import *


#--------------------------------------------------
#メイン関数
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])


    rospy.set_param('/param/dbg/sm/all', 0) #bool    

    #選択されたデバッグモードを使う
    if rospy.get_param('/param/dbg/sm/all') == 0:
        #上の方が優先度高い
        rospy.set_param('/param/dbg/sm/flow', 0) #bool 全ての機能なしでsmを流す
        rospy.set_param('/param/dbg/sm/stepin', 0) #bool ステートごとにキー入力を促す
        rospy.set_param('/param/dbg/sm/stepout', 0) #bool ステートごとにキー入力を促す
        rospy.set_param('/param/dbg/speech/onlyspeech', 0) #bool 音声認識のみのデバッグ
        rospy.set_param('/param/dbg/speech/ssynlog', 0) #bool 音声合成の文章をデバッグ表示
    #全デバッグモードを使う、else以下は触らない
    else:
        rospy.set_param('/param/dbg/sm/flow', 1) #bool
        rospy.set_param('/param/dbg/sm/stepin', 1) #bool ステートごとにキー入力を促す
        rospy.set_param('/param/dbg/sm/stepout', 1) #bool ステートごとにキー入力を促す
        rospy.set_param('/param/dbg/speech/onlyspeech', 1) #bool 音声認識のみのデバッグ
        rospy.set_param('/param/dbg/speech/ssynlog', 1) #bool 音声合成の文章をデバッグ表示


    rospy.set_param('/param/waypoint/pos', [{'x':6.57, 'y':0.36, 'yaw':0.0},
                                            {'x':2.0, 'y':5.79, 'yaw':2.47},
                                            {'x':10.02, 'y':3.86, 'yaw':-1.57}]) #float


    rospy.set_param('/param/waypoint2/cnt', 0)


    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        main_rate.sleep()
