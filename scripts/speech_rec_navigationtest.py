#! /usr/bin/env python
# -*- coding: utf-8 -*-


#--------------------------------------------------
#RCJ2016 Navigation Test用音声認識ActionのROSノード
#
#author: Yutaro ISHIDA
#date: 16/03/17
#--------------------------------------------------


#-----------speech recognition------------------------
from __future__ import print_function
import socket
from contextlib import closing
import commands

import re
import csv
import numpy as np
#-----------speech recognition------------------------


import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir('common_pkg') + '/scripts/common')

from common_import import *
from common_function import *


rospy.sleep(5) #paramノードが立ち上がるまで待つ


#--------------------------------------------------
#--------------------------------------------------
class SpeechRec(object):
    #--------------------------------------------------
    #--------------------------------------------------   
    def __init__(self, julius_bufsize, julius_sock, RecgDicts):
        self._speech_rec_action_server = actionlib.SimpleActionServer('speech_rec_action', SpeechRecAction, execute_cb = self.speech_rec)
        self._speech_rec_action_server.start()
        self.julius_bufsize = julius_bufsize
        self.julius_sock = julius_sock
        self.RecgDicts = RecgDicts


    #--------------------------------------------------
    #-------------------------------------------------- 
    def speech_rec(self, goal):
        #--------------------------------------------------
        #オペレータの呼びかけを認識する
        #-------------------------------------------------- 
        if goal.speech_rec_goal == 'SRec_WaitOperator':
            if rospy.get_param('/param/dbg/sm/flow') == 0:
                commonf_speech_single('オペレータは、私の右側の指示書を取ってください。')
                commonf_speech_single('オペレータは私に、エクシアちょっときて。と話しかけて下さい。')
                while 1:
                    text = self.voice2text()
                    flag = self.returnFlag('CMD', text)
                    if text == '':
                        pass
                    elif flag == 'start':
                        break
                    else:
                        commonf_speech_single('すみません。聞き取れませんでした。')

            result = SpeechRecResult(speech_rec_result = True)
            self._speech_rec_action_server.set_succeeded(result)


        #--------------------------------------------------
        #オペレータに追跡停止方法を教える
        #-------------------------------------------------- 
        elif goal.speech_rec_goal == 'SRec_InstructOperator':
            if rospy.get_param('/param/dbg/sm/flow') == 0:
                commonf_speech_single('今から追跡を停止する方法を教えます。追跡を停止する時は私に向かって、ここで止まって。と言ってください。この方法を理解した場合は、分かったよ。と言ってください。もう一度聞きたい場合は、もう一度お願い。と言ってください。')
                while 1:
                    text = self.voice2text()                
                    flag = self.returnFlag('PRIDE', text)
                    if text == '':
                        pass
                    elif flag == 'again':
                        commonf_speech_single('話をちゃんと聞いてください。追跡を停止する時は私に向かって、ここで止まって。と言ってください。この方法を理解した場合は、分かったよ。と言ってください。もう一度聞きたい場合は、もう一度お願い。と言ってください。')
                    elif flag == 'ok':
                        break
                    else:
                        commonf_speech_single('すみません。聞き取れませんでした。')

            result = SpeechRecResult(speech_rec_result = True)
            self._speech_rec_action_server.set_succeeded(result)


        #--------------------------------------------------
        #追跡停止の呼びかけを認識する
        #-------------------------------------------------- 
        elif goal.speech_rec_goal == 'FollowOperator':
            if rospy.get_param('/param/dbg/sm/flow') == 0:
                while 1:                
                    text = self.voice2text()
                    flag = self.returnFlag('MOVING', text)
                    if text == '':
                        pass
                    elif flag == 'stop':
                        commonf_speech_single('止まります。')
                        break
                    else:
                        commonf_speech_single('すみません。聞き取れませんでした。')

            result = SpeechRecResult(speech_rec_result = True)
            self._speech_rec_action_server.set_succeeded(result)


        #--------------------------------------------------
        #-------------------------------------------------- 
        else:
            rospy.logwarn('[speech_rec]: ステートに対する処理が記述されていません。')
            result = SpeechRecResult(speech_rec_result = False)
            self._speech_rec_action_server.set_succeeded(result)
    
    
    #--------------------------------------------------
    #Juliusから返ってきた出力（sock,bufsize,XML形式）から、文章部分の抽出を行う関数
    #-------------------------------------------------- 
    def voice2text(self):
        #sentence = ''
        #追加socket削除するため以下2行を追加
        socket = self.julius_sock
        commonf_actionf_sound_effect_multi('speech_rec')
        rospy.sleep(0.5)
        os.system('amixer -c 2 sset "Mic" 100%')
        #rospy.sleep(0.5)
        recv_data = socket.recv(self.julius_bufsize)
        recv_data = ''
        #print('------ speech rec start ------------')
        while True:
            #socket = self.julius_sock
            recv_data += socket.recv(self.julius_bufsize)
            #sentence_start = re.findall(r'<s>', recv_data)
            sentence_end = re.findall(r'</s>', recv_data)
            if sentence_end:
                sentence = ''
                matchs = re.findall(r'<WHYPO WORD=".*?"', recv_data)
                for match in matchs:
                    s = match[13:-1]
                    sentence += s
                out_sentence = sentence.strip()
                break
        os.system('amixer -c 2 sset "Mic" 0%')
        print(out_sentence)
        return out_sentence


    #--------------------------------------------------
    #認識した文章に対応するフラグを返す
    #--------------------------------------------------     
    def returnFlag(self, state, text):
        if text == '':
            return 'error'
        RecgDict = self.RecgDicts[state]
        for k, v in RecgDict.items():
            if text.find(k) >= 0:
                return v
        else:
            return 0
    

#--------------------------------------------------
#メイン関数
#--------------------------------------------------
if __name__ == '__main__':
    node_name = os.path.basename(__file__)
    node_name = node_name.split('.')
    rospy.init_node(node_name[0])


    #初期設定
    #-----------speech recognition------------------------
    julius_host = 'localhost'
    julius_port = 10500
    julius_bufsize = 4096 * 4
    julius_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    julius_sock.connect((julius_host,julius_port))
    RecgDicts = {
        'PRIDE': {'わかったよ': 'ok', 'もう一度お願い': 'again'},  # 教示
        'MOVING': {'ここで止まって': 'stop'},  # 移動中
        'CMD': {'エクシアちょっときて': 'start'}
    }
    #-----------speech recognition------------------------
    

    speech_rec = SpeechRec(julius_bufsize, julius_sock, RecgDicts)


    main_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        main_rate.sleep()
