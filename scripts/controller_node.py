#!/usr/bin/env python
# coding:utf-8

import rospy
from trx_s_ros_pkg.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from seed_command import SeedCommand
#----------------------------
class SeedController:
  def __init__(self):
    self.serial_port = rospy.get_param("serial_port", "ttyACM0")
    self.can_id = rospy.get_param("can_id",1)
    self.controller_rate = rospy.get_param("controller_rate",20)

    self.seed=SeedCommand(self.serial_port,115200)
    self.seed.COM_Open()

    self.seed.Script_Go(self.can_id,1) #Initialize
    signal = 0
    while(signal != 0xFF):  #wait for initialize
      response = self.seed.CAN_Read()
      signal = int(response[13]+response[14],16)

  def hand_controller(self,req):
    if ((req.command == "grasp") or (req.command == "open")): 
      if (req.command == "grasp"): 
        self.seed.Script_Go(self.can_id,2)
      elif (req.command == "open"): 
        self.seed.Script_Go(self.can_id,3)
      return HandControllerResponse("succeeded")
    else:
      return HandControllerResponse("failed")

  def get_position(self):
    position = self.seed.Get_Pos(self.can_id)[1] #pulse
    if(position != "None"):
      if (position <= 0):
        joint_states.position[0] = 0
        joint_states.position[1] = 0
        joint_states.position[2] = 0
        joint_states.position[3] = 0     
      elif (position <= 52000):
        joint_states.position[0] = position * 1.2 / 52000
        joint_states.position[1] = 0
        joint_states.position[2] = 0
        joint_states.position[3] = position * -2.0 / 52000
      elif (position <= 60000):
        joint_states.position[0] = 1.2
        joint_states.position[1] = (position - 52000) * 0.8 / 8000
        joint_states.position[2] = (position - 52000) * 0.8 / 8000 
        joint_states.position[3] = -2.0  
      else:
        joint_states.position[0] = 1.2
        joint_states.position[1] = 0.8
        joint_states.position[2] = 0.8
        joint_states.position[3] = -2.0  

      joint_states.header.stamp = rospy.Time.now()

      pub.publish(joint_states)
      rospy.Rate(self.controller_rate).sleep()

#----------- main -----------------#
if __name__ == "__main__":
  rospy.init_node('controller_node')
  sc = SeedController()

  pub = rospy.Publisher('joint_states', JointState, queue_size = 100)
  rospy.Service('hand_controller', HandController, sc.hand_controller)
  joint_states = JointState()
  
  joint_states.name = ["indexbase_joint","indexmid_joint","indexend_joint","thumb_joint"]
  joint_states.position = 4*[0]

  position = 0
  
  while not rospy.is_shutdown():
    sc.get_position()

  sc.seed.COM_Close()
