'''
Created on Aug 8, 2020

@author: camrdale
'''

import time
import krpc
import rocket

if __name__ == '__main__':
  conn = krpc.connect(name='Tourist to orbit and back')
  controller = rocket.Controller(conn)
  
  controller.setup_srb(4)
  controller.setup_second_stage(3)
  controller.setup_gravity_turn(40000)
 
  controller.launch()
  controller.to_altitude(80000)
  controller.to_space()
  controller.circularization_burn()
   
  time.sleep(10)

  controller.reentry_burn()
  controller.prepare_for_reentry()
  controller.reentry_parachute()
