'''
Created on Aug 8, 2020

@author: camrdale
'''

import krpc
import rocket

if __name__ == '__main__':
  conn = krpc.connect(name='Launch into orbit')
  controller = rocket.Controller(conn)
  
  controller.setup_lrb(2)
  controller.setup_second_stage(1)
  controller.setup_gravity_turn(40000)

  controller.launch()
  controller.to_altitude(100000)
  controller.to_space()
  controller.activate_second_stage()
  controller.circularization_burn()
