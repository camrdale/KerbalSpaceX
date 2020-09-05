'''
Created on Aug 8, 2020

@author: camrdale
'''

import time
import krpc
import rocket
  
if __name__ == '__main__':
  conn = krpc.connect(name='Launch for Mun')
  controller = rocket.Controller(conn)
  
  controller.setup_lrb(4)
  controller.setup_second_stage(3)
  controller.setup_gravity_turn(40000)
     
  controller.launch()
  controller.to_altitude(80000)
  controller.to_space()
  controller.circularization_burn()
  
  target = conn.space_center.bodies['Mun']
  controller.inclination_burn(target.orbit)
  controller.transfer_burn(target, 200000)
  controller.warp_to_encounter()
  
  time.sleep(10)

  controller.warp_to_encounter()
  controller.reentry_burn()
  controller.prepare_for_reentry()
  controller.reentry_parachute()
  