'''
Created on Aug 8, 2020

@author: camrdale
'''

import krpc
import rocket
  
if __name__ == '__main__':
  conn = krpc.connect(name='High Orbit Rescue')
  controller = rocket.Controller(conn)
  
  controller.setup_lrb(4)
  controller.setup_second_stage(3)
  controller.setup_gravity_turn(45000)
     
  controller.launch()
  controller.to_altitude(100000)
  controller.to_space()
  controller.circularization_burn()

  target = conn.space_center.target_vessel
  controller.inclination_burn(target.orbit)
  controller.transfer_burn(target, 0)
  # controller.circularization_burn()
  # controller.rendezvous_burn()
  controller.rendezvous()
  