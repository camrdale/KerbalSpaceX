'''
Created on Aug 8, 2020

@author: camrdale
'''

import krpc
import rocket
  
if __name__ == '__main__':
  conn = krpc.connect(name='Return to Kerbin')
  controller = rocket.Controller(conn)
  
  controller.setup_gravity_turn(1000, turn_start=50.0)
    
  controller.launch(throttle=0.5, activate_stage=False)
  controller.to_altitude(10500)
  controller.circularization_burn()

  controller.return_burn()
  controller.warp_to_encounter()
  controller.prepare_for_reentry()
  controller.reentry_parachute()
  