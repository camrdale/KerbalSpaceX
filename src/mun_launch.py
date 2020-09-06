'''
Created on Aug 8, 2020

@author: camrdale
'''

import krpc
import rocket
  
if __name__ == '__main__':
  conn = krpc.connect(name='Return to Kerbin')
  controller = rocket.Controller(conn)
  
  controller.wait_for_launch()

  target_heading = controller.heading(conn.space_center.target_vessel.orbit)
  controller.setup_gravity_turn(1000, turn_start=50.0, target_heading=target_heading)
    
  controller.launch(throttle=0.5, activate_stage=False)
  controller.to_altitude(conn.space_center.target_vessel.orbit.apoapsis_altitude)
  controller.circularization_burn()
           
  controller.inclination_burn()
  controller.rendezvous_burn()
  controller.rendezvous() 
  