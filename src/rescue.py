'''
Created on Aug 8, 2020

@author: camrdale
'''

import krpc
import rocket

if __name__ == '__main__':
  conn = krpc.connect(name='Rescue rendezvous')
  controller = rocket.Controller(conn)
  
  controller.wait_for_launch()
            
  controller.setup_srb(4)
  controller.setup_second_stage(3)
  controller.setup_gravity_turn(40000)
             
  controller.launch()
  controller.to_altitude(
    conn.space_center.target_vessel.orbit.apoapsis_altitude)
  controller.to_space()
  controller.circularization_burn()
           
  controller.inclination_burn()
  controller.rendezvous_burn()
  controller.rendezvous() 
  