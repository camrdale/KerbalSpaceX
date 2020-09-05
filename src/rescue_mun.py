'''
Created on Aug 8, 2020

@author: camrdale
'''

import krpc
import rocket
  
if __name__ == '__main__':
  conn = krpc.connect(name='Rescue from Mun')
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
  controller.transfer_burn(
    target,
    conn.space_center.target_vessel.orbit.apoapsis_altitude +
    conn.space_center.target_vessel.orbit.periapsis_altitude)
  controller.warp_to_encounter()
  controller.circularization_burn(periapsis=True)
      
  controller.inclination_burn()
  controller.rendezvous_burn()
  controller.rendezvous() 
  