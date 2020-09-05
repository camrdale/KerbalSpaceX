'''
Created on Aug 8, 2020

@author: camrdale
'''

import krpc
import rocket
  
if __name__ == '__main__':
  conn = krpc.connect(name='Return to Kerbin')
  controller = rocket.Controller(conn)
  
  controller.return_burn()
  controller.warp_to_encounter()
  controller.prepare_for_reentry()
  controller.reentry_parachute()
  