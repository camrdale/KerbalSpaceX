'''
Created on Aug 8, 2020

@author: camrdale
'''

import krpc
import rocket

if __name__ == '__main__':
  conn = krpc.connect(name='Reentry from orbit')
  controller = rocket.Controller(conn)
  
  controller.reentry_burn()
  controller.prepare_for_reentry()
  controller.reentry_parachute()
