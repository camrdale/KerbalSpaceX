'''
Created on Aug 8, 2020

@author: camrdale
'''

import krpc
import rocket


if __name__ == '__main__':
  conn = krpc.connect(name='Escape')
  controller = rocket.Controller(conn)
  
  controller.launch()
  controller.to_space()
  controller.drop_engine_at_peak()
  controller.reentry_parachute()
