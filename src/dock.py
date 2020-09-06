'''
Created on Aug 8, 2020

@author: camrdale
'''

import krpc
import rocket

if __name__ == '__main__':
  conn = krpc.connect(name='Dock to target')
  controller = rocket.Controller(conn)
  
  controller.cancel_relative_velocity()
  controller.dock()
  