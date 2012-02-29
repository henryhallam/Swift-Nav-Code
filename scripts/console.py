#!/usr/bin/env python

import serial_link
import struct
import time

import argparse
parser = argparse.ArgumentParser(description='Swift Nav Console.')
parser.add_argument('-p', '--port',
                   default=[serial_link.DEFAULT_PORT], nargs=1,
                   help='specify the serial port to use.')
args = parser.parse_args()
serial_port = args.port[0]

import logging
logging.basicConfig()

# Fix default font issue on Linux
import os
from enthought.kiva.fonttools.font_manager import fontManager, FontProperties
if os.name == "posix":
  font = FontProperties()
  font.set_name("Arial")
  fontManager.defaultFont = fontManager.findfont(font)

from enthought.traits.api import Str, Instance, Dict, HasTraits, Int
from enthought.traits.ui.api import Item, ShellEditor, View, VSplit, HSplit, Tabbed, InstanceEditor

import struct

from output_stream import OutputStream
from tracking_view import TrackingView
from almanac_view import AlmanacView
import flash

class SwiftConsole(HasTraits):
  link = Instance(serial_link.SerialLink)
  console_output = Instance(OutputStream)
  python_console_env = Dict
  a = Int
  b = Int
  tracking_view = Instance(TrackingView)
  almanac_view = Instance(AlmanacView)



  view = View(
      HSplit(
        Item('python_console_env', editor=ShellEditor()),
        Item('console_output', style='custom', editor=InstanceEditor()),
        show_labels=False
    ),
    resizable = True,
    width = 1000,
    height = 600
  )

  def print_message_callback(self, data):
    try:
      self.console_output.write(data.encode('ascii', 'ignore'))
    except UnicodeDecodeError:
      self.console_output.write('[[garbled]]\n');

  def spoon(self, data):
    new_spoon = struct.unpack('<I',data)[0]
    if new_spoon % 10000 == 0:
      try:
        print 'Spoon = %u, spoonterval = %.2f' % (new_spoon, time.time() - self.spoon_time)
      except AttributeError:
        pass
      self.spoon_time = time.time()
    if new_spoon != self.prev_spoon + 1:
      print '!!! Spoon mismatch: got %u, expected %u' % (new_spoon, self.prev_spoon + 1)
    self.prev_spoon = new_spoon

  def __init__(self, port=serial_link.DEFAULT_PORT):
    self.console_output = OutputStream()

    self.link = serial_link.SerialLink(port, serial_link.DEFAULT_BAUD)
    self.link.add_callback(serial_link.MSG_PRINT, self.print_message_callback)
    
    self.prev_spoon = -1;
    self.link.add_callback(0xE0, self.spoon)

    self.flash = flash.Flash(self.link)
    self.python_console_env = {
        'send_message': self.link.send_message,
        'link': self.link,
        'flash': self.flash
    }

  def stop(self):
    self.link.close()

console = SwiftConsole(serial_port)

console.configure_traits()
console.stop()
