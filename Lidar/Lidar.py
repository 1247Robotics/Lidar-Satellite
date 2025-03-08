from enum import Enum
import serial

from Lidar.ParseData import ParseData
from Lidar.GetXYConfidence import GetXYConfidence
from queue import Queue
from queue import Empty

State = Enum("State", ["SYNC0", "SYNC1", "SYNC2", "LOCKED", "PUSH_LATEST"])

FIRST_HEADER = b'\x54'
SECOND_HEADER = b'\x2C'

PACKET_LENGTH = 47

MEASUREMENTS_PER_PLOT = 480

class Lidar:
  state = State.SYNC0
  data = b''
  measurements = []
  output = Queue()
  run = True

  def __init__(self, port):
    self.lidar_serial = serial.Serial(port, 230400, timeout=0.5)
    self.measurements = []

  def check_header1(self):
    return self.lidar_serial.read() == FIRST_HEADER

  def check_header2(self):
    return self.lidar_serial.read() == SECOND_HEADER

  def read_data(self):
    return self.lidar_serial.read(PACKET_LENGTH - 2)

  def data_fits_packet(self, data):
    return len(data) == PACKET_LENGTH

  def read_full_packet(self):
    return self.lidar_serial.read(PACKET_LENGTH)

  def first_byte_is_header(self, data):
    return data[0] == FIRST_HEADER

  def has_lost_sync(self, data):
    return not self.first_byte_is_header(data) or not self.data_fits_packet(data)

  def check_headers(self):
    if self.state == State.SYNC0:
      self.data = b''
      self.measurements = []
      if self.check_header1():
        self.data = FIRST_HEADER
        self.state = State.SYNC1
      return True;
    
    elif self.state == State.SYNC1:
      if self.check_header2():
        self.data += SECOND_HEADER
        self.state = State.SYNC2
      else:
        self.state = State.SYNC0
      return True

    return False


  def intake_data(self):
    if self.state == State.SYNC2:
      self.data += self.read_data()
      if not self.data_fits_packet(self.data):
        self.state = State.SYNC0
        return
      self.measurements += ParseData(self.data)
      self.state = State.LOCKED

    elif self.state == State.LOCKED:
      self.data = self.read_full_packet()
      if self.has_lost_sync(self.data):
        self.state = State.SYNC0
        return
      self.measurements += ParseData(self.data)
      if len(self.measurements) > MEASUREMENTS_PER_PLOT:
        self.state = State.PUSH_LATEST

    elif self.state == State.PUSH_LATEST:
      x, y, c = GetXYConfidence(self.measurements)

      if not self.output.empty():
        try:
          self.output.get_nowait()
        except Empty:
          pass
      
      self.output.put((x, y, c))
      self.state = State.SYNC0
      self.measurements = []

  def run_lidar(self):
    self.run = True
    while self.run:
      self.check_headers()
      self.intake_data()

  def stop_lidar(self):
    self.run = False

  def get_data(self):
    return self.output.get()