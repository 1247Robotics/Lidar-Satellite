import socket
import random
from enum import Enum
import json
import zlib
from queue import Queue
from cachetools import TTLCache

DISCOVERY_PORT = 5556
OPERATING_PORT = 5557
DISCOVER_MESSAGE = b"LIDAR_DISCOVERY"

RANDOM_ID = random.randint(0, 0xFFFFFFFF).to_bytes(4, byteorder='big')

def hash(data):
  return zlib.crc32(data.encode())

def random_id():
  return random.randint(0, 0xFFFFFFFF)

class Communication:
  c2_ip = None
  sock = None
  inboundQueue = Queue()
  outboundQueue = Queue()
  outboundCache = TTLCache(maxsize=256, ttl=16)
  awaitingResend = TTLCache(maxsize=256, ttl=16)

  def __init__(self):
    print("Starting Communication")
    print("Creating socket")
    self.create_socket()
    print("Entering C2 discovery loop")
    self.c2_ip = self.discover_c2()
    if self.c2_ip is None:
      print("Failed to discover C2")
      print("Exiting")
      return
    print("Recieved Response")
    print("C2 IP: " + self.c2_ip)

    print("Connecting to C2")
    self.connect_to_c2()
    print("Connected to C2")
  
  def create_socket(self):
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.sock.bind(('', 0))

  def discover_c2(self):
    self.sock.settimeout(5)

    tries = 0

    while tries < 1000:
      self.sock.sendto(DISCOVER_MESSAGE + RANDOM_ID, ('<broadcast>', DISCOVERY_PORT))

      try:
        data, addr = self.sock.recvfrom(1024)
        if data == RANDOM_ID:
          return addr[0]
      except socket.timeout:
        pass
      tries += 1

    return None

  def connect_to_c2(self):
    self.sock.connect((self.c2_ip, OPERATING_PORT))

  def intake_inbound_loop(self):
    while True:
      data = self.sock.recv(4196)
      self.inboundQueue.put(data)

  def send_single_payload(self, payload):
    message_id = random_id()
    payload = json.dumps(payload)
    hashedPayload = hash(payload)

    data = {
      "hash": hashedPayload,
      "message_id": message_id,
      "payload": payload
    }
    data = json.dumps(data)

    self.outboundCache[message_id] = data

    self.sock.send(data.encode())

  def send_one_from_queue(self):
    if not self.outboundQueue.empty():
      payload = self.outboundQueue.get()
      self.send_single_payload(payload)

  def decode_data(self, data):
    data = data.decode()
    data = json.loads(data)

    hash = data["hash"]
    message_id = data["message_id"]
    payload = data["payload"]

    malformed = False

    hashedPayload = hash(payload)

    if hash != hashedPayload:
      malformed = True

    return (message_id, payload, malformed)

  def handleSingleIntake(self, data):
    message_id, payload, malformed = self.decode_data(data)