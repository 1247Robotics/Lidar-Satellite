import socket
import random

DISCOVERY_PORT = 5556
DISCOVER_MESSAGE = b"LIDAR_DISCOVERY"

RANDOM_ID = random.randint(0, 0xFFFFFFFF).to_bytes(4, byteorder='big')

def FindC2():
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  sock.settimeout(5)

  tries = 0

  while tries < 1000:
    sock.sendto(DISCOVER_MESSAGE + RANDOM_ID, ('<broadcast>', DISCOVERY_PORT))

    try:
      data, addr = sock.recvfrom(1024)
      if data == RANDOM_ID:
        return addr[0]
    except socket.timeout:
      pass
    tries += 1