import struct

MEASUREMENT_LENGTH = 12 
MESSAGE_FORMAT = "<xBHH" + "HB" * MEASUREMENT_LENGTH + "HHB"

def ParseData(data):
  length, speed, start_angle, *pos_data, stop_angle, timestamp, crc = struct.unpack(MESSAGE_FORMAT, data);

  start_angle = float(start_angle) / 100.0
  stop_angle = float(stop_angle) / 100.0

  if stop_angle < start_angle:
    stop_angle += 360.0

  step_size = (stop_angle - start_angle) / (MEASUREMENT_LENGTH - 1)

  angle = [start_angle + step_size * i for i in range(0,MEASUREMENT_LENGTH)]
  distance = pos_data[0::2]
  confidence = pos_data[1::2]

  return list(zip(angle, distance, confidence))