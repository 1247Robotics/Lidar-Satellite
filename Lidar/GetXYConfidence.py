

import numpy as np


def GetXYConfidence(measurements):
  angle = np.array([measurement[0] for measurement in measurements])
  distance = np.array([measurement[1] for measurement in measurements])
  confidence = np.array([measurement[2] for measurement in measurements])

  x = np.sin(np.radians(angle)) * (distance / 1000.0)
  y = np.cos(np.radians(angle)) * (distance / 1000.0)
  
  return x, y, confidence