from networktables import NetworkTables
import argparse

from Lidar.Lidar import Lidar

FRC_TEAM_NUMBER = 1247
NETWORKTABLES_TABLE = 'ld01'
SERIAL_PORT = '/dev/ttyS0'

def main(team, port, table):
  NetworkTables.initialize(server=f'roborio-{team}-frc.local')
  table = NetworkTables.getTable(table)

  lidar = Lidar(port)
  while True:
    print(lidar.get_data())


parser = argparse.ArgumentParser(description='Sends LIDAR data to NetworkTables')
parser.add_argument('--team', type=int, default=FRC_TEAM_NUMBER, help='FRC team number')
parser.add_argument('--port', type=str, default=SERIAL_PORT, help='Serial port to use')
parser.add_argument('--table', type=str, default=NETWORKTABLES_TABLE, help='NetworkTables table to use')
parser.parse_args()

if __name__ == '__main__':
  main(parser.team, parser.port, parser.table)