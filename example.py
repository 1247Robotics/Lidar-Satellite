#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import serial
from enum import Enum
import struct

from Lidar.GetXYConfidence import GetXYConfidence
from Lidar.ParseData import ParseData
from Lidar.Lidar import Lidar

# ----------------------------------------------------------------------
# System Constants
# ----------------------------------------------------------------------
# Serial port of the LIDAR unit
SERIAL_PORT = "/dev/ttyS0"
# At the default rotation speed (~3600 deg/s) the system outputs about 
# 480 measurements in a full rotation. We want to plot at least this
# many in order to get a 360 degree plot
MEASUREMENTS_PER_PLOT = 480
# System will plot between +/-PLOT_MAX_RANGE on both X and Y axis
PLOT_MAX_RANGE = 4.0 # in meters
# Set the plot area to autoscale to the max distance in X or Y
PLOT_AUTO_RANGE = False
# Show the confidence as a colour gradiant on the plot
PLOT_CONFIDENCE = True
# Set the colour gradiant to use. See this URL for options:
# https://matplotlib.org/stable/gallery/color/colormap_reference.html
PLOT_CONFIDENCE_COLOUR_MAP = "bwr_r"
# Enable debug messages
PRINT_DEBUG = False
# ----------------------------------------------------------------------
# Main Packet Format
# ----------------------------------------------------------------------
# All fields are little endian
# Header (1 byte) = 0x54
# Length (1 byte) = 0x2C (assumed to be constant)
# Speed (2 bytes) Rotation speed in degrees per second
# Start angle (2 bytes) divide by 100.0 to get angle in degrees
# Data Measurements (MEASUREMENT_LENGTH * 3 bytes)
#                   See "Format of each data measurement" below
# Stop angle (2 bytes) divide by 100.0 to get angle in degrees
# Timestamp (2 bytes) In milliseconds
# CRC (1 bytes) Poly: 0x4D, Initial Value: 0x00, Final Xor Value: 0x00
#               Input reflected: False, Result Reflected: False
#               http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
# Format of each data measurement
# Distance (2 bytes) # In millimeters
# Confidence (1 byte)

# ----------------------------------------------------------------------
# Packet format constants
# ----------------------------------------------------------------------
# These do not vary
PACKET_LENGTH = 47
MEASUREMENT_LENGTH = 12 
MESSAGE_FORMAT = "<xBHH" + "HB" * MEASUREMENT_LENGTH + "HHB"

State = Enum("State", ["SYNC0", "SYNC1", "SYNC2", "LOCKED", "UPDATE_PLOT"])

parse_lidar_data = ParseData;
get_xyc_data = GetXYConfidence;

running = True

def on_plot_close(event):
    global running
    running = False

if __name__ == "__main__":
    # Connect up to the LIDAR serial port
    # lidar_serial = serial.Serial(SERIAL_PORT,  230400, timeout=0.5)
    lidar = Lidar(SERIAL_PORT)

    # Set up initial state
    measurements = []
    data = b''
    state = State.SYNC0

    # Set up matplotlib plot
    plt.ion()
    # Force a square aspect ratio
    plt.rcParams['figure.figsize'] = [10, 10]
    plt.rcParams['lines.markersize'] = 2.0
    if PLOT_CONFIDENCE:
        graph = plt.scatter([], [], c=[], marker=".", vmin=0,
                            vmax=255, cmap=PLOT_CONFIDENCE_COLOUR_MAP)
    else:
        graph = plt.plot([], [], ".")[0]
    # Set up the program to shutdown when the plot is closed
    graph.figure.canvas.mpl_connect('close_event', on_plot_close)
    # Limit to +/- PLOT_MAX_RANGE meters
    plt.xlim(-PLOT_MAX_RANGE,PLOT_MAX_RANGE)
    plt.ylim(-PLOT_MAX_RANGE,PLOT_MAX_RANGE)

    # Main state machine
    while running:
        lidar.check_headers();

        # Read remainder of the packet (PACKET_LENGTH minus the 2 header
        # bytes we have already read).
        if state == State.SYNC2:
            data += lidar.read_data()
            if not lidar.data_fits_packet(data):
                state = State.SYNC0
                continue
            measurements += parse_lidar_data(data)
            state = State.LOCKED
        elif state == State.LOCKED:
            data = lidar.read_full_packet()
            if not lidar.first_byte_is_header or not lidar.data_fits_packet(data):
                print("WARNING: Serial sync lost")
                state = State.SYNC0
                continue
            measurements += parse_lidar_data(data)
            if len(measurements) > MEASUREMENTS_PER_PLOT:
                state = State.UPDATE_PLOT
        elif state == State.UPDATE_PLOT:
            x, y, c = get_xyc_data(measurements)
            # Work out max coordinate, and set the scale based on this.
            # Force a 1:1 aspect ratio
            if PLOT_AUTO_RANGE:
                mav_val = max([max(abs(x)), max(abs(y))]) * 1.2
                plt.xlim(-mav_val,mav_val)
                plt.ylim(-mav_val,mav_val)
            # Clear the previous data
            graph.remove()
            # Plot the new data
            if PLOT_CONFIDENCE:
                graph = plt.scatter(x, y, c=c, marker=".",
                                    vmin=0,vmax=255,
                                    cmap=PLOT_CONFIDENCE_COLOUR_MAP)
            else:
                graph = plt.plot(x,y,'b.')[0]
            # Show the new data
            plt.pause(0.00001)
            # Get the next packet
            state = State.LOCKED
            measurements = []
