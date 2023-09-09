# File: GS2_Lidar_Viewer.py
# Author: Robert Stevenson
# Date: 08/09/2023 (DD/MM/YYYY)
# Description:
# Lidar Data Visualizer

from GS2_Lidar import GS2_Lidar
import matplotlib.pyplot as plt
import cmath as math

# Main Entry point of the script
if __name__ == "__main__":
    # Open the serial port connection
    lidar = GS2_Lidar(serial_port="COM3")

    # start the lidar
    try:
        lidar.start_scan()
        
        plt.axes().set_xlim(-350, 350)
        plt.axes().set_ylim(-10, 350)

        # get the scan data and print it constantly
        while True:
            distances, thetas = lidar.get_scan_data()
            x = []
            y = []

            for i in range(len(distances)):
                x.append(distances[i]*math.sin(thetas[i]).real)
                y.append(distances[i]*math.cos(thetas[i]).real)
            
            plt.clf()
            plt.scatter(x, y)
            plt.pause(0.1)

    except KeyboardInterrupt:
        plt.close()
    finally:
        lidar.stop_scan()