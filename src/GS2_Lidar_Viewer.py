# File: GS2_Lidar_Viewer.py
# Author: Robert Stevenson
# Date: 08/09/2023 (DD/MM/YYYY)
# Description:
# Lidar Data Visualizer

from GS2_Lidar import GS2_Lidar

# Main Entry point of the script
if __name__ == "__main__":
    # Open the serial port connection
    lidar = GS2_Lidar(serial_port="COM3")

    # start the lidar
    try:
        lidar.start_scan()

        # get the scan data and print it constantly
        while True:
            distances, thetas = lidar.get_scan_data()
            print("Distances:")
            print(distances)
            print("Angles:")
            print(thetas)
            print("==========================")
    except KeyboardInterrupt:
        pass
    finally:
        lidar.stop_scan()