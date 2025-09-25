# Joey McIntyre
# April 8th, 2025 
# 2DX3 Final Project - Python Code
#
# Import necessary libraries
import math
import numpy as np
import open3d as o3d
import serial

# Initialize the global coordinate variables (x, y, z)
x = 0
y = 0
z = 0

# Main execution block
if __name__ == "__main__":

    # Create a serial connection to COM4 at 115200 baud rate with a timeout of 100 seconds 
    s = serial.Serial('COM4', 115200, timeout = 100)

    # Clear any existing data in the serial buffers
    s.reset_output_buffer()
    s.reset_input_buffer()

    # Wait for user input before starting the program
    input("Press Enter to start program:")

    num_depth_scans = int(input("How many depth measurements (slices along the x-axis): "))      # Ask user for number of depth scans
    distance = int(input("Distance between slices (in mm): "))                                  # Ask user for the fixed displacement between each scan

    # Sets number of scan points per full rotation and calculates angular increment per scan
    scans = int(32)
    angles = 360/scans

    # Instructions for the user
    print("Press the reset button on the microcontroller to get started.")

    # Create a file to save the 3D coordinates data
    f = open("JoeyMcIntyre_FinalProj.xyz", "w")

    # Send 's' to the microcontroller via UART to signal it to start data transmission
    s.write('s'.encode())

    # Read and print initial startup messages from the microcontroller
    for a in range(8): 
        a = s.readline()
        print(a.decode())

    # Sensor is not initialized
    # Inform the user to press the on-board interrupt button to trigger scanning
    print("Press on board interrupt button (PJ1) to begin scanning.")

    # Acquire data for each depth scan
    for i in range(num_depth_scans):
        z = distance*(i-1)          # Depth of current scan

        # Print statements for sensor startup
        for k in range(8): 
            a = s.readline()

        # Error catching
        if (a == b' Please try again...\r\n'): 
            print("Please restart board and code") 

        # Initializes the current angle to 0 for the scan
        curr_angle = 0 

        # Acquire 32 measurements for a full 360deg scan
        for j in range(scans): 
            # Read line from the serial port: should be measurement data
            a = s.readline()

            # Error catching
            if(a != b'0\r\n'): 
                print("out of range")

                # Try to read next line which is expected to be valid measurement
                a = s.readline()
                print(a.decode())

                # Convert received measurement to an integer and calculate the coordinates
                x = int(a)*math.cos(math.radians(curr_angle)) # X component 
                y = int(a)*math.sin(math.radians(curr_angle)) # Y component
                f.write('{0:d} {1:d} {2:d}\n'.format(int(x), int(y), z)) # Write the calculated coordinates to the file

                # Read additional 8 setup lines
                for k in range(8): 
                    a = s.readline()

            else:
                curr_angle = (-1)*(angles*j)    # Change the variable to correct for the current angle of motor
                a = s.readline()                # Read next line which should be measurement

                # Checks if the sensor reports scan failed
                if(a != b'scan failed\r\n'):    # If no error detected
                    print(a.decode())           # Print received measurement (decoded to string)
                    p = a                       # Stores raw data

                    # Convert received measurement to an integer and calculate the coordinates
                    x = int(a)*math.cos(math.radians(curr_angle)) # X component
                    y = int(a)*math.sin(math.radians(curr_angle)) # Y component
                    f.write('{0:d} {1:d} {2:d}\n'.format(int(x), int(y), z))    # Write x, y, z to the file

                else:
                    # If "scan failed" is received, notify the user
                    print('An error occurred')
                    a = s.readline()            # Distance measure will be zero for this error
                    print(a.decode())
                    i -=1                       # Decrement i since depth scan is useless. This allows another depth scan to take place
                    break

        # After finishing one complete scan, read an additional line that indicates the scan is complete
        a = s.readline()   
        print(a.decode())

        # Wait for user to press enter to continue with next depth scan
        input("Press Enter to continue scanning...")

    # Close the files. There should now be a file containing the collected 3D coordinates
    print("Closing: " + s.name)
    s.close()       
    f.close()                       

    # Read the test data in from the file we created        
    print("Read in the prism point cloud data (pcd)")
    pcd = o3d.io.read_point_cloud("JoeyMcIntyre_FinalProj.xyz", format="xyz")

    # Let’s see what our point cloud data looks like numerically       
    print("The PCD array:")
    print(np.asarray(pcd.points))

    # Let’s see what our point cloud data looks like graphically       
    print("Let's visualize the PCD: (spawns separate interactive window)")
    o3d.visualization.draw_geometries([pcd])

    # Create a list with each vertex
    yz_slice_vertex = []
    num_vert = scans*num_depth_scans
    for i in range(0, num_vert):
        yz_slice_vertex.append([i]) # List of individual vertexes 

    # Define coordinates to connect lines in each depth measurement      
    lines = []  # List with all line connections 
    for i in range(0, num_vert, scans):
        for j in range(0, scans, 1):
            if(j == (scans-1)):
                k = 0
            else:
                k = j+1
            lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i+k]]) # Line connections between a single depth reading 

    # Coordinates to connect the different yz slices
    connects = scans*(num_depth_scans-1)
    for i in range(0, connects, scans):
        for j in range(0, scans):
            lines.append([yz_slice_vertex[i+j], yz_slice_vertex[i+scans+j]]) # List of lines between depth readings

    # This line maps the lines to the 3D coordinate vertices
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
                                    lines=o3d.utility.Vector2iVector(lines))

    # Let’s see what our point cloud data with lines looks like graphically       
    o3d.visualization.draw_geometries([line_set])
