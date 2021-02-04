# For each cylinder in the scan, find its cartesian coordinates,
# in the scanner's coordinate system.
# Write the result to a file which contains all cylinders, for all scans.
# 03_d_find_cylinders_cartesian
# Claus Brenner, 09 NOV 2012
from lego_robot import *
from math import sin, cos

# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):
    jumps = [ 0 ]
    for i in range(1, len(scan) - 1):
        l = scan[i-1]
        r = scan[i+1]
        if l > min_dist and r > min_dist:
            derivative = (r - l) / 2.0
            jumps.append(derivative)
        else:
            jumps.append(0)
    jumps.append(0)
    return jumps

# For each area between a left falling edge and a right rising edge,
# determine the average ray number and the average depth.
def find_cylinders(scan, scan_derivative, jump, min_dist):
     # --->>> Insert here your previous solution from find_cylinders_question.py.
    cylinder_list = []
    on_cylinder = False
    Left = False
    Right = False
    repeat = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0

    for i in range(len(scan_derivative)):
        curr_der = scan_derivative[i]

        if ((abs(curr_der) > jump)):
            on_cylinder = True
            if curr_der > 0:
                Right = True
            elif curr_der < 0:
                Left = True

        if on_cylinder and Left:
            if curr_der<0:
                #print('[INFO] Left Repeat found. Resetting values')
                sum_ray, sum_depth, rays = 0.0, 0.0, 0
            if curr_der > 0:
                #print("----end cylinder -----")
                depth_avg = sum_depth/rays
                ray_avg = sum_ray/rays
                cylinder_list.append((ray_avg,depth_avg))
                on_cylinder = False
                Left = False
                sum_ray = 0.0
                sum_depth = 0.0
                rays = 0
            else:
                rays +=1
                sum_depth += scan[i]
                sum_ray += i

    return cylinder_list

def compute_cartesian_coordinates(cylinders, cylinder_offset):
    result = []
    for c in cylinders:
        # --->>> Insert here the conversion from polar to Cartesian coordinates.
        # c is a tuple (beam_index, range).
        # For converting the beam index to an angle, use
        # LegoLogfile.beam_index_to_angle(beam_index)
        theta  = LegoLogfile.beam_index_to_angle(c[0])
        dist = c[1]
        cx = (dist+cylinder_offset)*cos(theta)
        cy =  (dist+cylinder_offset)*sin(theta)
        result.append( (cx,cy) ) # Replace this by your (x,y)
    return result
        

if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Write a result file containing all cylinder records.
    # Format is: D C x[in mm] y[in mm] ...
    # With zero or more points.
    # Note "D C" is also written for otherwise empty lines (no
    # cylinders in scan)
    out_file = open("cylinders.txt", "w")
    for scan in logfile.scan_data:
        # Find cylinders.
        der = compute_derivative(scan, minimum_valid_distance)
        cylinders = find_cylinders(scan, der, depth_jump,
                                   minimum_valid_distance)
        cartesian_cylinders = compute_cartesian_coordinates(cylinders,
                                                            cylinder_offset)
        # Write to file.
        print ("D C",file = out_file)
        for c in cartesian_cylinders:
            print("%.1f %.1f" % c, file = out_file)
        #print(file=out_file)
    out_file.close()
