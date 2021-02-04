# For each cylinder in the scan, find its ray and depth.
# 03_c_find_cylinders
# Claus Brenner, 09 NOV 2012
from pylab import *
from lego_robot import *

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
            # --->>> Insert your cylinder code here.
        # Whenever you find a cylinder, add a tuple
        # (average_ray, average_depth) to the cylinder_list.

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

        '''elif on_cylinder and Right:
            if curr_der<0:
                print('[INFO] Right Repeat found. Resetting values')
                sum_ray, sum_depth, rays = 0.0, 0.0, 0
            if curr_der > 0:
                print("----end cylinder -----")
                depth_avg = sum_depth/rays
                ray_avg = sum_ray/rays
                cylinder_list.append((ray_avg,depth_avg))
                on_cylinder = False
                Right = False
                sum_ray = 0.0
                sum_depth = 0.0
                rays = 0
            else:
                rays +=1
                sum_depth += scan[i]
                sum_ray += i'''   

    return cylinder_list


if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 100.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Pick one scan.
    scan = logfile.scan_data[8]

    # Find cylinders.
    der = compute_derivative(scan, minimum_valid_distance)
    cylinders = find_cylinders(scan, der, depth_jump,
                               minimum_valid_distance)

    # Plot results.
    plot(scan)
    scatter([c[0] for c in cylinders], [c[1] for c in cylinders],
        c='r', s=200)
    show()
