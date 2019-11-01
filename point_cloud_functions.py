import pyrealsense2 as rs
import time
import numpy as np
import random
import math
import open3d as o3d
import pcl
import pptk
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from scipy.spatial.transform import Rotation as R

# Find the optimal number of cluster that is the number of planes in point cloud
def calculate_wcss(norms, min_n, max_n):
    
    wcss = []
    for n in range(min_n, max_n + 1):
        kmeans = KMeans(n_clusters=n, init='k-means++', random_state=167)
        kmeans.fit(norms)
        wcss.append(kmeans.inertia_)
    
    return wcss

def optimal_num_of_clus(wcss, min_n, max_n):
    
    x1, y1 = min_n, wcss[0]
    x2, y2 = max_n, wcss[len(wcss) - 1]
    
    distances = []
    
    for i in range(len(wcss)):
        x0 = i + min_n
        y0 = wcss[i]
        
        segment = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        norm = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
        distances.append(segment / norm)
    
    return distances.index(max(distances)) + min_n

def optimal_num_clus(norms, min_n, max_n):
    
    wcss = calculate_wcss(norms, min_n, max_n)
    
    return optimal_num_of_clus(wcss, min_n, max_n)

def select_point_from_group(group, pts):
    """ Choose the points from index in groups
        Input: group with index
        Output: lists of points in the group
    """

    p_in_group = []

    for i in range(len(group)):
        p_in_group.append(pts[group[i]])
    
    return p_in_group

def fit_to_plane(pts):
    """Given points in a group, compute the plane which minimizes the distance from the points to the plane
       Output the norm vector norm and constant factor d
    """
    # Compute x_mean, y_mean, z_mean
    
    n = len(pts)
    
    x_total = 0
    y_total = 0
    z_total = 0

    for i in range(n):
        x_total += pts[i][0]
        y_total += pts[i][1]
        z_total += pts[i][2]

    x_mean = x_total * 1.0 / n
    y_mean = y_total * 1.0 / n
    z_mean = z_total * 1.0 / n

    # Compute the p[i] = [x[i]-x_mean,y[i]-y.mean,z[i]-z.mean]
    p = []
    for i in range(n):
        p1 = pts[i][0] - x_mean
        p2 = pts[i][1] - y_mean
        p3 = pts[i][2] - z_mean
        p.append([p1, p2, p3])
 
    # Compute the matrix A
    a1 = 0
    a2 = 0
    a3 = 0
    a4 = 0
    a5 = 0
    a6 = 0
    for i in range(n):
        a1 += p[i][0] * p[i][0]
        a2 += p[i][0] * p[i][1]
        a3 += p[i][0] * p[i][2]
        a4 += p[i][1] * p[i][1]
        a5 += p[i][1] * p[i][2]
        a6 += p[i][2] * p[i][2]

    A = np.array([[a1, a2, a3], [a2, a4, a5], [a3, a5, a6]])

    # Compute the smallest eigen value and accordingly eigen vector of A
    w, v = np.linalg.eigh(A)

    # The minimal eigenvalue is w[0]
    eig = w[0]

    # The norm is eigenvector v[:,0]
    norm = v[:,0].tolist()
    d = -norm[0] * x_mean - norm[1] * y_mean - norm[2] * z_mean

    return norm, d

def norm_align(norm, d, reference):
    
    """Let the norm of plane along with the estimated normal
    """
    
    if(np.dot(norm, reference) < 0):
        norm = [x * (-1.0) for x in norm]
        d = d * (-1.0)
    return norm, d

def proj_to_plane(norm, d, pts):
    """ Project points on to a certain plane
        Input: plane's norm (normalized), and a bunch of points
        Output: All the projection of points on the plane

    """
    a = norm[0]
    b = norm[1]
    c = norm[2]

    p = []

    for i in range(len(pts)):
        x_p = pts[i][0]
        y_p = pts[i][1]
        z_p = pts[i][2]

        if a != 0:
            x_0 = (b * b + c * c) * x_p - a * b * y_p - a * c * z_p - a * d
            y_0 = (b * 1.0 / a) * (x_0 - x_p) + y_p
            z_0 = (c * 1.0 / a) * (x_0 - x_p) + z_p

        elif b != 0:
            x_0 = x_p 
            y_0 = c * c * y_p - b * (d + c)
            z_0 = (c * 1.0 / b) *(y_0 - y_p) + z_p

        else:
            x_0 = x_p
            y_0 = y_p
            z_0 = - d * 1.0 / c

        p.append([x_0, y_0, z_0])
        
    return p

def build_coord(norm, d, pts):
    """Given the function of a plane (in global accordinate system) and a lot of points on the plane,
       construct an accordinate system
       Set the origin in the center of points.
       Output the coordinates
    """
    # Compute the origin as the mean point of the points, and this point has to be on the plane
    
    n = len(pts) 
    x_total = 0
    y_total = 0
    z_total = 0
    
    for i in range(n):
        x_total += pts[i][0]
        y_total += pts[i][1]
        z_total += pts[i][2]

    x_o = x_total * 1.0 / n
    y_o = y_total * 1.0 / n
    z_o = z_total * 1.0 / n
    p_o = [x_o, y_o, z_o]
    
    # Choose p be the projection of a vector in the z-axis to the plane
    # If the plane is not perpendicular to the z-axis
    if ((norm[2] != 1) and (norm[2] != -1)): 
        # Choose a point
        o_z = [x_o, y_o, z_o + 1]
    
        [[x_p, y_p, z_p]] = proj_to_plane(norm, d, [o_z])
     
        dist = np.linalg.norm([x_p - x_o, y_p - y_o, z_p - z_o])

        x_c = (x_p - x_o) * 1.0 / dist 
        y_c = (y_p - y_o) * 1.0 / dist
        z_c = (z_p - z_o) * 1.0 / dist
        # Thus we have unit vector in x direction
        e_y = [x_c, y_c, z_c]
        #Compute the unit vector in y direction
        e_x = np.cross(e_y, norm).tolist()
    else:
        e_x = [1, 0, 0]
        e_y = [0, 1, 0]
        
    return [e_x, e_y, norm] , p_o

def compute_T_matrix(coordinates, p, reference=[[1,0,0],[0,1,0],[0,0,1]], origin=[0,0,0]):
    """Compute the tranform matrix from origin to the coordinates and its inverse
    """
    e_b_x = coordinates[0]
    e_b_y = coordinates[1]
    e_b_z = coordinates[2]
    
    e_a_x = reference[0] 
    e_a_y = reference[1]
    e_a_z = reference[2]
    
    # Compute the rotation matrix
    x_b_a = [np.dot(e_b_x, e_a_x), np.dot(e_b_x, e_a_y), np.dot(e_b_x, e_a_z)]
    y_b_a = [np.dot(e_b_y, e_a_x), np.dot(e_b_y, e_a_y), np.dot(e_b_y, e_a_z)]
    z_b_a = [np.dot(e_b_z, e_a_x), np.dot(e_b_z, e_a_y), np.dot(e_b_z, e_a_z)]
    
    R_b_a = [[x_b_a[0], y_b_a[0], z_b_a[0]],[x_b_a[1], y_b_a[1], z_b_a[1]],x_b_a[2], y_b_a[2], z_b_a[2]]
    
    # Compute the displacement 
    displacement = [p[0]-origin[0], p[1]-origin[1], p[2]-origin[2]]
    
    # Make it into a transform matrix
    T_b_a = [[x_b_a[0], y_b_a[0], z_b_a[0], displacement[0]],
         [x_b_a[1], y_b_a[1], z_b_a[1], displacement[1]],
         [x_b_a[2], y_b_a[2], z_b_a[2], displacement[2]],
         [0, 0, 0, 1]]
    
    T_a_b = np.linalg.inv(T_b_a).tolist()
    
    return T_b_a, T_a_b

def trans_to_coordinates(T, pts):
    """Transfer all the points to a new fram using transformation matrix.
    """
    p = []
    for i in range(len(pts)):
        
        p_b = [pts[i][0], pts[i][1], pts[i][2], 1]
        p_a = np.matmul(T, p_b).tolist()
        p.append(p_a[0:3])

    return p

def trans_to_end(pts,dx=0.05, dz=0.03):
    """Transfer to the end-effector coordinate
    """
    p = []
    for i in range(len(pts)):
        p.append([pts[i][0]+dx, pts[i][1],pts[i][2]+dz])
    return p

def trans_to_joint(pts,dx=0.05,dz=0.135):
    """Transfer to the last joint coordinate
    """
    p = []
    for i in range(len(pts)):
        p.append([pts[i][0]+dx, pts[i][1],pts[i][2]+dz])
    return p

def segement_divide(pts,step=0.10, offset_x=0.01, offset_y=0.0):
    """Divide the points on the plane into a sequence of rectangles.
    
    """

    # Select the x and y of the points
    n = len(pts)
    
    z = pts[0][2]
    
    points_plane = []    
    points_x = []
    paint_point = []

    for i in range(n):
        points_plane.append([pts[i][0], pts[i][1]])
        
    # Sorted the list according to x 
    points_plane.sort(key=lambda x:x[0])

    # Segment the points according to x 
    counter = 0   # Count the interval
    x_min = points_plane[0][0]
    x_max = points_plane[n-1][0]

    # The whole interval that needs to be divided
    upper = x_max + offset_x
    lower = x_min - offset_x
    lower_bound = lower
    
    # Set each segement's lower and upperbound
    while (lower_bound + step <= upper):       
        # The break condition will be lower_bound > upper - step
        upper_bound = lower_bound + step

        # Find the index between lower bound and upper bound
        # First, find the index which x >= lower bound
        index = 0
        
        while (points_plane[index][0] < lower_bound): 
            index = index + 1  # The index of the first point in the interval
            
        # If there is at least one point in the [lower_bound, upper_bound]
        if (points_plane[index][0] <= upper_bound): 

            x_start = points_plane[index][0]
            y_max = points_plane[index][1]
            y_min = points_plane[index][1]
        
            while (points_plane[index][0] <= upper_bound): 
                # The break condition will be x[index] > upper bound or index = n - 1
                # Compute the y max and y min in this interval
                
                if points_plane[index][1] > y_max:   
                    y_max = points_plane[index][1]

                if points_plane[index][1] < y_min:
                    y_min = points_plane[index][1]
                
                if index < n - 1:
                    index = index + 1
                else:
                    break
            # The index of the last point in the interval, when index < n-1
            
            x_end = points_plane[index][0]

            paint_point.append([lower_bound,y_max+offset_y,z]) 
            paint_point.append([lower_bound,y_min-offset_y,z])
            points_x.append([x_start, x_end])
            
        counter = counter + 1

        # Update interval
        lower_bound = upper_bound - offset_x
    
    # Deal with the last interval
    lower_bound_last = upper - step
    index_last = 0
    counter = counter + 1
    while ((index_last < n) and (points_plane[index_last][0] < lower_bound_last)): 
        # The first point in the last interval
        index_last = index_last + 1
        
    if (index_last < n): 
        # There is at least one point in the last interval
        x_start_last = points_plane[index_last][0]
        y_max_last = points_plane[index_last][1]
        y_min_last = points_plane[index_last][1]

        while ((index_last)<n) and (points_plane[index_last][0] <= upper):

            if points_plane[index_last][1] > y_max_last:   
                y_max_last = points_plane[index_last][1]
            
            if points_plane[index_last][1] < y_min_last:
                y_min_last = points_plane[index_last][1]

            index_last = index_last + 1
            
        index_last = index_last - 1 # The index of the last point in the interval
        
        paint_point.append([lower_bound_last, y_max_last+offset_y, z])
        paint_point.append([lower_bound_last, y_min_last-offset_y, z])
#   paint_point.append([upper, y_max_last+offset_y, z])
#   paint_point.append([upper, y_min_last-offset_y, z])
# return trans_to_end(paint_point)
        return paint_point        

def coor_to_quater(coordinate):
    """Transfer dcm to quaternion
    """
    coord = np.transpose(coordinate)
    
    r = R.from_dcm(coord)
    
    q = r.as_quat()
    # m = r.as_dcm()
    
    return q.tolist()

def norm_to_base(coordinate):
    [f_x, f_y, f_z] = coordinate
    e_x = [y * (-1.0) for y in f_y]
    e_y = [x * (-1.0) for x in f_x]
    e_z = [z * (-1.0) for z in f_z] 
    return [e_x, e_y, e_z]

