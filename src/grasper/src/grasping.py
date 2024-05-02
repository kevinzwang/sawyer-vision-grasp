#!/usr/bin/env python
"""
Starter code for EE106B grasp planning project.
Author: Amay Saxena, Tiffany Cappellari
Modified by: Kirthi Kumar
#!/usr/bin/env python -W ignore::DeprecationWarning
"""
# may need more imports
import numpy as np
import utils
import math
import trimesh
import vedo
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
import tf2_geometry_msgs
import tf


MAX_GRIPPER_DIST = 0.075
MIN_GRIPPER_DIST = 0.03
GRIPPER_LENGTH = 0.11

import cvxpy as cvx # suggested, but you may change your solver to anything you'd like (ex. casadi)

def compute_force_closure(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Compute the force closure of some object at contacts, with normal vectors 
    stored in normals. Since this is two contact grasp, we are using a basic algorithm
    wherein a grasp is in force closure as long as the line connecting the two contact
    points lies in both friction cones.

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient
    object_mass (float): mass of the object

    Returns
    -------
    (float): quality of the grasp
    """
    vertices = vertices.T
    
    line = vertices[0] - vertices[1]
    line /= np.linalg.norm(line)
    
    normals = normals.T
    normals[0] /= np.linalg.norm(normals[0])
    normals[1] /= np.linalg.norm(normals[1])
    
    angle1 = np.arccos(normals[0].T @ line)
    angle1 = min(angle1, np.pi - angle1)
    
    angle2 = np.arccos(normals[1].T @ line)
    angle2 = min(angle2, np.pi - angle2)
    
    alpha = np.arctan(mu)
    
    return angle1 < alpha and angle2 < alpha and gamma > 0
    
def get_grasp_map(vertices, normals, num_facets, mu, gamma):
    """ 
    Defined in the book on page 219. Compute the grasp map given the contact
    points and their surface normals

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient

    Returns
    -------
    (np.ndarray): grasp map
    """
    
    B = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 1]
    ])
    
    normal1 = utils.normalize(-normals[0])
    vertex1 = vertices[0]
    
    g_oc1 = utils.look_at_general(vertex1, normal1)
    adj_T_inv1 = utils.adj(np.linalg.inv(g_oc1)).T
            
    G_1 = adj_T_inv1 @ B
        
    
    normal2 = utils.normalize(-normals[1])
    vertex2 = vertices[1]
    
    g_oc2 = utils.look_at_general(vertex2, normal2)
    adj_T_inv2 = utils.adj(np.linalg.inv(g_oc2)).T
            
    G_2 = adj_T_inv2 @ B
            
    return np.hstack((G_1, G_2))

def contact_forces_exist(vertices, normals, num_facets, mu, gamma, desired_wrench):
    """
    Compute whether the given grasp (at contacts with surface normals) can produce 
    the desired_wrench. Will be used for gravity resistance. 

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient
    desired_wrench (np.ndarray):potential wrench to be produced

    Returns
    -------
    (bool): whether contact forces can produce the desired_wrench on the object
    """
    
    G = get_grasp_map(vertices, normals, num_facets, mu, gamma)
    
    fc = cvx.Variable(8)
    
    # constraints = [np.sqrt(FC[0]**2 + FC[1]**2) <= mu * FC[2], FC[2] > 0, np.abs(FC[3] <= gamma*FC[2]), 
    #                                         np.sqrt(FC[5]**2 + FC[6]**2) <= mu * FC[7], FC[7] > 0, np.abs(FC[8] <= gamma*FC[7])]

    constraints = []
    
    for fc_i in (fc[:4], fc[4:]):
        constraints += [
            # cvx.sqrt(fc_i[0]**2 + fc_i[1]**2) <= mu * fc_i[2],
            fc_i[2] >= 0,
            cvx.abs(fc_i[3]) <= gamma * fc_i[2],
        ]
        
    # friction cone constraints
    f_i_z = np.cos(np.arctan(mu)) / np.sqrt(1 - np.cos(np.arctan(mu))**2)
    F = [[0, 0, 1]]
    for i in range(num_facets):
        angle = 2 * np.pi * i / num_facets
        F.append([np.cos(angle), np.sin(angle), f_i_z])
        
    F = np.array(F).T
        
    alpha = cvx.Variable((2, num_facets+1))
    for (i, fc_i) in enumerate((fc[:4], fc[4:])):
        # print(F.shape, alpha[i].shape)
        constraints.append(fc_i[:3] == F @ alpha[i])
    constraints.append(alpha >= 0)

    cost = cvx.sum_squares(desired_wrench - G @ fc)
    prob = cvx.Problem(cvx.Minimize(cost), constraints)
    
    try:
        prob.solve()
    except Exception as e:
        print("problem solve error:", e)
        return False, None
    
    if prob.status != 'optimal':
        print(prob.status)


    # print(fc.value)
    return prob.status == 'optimal', fc.value
    
def compute_gravity_resistance(vertices, normals, num_facets, mu, gamma, object_mass):
    """
    Gravity produces some wrench on your object. Computes whether the grasp can 
    produce an equal and opposite wrench.

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient
        torsional friction coefficient
    object_mass (float): mass of the object

    Returns
    -------
    (float): quality of the grasp
    """
    # YOUR CODE HERE
    desired_wrench = np.array([0, 0, 9.8 * object_mass, 0, 0, 0]).T
    
    statement, _ = contact_forces_exist(vertices, normals, num_facets, mu, gamma, desired_wrench)
    
    return statement

"""
you're encouraged to implement a version of this method, 
def sample_around_vertices(delta, vertices, object_mesh=None):
    raise NotImplementedError
"""

def compute_robust_force_closure(vertices, normals, num_facets, mu, gamma, object_mass, object_mesh):
    """
    Should return a score for the grasp according to the robust force closure metric.

    Parameters
    ----------
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed
    normals (2x3 np.ndarray): obj mesh normals at the contact points
    num_facets (int): number of vectors to use to approximate the friction cone, vectors 
        will be along the friction cone boundary
    mu (float): coefficient of friction
    gamma (float): torsional friction coefficient
        torsional friction coefficient
    object_mass (float): mass of the object

    Returns
    -------
    (float): quality of the grasp
    """
    desired_wrench = np.array([0, 0, 9.8 * object_mass, 0, 0, 0]).T
        
    delta = 0.01
    trials = 250
    p1_list = np.random.uniform(vertices[0]-delta, vertices[0]+delta, (trials, 3))
    p2_list = np.random.uniform(vertices[1]-delta, vertices[1]+delta, (trials, 3))
    
    full_trails = 0
    success = 0
    
    for p1, p2 in zip(p1_list, p2_list):
        ray_dir = p2 - p1
        
        p1_locs_in, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p1],
            ray_directions=[ray_dir],
            multiple_hits=True
        )
        
        p1_locs_out, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p1],
            ray_directions=[-ray_dir],
            multiple_hits=True
        )
        
        if len(p1_locs_in) % 2 != 0 or len(p1_locs_in) == 0 or len(p1_locs_out) != 0:
            # print("p1 inside mesh")
            continue
        
        p2_locs_in, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p2],
            ray_directions=[-ray_dir],
            multiple_hits=True
        )
        
        p2_locs_out, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p2],
            ray_directions=[ray_dir],
            multiple_hits=True
        )
        
        if len(p2_locs_in) % 2 != 0 or len(p2_locs_in) == 0 or len(p2_locs_out) != 0:
            # print("p2 inside mesh")
            continue
        
        vertices, _ = utils.find_grasp_vertices(object_mesh, p1, p2)
        
        v1, v2 = vertices
        
        grip_dist = np.linalg.norm(v1 - v2)
        
        if not (MIN_GRIPPER_DIST <= grip_dist <= MAX_GRIPPER_DIST):
            # print("ray outside of gripper bounds")
            continue
        
        if v1[2] <= 0 or v2[2] <= 0:
            # print("point under object")
            continue
        
        n1 = utils.normal_at_point(object_mesh, v1)
        n2 = utils.normal_at_point(object_mesh, v2)
        
        normals = np.array([n1, n2])
        
        full_trails += 1
        
        statement, _ = contact_forces_exist(vertices, normals, num_facets, mu, gamma, desired_wrench)
        if statement:
            success += 1
            
    return success/full_trails, full_trails    

def custom_grasp_planner(object_mesh, vertices):
    """
    Write your own grasp planning algorithm! You will take as input the mesh
    of an object, and a pair of contact points from the surface of the mesh.
    You should return a 4x4 ridig transform specifying the desired pose of the
    end-effector (the gripper tip) that you would like the gripper to be at
    before closing in order to execute your grasp.

    You should be prepared to handle malformed grasps. Return None if no
    good grasp is possible with the provided pair of contact points.
    Keep in mind the constraints of the gripper (length, minimum and maximum
    distance between fingers, etc) when picking a good pose, and also keep in
    mind limitations of the robot (can the robot approach a grasp from the inside
    of the mesh? How about from below?). You should also make sure that the robot
    can successfully make contact with the given contact points without colliding
    with the mesh.

    The trimesh package has several useful functions that allow you to check for
    collisions between meshes and rays, between meshes and other meshes, etc, which
    you may want to use to make sure your grasp is not in collision with the mesh.

    Take a look at the functions find_intersections, find_grasp_vertices, 
    normal_at_point in utils.py for examples of how you might use these trimesh 
    utilities. Be wary of using these functions directly. While they will probably 
    work, they don't do excessive edge-case handling. You should spend some time
    reading the documentation of these packages to find other useful utilities.
    You may also find the collision, proximity, and intersections modules of trimesh
    useful.

    Feel free to change the signature of this function to add more arguments
    if you believe they will be useful to your planner.

    Parameters
    ----------
    object_mesh (trimesh.base.Trimesh): A triangular mesh of the object, as loaded in with trimesh.
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed

    Returns
    -------
    (4x4 np.ndarray): The rigid transform for the desired pose of the gripper, in the object's reference frame.
    """

    # constants -- you may or may not want to use these variables below
    num_facets = 64
    mu = 0.5
    gamma = 0.1
    object_mass = 0.25
    g = 9.8
    desired_wrench = np.array([0, 0, g * object_mass, 0, 0, 0]).T

    trials = 1000
    delta = 0.04
    ferrari_th = 0.31 #Nozzle 0.6, Pawn 0.3
    RFC_bound = 0.94
    
    p1_list = np.random.uniform(vertices[0]-delta, vertices[0]+delta, (trials, 3))
    p2_list = np.random.uniform(vertices[1]-delta, vertices[1]+delta, (trials, 3))

    # YOUR CODE HERE
    for p1, p2 in zip(p1_list, p2_list):        
        ray_dir = p2 - p1
        
        p1_locs_in, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p1],
            ray_directions=[ray_dir],
            multiple_hits=True
        )
        
        p1_locs_out, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p1],
            ray_directions=[-ray_dir],
            multiple_hits=True
        )
        
        if len(p1_locs_in) % 2 != 0 or len(p1_locs_in) == 0 or len(p1_locs_out) != 0:
            # print("p1 inside mesh")
            continue
        
        p2_locs_in, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p2],
            ray_directions=[-ray_dir],
            multiple_hits=True
        )
        
        p2_locs_out, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p2],
            ray_directions=[ray_dir],
            multiple_hits=True
        )
        
        if len(p2_locs_in) % 2 != 0 or len(p2_locs_in) == 0 or len(p2_locs_out) != 0:
            # print("p2 inside mesh")
            continue
        
        vertices, _ = utils.find_grasp_vertices(object_mesh, p1, p2)
        
        v1, v2 = vertices
        
        grip_dist = np.linalg.norm(v1 - v2)
        
        if not (MIN_GRIPPER_DIST <= grip_dist <= MAX_GRIPPER_DIST):
            # print("ray outside of gripper bounds")
            continue
        
        if v1[2] <= 0 or v2[2] <= 0:
            # print("point under object")
            continue
        
        n1 = utils.normal_at_point(object_mesh, v1)
        n2 = utils.normal_at_point(object_mesh, v2)
        
        normals = np.array([n1, n2])
        
        if not compute_force_closure(vertices, normals, num_facets, mu, gamma, object_mass):
            print("grasp not force closure")
            continue
        
        # Gravity
        # if not compute_gravity_resistance(vertices, normals, num_facets, mu, gamma, object_mass):
        #     print("grasp cannot get desired wrench for gravity")
        #     continue
        # quality = 1
        
        # RFC
        # quality = compute_robust_force_closure(vertices, normals, num_facets, mu, gamma, object_mass, object_mesh)
        # if quality < RFC_bound:
        #     print("Failed RFC {}".format(quality))
        #     continue
        
        # #Ferrari   
        quality, num_trails = compute_ferrari_canny(vertices, normals, num_facets, mu, gamma, object_mass)
        if  quality< ferrari_th:
            print("Failed Ferrari: {}".format(quality))
            
            continue
        
        # if not (compute_force_closure(vertices, normals, num_facets, mu, gamma, object_mass) \
        #     and compute_gravity_resistance(vertices, normals, num_facets, mu, gamma, object_mass)):
        #     continue
        
        print("Good {}".format(quality))
        return vertices, get_gripper_pose(vertices, object_mesh), quality

    return None, None, None

def custom_grasp_planner_2(object_mesh):
    """
    Write your own grasp planning algorithm! You will take as input the mesh
    of an object, and a pair of contact points from the surface of the mesh.
    You should return a 4x4 ridig transform specifying the desired pose of the
    end-effector (the gripper tip) that you would like the gripper to be at
    before closing in order to execute your grasp.

    You should be prepared to handle malformed grasps. Return None if no
    good grasp is possible with the provided pair of contact points.
    Keep in mind the constraints of the gripper (length, minimum and maximum
    distance between fingers, etc) when picking a good pose, and also keep in
    mind limitations of the robot (can the robot approach a grasp from the inside
    of the mesh? How about from below?). You should also make sure that the robot
    can successfully make contact with the given contact points without colliding
    with the mesh.

    The trimesh package has several useful functions that allow you to check for
    collisions between meshes and rays, between meshes and other meshes, etc, which
    you may want to use to make sure your grasp is not in collision with the mesh.

    Take a look at the functions find_intersections, find_grasp_vertices, 
    normal_at_point in utils.py for examples of how you might use these trimesh 
    utilities. Be wary of using these functions directly. While they will probably 
    work, they don't do excessive edge-case handling. You should spend some time
    reading the documentation of these packages to find other useful utilities.
    You may also find the collision, proximity, and intersections modules of trimesh
    useful.

    Feel free to change the signature of this function to add more arguments
    if you believe they will be useful to your planner.

    Parameters
    ----------
    object_mesh (trimesh.base.Trimesh): A triangular mesh of the object, as loaded in with trimesh.
    vertices (2x3 np.ndarray): obj mesh vertices on which the fingers will be placed

    Returns
    -------
    (4x4 np.ndarray): The rigid transform for the desired pose of the gripper, in the object's reference frame.
    """

    # constants -- you may or may not want to use these variables below
    num_facets = 64
    mu = 0.5
    gamma = 0.1
    object_mass = 0.25
    g = 9.8
    desired_wrench = np.array([0, 0, g * object_mass, 0, 0, 0]).T

    trials = 1000
    delta = 0.04
    RFC_bound = 0.94

    quality = 0

    min_corner, max_corner = object_mesh.bounding_box.bounds
    
    
    # YOUR CODE HERE
    print("Searching for points that meet base criteria")
    while quality < RFC_bound:  
        #Randomly sample 2 points from mesh  
        xs = np.random.uniform(min_corner[0],max_corner[0],2)
        ys = np.random.uniform(min_corner[1],max_corner[1],2)
        zs = np.random.uniform(min_corner[2],max_corner[2],2)
        vertices = np.array([[xs[0], ys[0], zs[0]],[xs[1], ys[1], zs[1]]])
        
        p1 = vertices[0]
        p2 = vertices[1]

        locations = utils.find_grasp_vertices(object_mesh, p1, p2)[0]

        if len(locations) > 1:
            p1 = locations[0]
            p2 = locations[1]
        else:
            print("locations failed?? ", locations)

        vertices = np.array([p1,p2]).reshape(2,3)  # Reshape to 2x3 array for future manipulations



        #Make Sure points on within grab range
        grip_dist = np.linalg.norm(p1 - p2)
        
        if not (MIN_GRIPPER_DIST <= grip_dist <= MAX_GRIPPER_DIST):
            print("ray outside of gripper bounds")
            continue

        #Checking to make sure points are furthest out points in gripper width
        gripper_pose = get_gripper_pose(vertices, object_mesh) #Returns 4x4 Transformation matrix

        quat = tf.transformations.quaternion_from_matrix(gripper_pose)

        t = TransformStamped()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        v = Vector3Stamped()
        v.vector.z = -.10
        v_t = tf2_geometry_msgs.do_transform_vector3(v, t)
        ray_dir = np.array([v_t.vector.x,v_t.vector.y,v_t.vector.z])

        p1_locs_in, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p1],
            ray_directions=[ray_dir],
            multiple_hits=True
        )
        
        p1_locs_out, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p1],
            ray_directions=[-ray_dir],
            multiple_hits=True
        )
        
        if len(p1_locs_in) > 0 or len(p1_locs_in) > 0:
            print("p1 width failure")
            continue
        
        p2_locs_in, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p2],
            ray_directions=[-ray_dir],
            multiple_hits=True
        )
        
        p2_locs_out, _, _ = object_mesh.ray.intersects_location(
            ray_origins=[p2],
            ray_directions=[ray_dir],
            multiple_hits=True
        )
        
        if len(p2_locs_in) > 0 or len(p2_locs_out) > 0:
            print("p2 width failure")
            continue

        #Computing normal
        n1 = utils.normal_at_point(object_mesh, p1)
        n2 = utils.normal_at_point(object_mesh, p2)
        
        normals = np.array([n1, n2])
        
        ##TODO: ERRORING AFTER THIS STEP###################################################
        print("Point pass base criteria")

        if not compute_force_closure(vertices, normals, num_facets, mu, gamma, object_mass):
            print("grasp not force closure")
            continue
        
        #RFC
        quality, num_trails = compute_robust_force_closure(vertices, normals, num_facets, mu, gamma, object_mass, object_mesh)
        if quality < RFC_bound:
            print("Failed RFC {}".format(quality))
            print("Number trails: {}".format(num_trails))
            continue
        
        print("Good RFC {}".format(quality))
        print("Number trails: {}".format(num_trails))
        print("---------------------")

    return vertices, gripper_pose

def get_gripper_pose(vertices, object_mesh): # you may or may not need this method 
    """
    Creates a 3D Rotation Matrix at the origin such that the y axis is the same
    as the direction specified.  There are infinitely many of such matrices,
    but we choose the one where the z axis is as vertical as possible.
    z -> y
    x -> x
    y -> z

    Parameters
    ----------
    object_mesh (tnp.ndarrayrimesh.base.Trimesh): A triangular mesh of the object, as loaded in with trimesh.
    vertices (2x3 ): obj mesh vertices on which the fingers will be placed

    Returns
    -------
    4x4 :obj:`numpy.ndarray`
    """
    origin = np.mean(vertices, axis=0)
    direction = vertices[0] - vertices[1]

    up = np.array([0, 0, 1])
    y = utils.normalize(direction)
    x = utils.normalize(np.cross(up, y))
    z = np.cross(x, y)

    gripper_top = origin + GRIPPER_LENGTH * z
    gripper_double = origin + 2 * GRIPPER_LENGTH * z
    if len(utils.find_intersections(object_mesh, gripper_top, gripper_double)[0]) > 0:
        z = utils.normalize(np.cross(up, y))
        x = np.cross(y, x)
    result = np.eye(4)
    result[0:3,0] = x
    result[0:3,1] = y
    result[0:3,2] = z
    result[0:3,3] = origin + np.array([0,0,0.07]) #Pawn 0.065, Nozzle 0.07
    return result


def visualize_grasp(mesh, vertices, pose):
    """Visualizes a grasp on an object. Object specified by a mesh, as
    loaded by trimesh. vertices is a pair of (x, y, z) contact points.
    pose is the pose of the gripper tip.

    Parameters
    ----------
    mesh (trimesh.base.Trimesh): mesh of the object
    vertices (np.ndarray): 2x3 matrix, coordinates of the 2 contact points
    pose (np.ndarray): 4x4 homogenous transform matrix
    """
    # vertices = np.array([[0.5,0,-.03],[-0.5,0,-.03]])
    
    p1, p2 = vertices
   
    center = (p1 + p2) / 2
    approach = pose[:3, 2]
    tail = center - GRIPPER_LENGTH * approach

    contact_points = []
    for v in vertices:
        contact_points.append(vedo.Point(pos=v, r=30))

    vec = (p1 - p2) / np.linalg.norm(p1 - p2)
    line = vedo.shapes.Tube([center + 0.5 * MAX_GRIPPER_DIST * vec,
                                   center - 0.5 * MAX_GRIPPER_DIST * vec], r=0.001, c='g')
    approach = vedo.shapes.Tube([center, tail], r=0.001, c='g')
    vedo.show([mesh, line, approach] + contact_points, new=True)


def randomly_sample_from_mesh(mesh, n):
    """Example of sampling points from the surface of a mesh.
    Returns n (x, y, z) points sampled from the surface of the input mesh
    uniformly at random. Also returns the corresponding surface normals.

    Parameters
    ----------
    mesh (trimesh.base.Trimesh): mesh of the object
    n (int): number of desired sample points

    Returns
    -------
    vertices (np.ndarray): nx3 matrix, coordinates of the n surface points
    normals (np.ndarray): nx3 matrix, normals of the n surface points
    """
    vertices, face_ind = trimesh.sample.sample_surface(mesh, n) # you may want to check out the trimesh mehtods here:)
    normals = mesh.face_normals[face_ind]
    return vertices, normals


PKG_DIR = "src/projects-3-4/proj4_pkg/src"

def load_grasp_data(object_name):
    """Loads grasp data from the provided NPZ files. It returns three arrays:

    Parameters
    ----------
    object_name (String): type of object

    Returns
    -------
    vertices (np.ndarray): nx3 matrix, coordinates of the n surface points
    normals (np.ndarray): nx3 matrix, normals of the n surface points

    grasp_vertices (np.ndarray): 5x2x3 matrix. For each of the 5 grasps,
            this stores a pair of (x, y, z) locations that are the contact points
            of the grasp.
    normals (np.ndarray): 5x2x3 matrix. For each grasp, stores the normal
            vector to the mesh at the two contact points. Remember that the normal
            vectors to a closed mesh always point OUTWARD from the mesh.
    tip_poses (np.ndarray): 5x4x4 matrix. For each of the five grasps, this
            stores the 4x4 rigid transform of the reference frame of the gripper
            tip before the gripper is closed in order to grasp the object.
    results (np.ndarray): 5x5 matrix. Stores the result of five trials for
            each of the five grasps. Entry (i, j) is a 1 if the jth trial of the
            ith grasp was successful, 0 otherwise.
    """    
    data = np.load('{}/grasp_data/{}.npz'.format(PKG_DIR, object_name))
    return data['grasp_vertices'], data['normals'], data['tip_poses'], data['results']


def load_mesh(object_name):
    mesh = trimesh.load_mesh('{}/objects/{}.obj'.format(PKG_DIR, object_name))
    mesh.fix_normals()
    return mesh

# mesh = # load_mesh('pawn') # you can set this here, or not

def main():
    """ Example for interacting with the codebase. Loads data and
    visualizes each grasp against the corresponding mesh of the given
    object.
    """
    obj = 'nozzle' # should be 'pawn' or 'nozzle'.
    vertices, normals, poses, results = load_grasp_data(obj)
    mesh = load_mesh(obj)
    for v, p in zip(vertices, poses):
        visualize_grasp(mesh, v, p)

    # you may or may not find the code below helpful:) i was supposed to delete this from the starter code but i didn't so enjoy
    v_and_p = [custom_grasp_planner(mesh, v) for v in vertices]
    for v, p in v_and_p:
        if not p is None:
            visualize_grasp(mesh, v, p)
        else:
            print("None")
    vert1, norm1 = randomly_sample_from_mesh(mesh, 3)
    while len(vert1) < 3:
        vert1, norm1 = randomly_sample_from_mesh(mesh, 3)
    vert2 = np.array([utils.find_intersections(mesh, vert1[i], vert1[i] - 5 * norm1[i])[0][-1] for i in range(len(vert1))]) #randomly_sample_from_mesh(mesh, 3)
    vertices = list(zip(vert1, vert2))


if __name__ == '__main__':
    main()

