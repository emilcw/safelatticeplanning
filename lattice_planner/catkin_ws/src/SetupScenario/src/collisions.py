#!/usr/bin/env python
###  Script to calculate number of collisions and length of every collision
###  Will currently print out the metrics when 'true' is written to topic /write_out
###  rostopic pub /write_log std_msgs/Bool "data: true"
###

import rospy
from std_msgs.msg import Bool
from setup_scenario.msg import Obstacles
from geometry_msgs.msg import PointStamped
import os
import numpy as np
import csv

DRONE_MODEL_NAME = 'dji0'
collisions = {}
collision_durations = []
collision_models = []
number_of_collisions = 0
number_of_dynamic_collisions = 0
number_of_static_collisions = 0
start_time = None
start_times = {}
intervals = []
dji_position = None

dji_safety_radius = rospy.get_param('/dji0/collision_detection_node/dji_safety_radius_', 1.0)

# Log files
homeDir = os.path.expanduser("~")
logfile_directory = os.path.join(homeDir, 'data',)
logfile_path = os.path.join(logfile_directory, 'collision.csv')
intervals_path = os.path.join(logfile_directory, 'intervals.csv')


# Classes for collision checking
class Sphere:
    def __init__(self, center, radius):
        self.center = center #np.array(3)
        self.radius = radius #float


class Plane:
    def __init__(self, position, direction):
        self.position = position   #np.array(3)
        self.direction = direction #np.array(3), normalized


class Box:
    def __init__(self, center, front, back, top, bottom, left, right):
        
        self.center = center #np.array(3)

        #Planes
        self.front = front
        self.back = back
        self.top = top
        self.bottom = bottom
        self. left = left
        self.right = right


# --------------- Helper functions for collision checking sphere - box ------------
def plane_distance(plane, point):
    return np.dot((point - plane.position), plane.direction)


def plane_distance_abs(plane, point):
    return abs(np.dot((point - plane.position), plane.direction))


def sphere_inside_plane(sphere, plane):
    return -plane_distance(plane, sphere.center) > sphere.radius


def sphere_outside_plane(sphere, plane):
    return plane_distance(plane, sphere.center) > sphere.radius


def sphere_intersects_plane(sphere, plane):
    return abs(plane_distance(plane, sphere.center)) <= sphere.radius


def sphere_inside_box(sphere, box):
    if(not sphere_inside_plane(sphere, box.front)): return False
    if(not sphere_inside_plane(sphere, box.back)): return False
    if(not sphere_inside_plane(sphere, box.top)): return False
    if(not sphere_inside_plane(sphere, box.bottom)): return False
    if(not sphere_inside_plane(sphere, box.left)): return False
    if(not sphere_inside_plane(sphere, box.right)): return False

    return True


def get_box(box_point, box_radius3d):
    
    # Center and radius
    x = box_point[0]
    y = box_point[1]
    z = box_point[2]
    rx = box_radius3d[0]
    ry = box_radius3d[1]
    rz = box_radius3d[2]

    # Edges in box
    a = np.array([x - rx, y - ry, z + rz])
    b = np.array([x + rx, y - ry, z + rz])
    c = np.array([x - rx, y - ry, z - rz])
    d = np.array([x + rx, y - ry, z - rz])
    e = np.array([x - rx, y + ry, z + rz])
    f = np.array([x + rx, y + ry, z + rz])
    g = np.array([x - rx, y + ry, z - rz])
    h = np.array([x + rx, y + ry, z - rz])
    points = [a, b, c, d, e, f, g, h]
    points_names = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']

    # Mid-position of plane
    front_pos = np.array([x, y - ry, z])
    back_pos = np.array([x, y + ry, z])
    top_pos = np.array([x, y, z + rz])
    bottom_pos = np.array([x, y, z - rz])
    left_pos = np.array([x - rx, y, z])
    right_pos = np.array([x + rx, y, z])

    # Collection of edges form each plan, normal pointing out
    # Ordering is important here since we want the normal to point out
    front = (a, b, c, d)  # normal pointing out
    back = (g, h, e, f)   # normal pointing out
    top = (b, a, f, e)    # normal pointing out 
    bottom = (c, d, g, h) # normal pointing out
    left = (a, c, e, g)   # normal pointing out
    right = (f, h, b, d)  # normal pointing out 

    edges = [front, back, top, bottom, left, right]
    names = ['front', 'back', 'top', 'bottom', 'left', 'right']
    plane_positions = [front_pos, back_pos, top_pos, bottom_pos, left_pos, right_pos]
    planes = {}

    for plane, name, plane_position in zip(edges, names, plane_positions):
        
        # 3 points in plane is enough to describe it
        p1 = plane[0]
        p2 = plane[1]
        p3 = plane[2]

        # Vectors that are in the plane (and orthogonal)
        v1 = p3 - p1
        v2 = p2 - p1

        # Cross product is a vector normal to plane
        cp = np.cross(v1, v2)
        
        # plane can be described on form ax + bx + cz = d
        a, b, c, = cp
        d = np.dot(cp, p3)

        normal = np.array([a, b, c])
        norm = np.linalg.norm(normal)
        if norm != 0:
            normal = normal / norm
        
        p = Plane(p1, normal)
        planes[name] = p

    b = Box(box_point, 
            planes['front'], 
            planes['back'], 
            planes['top'], 
            planes['bottom'], 
            planes['left'], 
            planes['right'])

    return b, points, points_names, planes


def sphere_intersects_plane_point(sphere, plane):
    distance = plane_distance(plane, sphere.center)
    projection = plane.direction * distance
    point = sphere.center - projection
    radius = np.sqrt(max( ( (sphere.radius * sphere.radius) - (distance * distance)) , 0))   
    return abs(distance) <= sphere.radius, point, radius, projection, distance


def sphere_intersects_box(sphere, box):

    intersect, point, radius, projection, distance = sphere_intersects_plane_point(sphere, box.top)
    if intersect:
        if (plane_distance(box.left, point)  <= radius and \
            plane_distance(box.right, point) <= radius and \
            plane_distance(box.front, point) <= radius and \
            plane_distance(box.back, point)  <= radius):
            return True, point, projection, distance

    intersect, point, radius, projection, distance = sphere_intersects_plane_point(sphere, box.bottom)
    if intersect:
        if (plane_distance(box.left, point)  <= radius and \
            plane_distance(box.right, point) <= radius and \
            plane_distance(box.front, point) <= radius and \
            plane_distance(box.back, point)  <= radius):
            return True, point, projection, distance

    intersect, point, radius, projection, distance = sphere_intersects_plane_point(sphere, box.left)
    if intersect:
        if (plane_distance(box.top, point)    <= radius  and \
            plane_distance(box.bottom, point) <= radius  and \
            plane_distance(box.front, point)  <= radius  and \
            plane_distance(box.back, point)   <= radius):
            return True, point, projection, distance
        
    intersect, point, radius, projection, distance = sphere_intersects_plane_point(sphere, box.right)    
    if intersect:
        if (plane_distance(box.top, point)    <= radius and \
            plane_distance(box.bottom, point) <= radius and \
            plane_distance(box.front, point)  <= radius and \
            plane_distance(box.back, point)   <= radius):
            return True, point, projection, distance
        
    intersect, point, radius, projection, distance = sphere_intersects_plane_point(sphere, box.front)
    if intersect:
        if (plane_distance(box.top, point)    <= radius and \
            plane_distance(box.bottom, point) <= radius and \
            plane_distance(box.left, point)   <= radius and \
            plane_distance(box.right, point)  <= radius):
            return True, point, projection, distance
        
    intersect, point, radius, projection, distance = sphere_intersects_plane_point(sphere, box.back)
    if intersect:
        if (plane_distance(box.top, point)    <= radius and \
            plane_distance(box.bottom, point) <= radius and \
            plane_distance(box.left, point)   <= radius and \
            plane_distance(box.right, point)  <= radius):
            return True, point, projection, distance
            
    return False, np.array([0,0,0]), np.array([0,0,0]), 0.0
    

# Aligned bounding boxes (AABB)
def collision_intersect(drone, human):
  return (
    #drone.minX <= human.maxX
    drone[0][0] <= human[0][1] and
    # drone.maxX >= human.minX 
    drone[0][1] >= human[0][0] and
    # drone.minY <= human.maxY 
    drone[1][0] <= human[1][1] and
    # drone.maxY >= human.minY 
    drone[1][1] >= human[1][0] and
   # drone.minZ <= human.maxZ 
    drone[2][0] <= human[2][1] and
    # drone.maxZ >= human.minZ
    drone[2][1] >= human[2][0]
  )

def collision_intersect_dynamic(dji_safety_radius, obstacle):
    dji_point = np.array([dji_position.x, dji_position.y, dji_position.z])
    obstacle_point = np.array([obstacle.position.x, obstacle.position.y, obstacle.position.z])

    distance = np.linalg.norm(dji_point - obstacle_point)
    hitbox_distance = distance - (dji_safety_radius + obstacle.radius)

    return hitbox_distance < 0


def collision_intersect_static(dji_safety_radius, obstacle):
    # Inspired by the work in 
    # https://theorangeduck.com/page/correct-box-sphere-intersection

    dji_point = np.array([dji_position.x, dji_position.y, dji_position.z])
    sphere = Sphere(dji_point, dji_safety_radius)
    
    box_point = np.array([obstacle.position.x, obstacle.position.y, obstacle.position.z])
    box_radius3d = np.array([obstacle.radius3D.x, obstacle.radius3D.y, obstacle.radius3D.z])

    box, points, points_names, planes = get_box(box_point, box_radius3d)

    inside = sphere_inside_box(sphere, box)

    intersect, point, projection, distance = sphere_intersects_box(sphere, box)

    outside = not(inside or intersect)

    #print("INSIDE: ", inside)
    #print("INTERSECT: ", intersect)
    #print("OUTSIDE: ", outside)

    return inside or intersect


def is_static(obstacle):
    return obstacle.state_space_model_velocity.linear.x == 0 and \
           obstacle.state_space_model_velocity.linear.y == 0 and \
           obstacle.state_space_model_velocity.linear.z == 0


def check_collision(msg):
    global number_of_collisions
    global number_of_dynamic_collisions
    global number_of_static_collisions
    global collisions
    global start_times
    global start_time
    
    if start_time is None:
        rospy.loginfo("Have not received time yet...")
        return
    
    if dji_position is None:
        rospy.loginfo("Have not received pose yet...")
        return

    # Loop over other models
    for obstacle in msg.obstacles:
        obstacle_name = obstacle.id
        isCollision = False

        if obstacle.is_advanced:
            # Advanced dynamic obstacle
            # Use radius from position
            isCollision = collision_intersect_dynamic(dji_safety_radius, obstacle)
        
        elif is_static(obstacle):
            # Static obstacle
            # Use radius3D from position
            isCollision = collision_intersect_static(dji_safety_radius, obstacle)
        
        else:
            # Simple dynamic obstacle
            # Use radius from position
            isCollision = collision_intersect_dynamic(dji_safety_radius, obstacle)

        if isCollision:
             # Check if we have already started a collision counter
            if obstacle_name not in collisions:
                rospy.loginfo("Adding model name {0}".format(obstacle_name))
                collisions[obstacle_name] = rospy.Time.now()
                number_of_collisions += 1
                if is_static(obstacle):
                    number_of_static_collisions += 1
                else:
                    number_of_dynamic_collisions += 1

                #Save the start time for the current collision
                start_times[obstacle_name] = rospy.Time.now() - start_time

        else:
            # No collision between drone and some model
            if obstacle_name in collisions:
                collision_duration = rospy.Time.now() - collisions[obstacle_name]
                collision_durations.append(collision_duration.to_sec())
                collision_models.append((obstacle_name, collision_duration.to_sec()))
                
                start_time_model = start_times[obstacle_name]
                end_time_model = rospy.Time.now() - start_time
                intervals.append((start_time_model.to_sec(), end_time_model.to_sec()))
                
                rospy.loginfo("Ending collision for model name {0}".format(obstacle_name))
                del collisions[obstacle_name]
                del start_times[obstacle_name]
            
    # Add a sleep to slow down the subscriber
    rate.sleep()

def write_log_callback(msg):
    #rospy.loginfo("Writing...")
    if msg.data:
        write_log(number_of_collisions, collision_durations, collision_models, number_of_dynamic_collisions, number_of_static_collisions)

def clock_callback(msg):
    if msg.data:
        rospy.loginfo("Starting to monitor collisions...")
        global start_time
        start_time = rospy.Time.now()

def pose_callback(msg):
    global dji_position
    dji_position = msg.point

def write_log(number_of_collisions, collision_durations, collision_models, dynamic_collisions, static_collisions):

    if not os.path.exists(logfile_directory):
        os.makedirs(logfile_directory)

    with open(logfile_path, 'w') as logfile:
        writer = csv.writer(logfile)
        writer.writerow(['Total duration', 'nr_of_collisions', 'dynamic_collisions', 'static_collisions'])
        writer.writerow([np.round(np.sum(collision_durations), 3), number_of_collisions, dynamic_collisions, static_collisions])

    with open(intervals_path, 'w') as intervalfile:
        writer = csv.writer(intervalfile)
        writer.writerow(['start-time', 'duration', 'end-time'])
        for interval in intervals:
            writer.writerow([interval[0], np.round(interval[1] - interval[0], 3), interval[1]])

    #rospy.loginfo("Number of collisions: {}".format(number_of_collisions))
    #rospy.loginfo("Number of dynamic_collisions: {}".format(dynamic_collisions))
    #rospy.loginfo("Number of static_collisions: {}".format(static_collisions))

    #rospy.loginfo("Collisions durations: ")
    #rospy.loginfo(" ".join(str(duration) for duration in collision_durations))
    #rospy.loginfo("\n")
    #rospy.loginfo(" ".join(str(col) for col in collision_models))

if __name__ == '__main__':
    rospy.init_node('collision_detection')
    rate = rospy.Rate(15)  # Create an instance of rospy.Rate()
    rospy.Subscriber('/dji0/obstacles', Obstacles, check_collision,  queue_size=1)
    rospy.Subscriber('/dji0/dji_sdk/local_position', PointStamped, pose_callback)
    rospy.Subscriber('/write_log', Bool, write_log_callback)
    rospy.Subscriber('/clock_start', Bool, clock_callback)
    while not rospy.is_shutdown():  # Loop until the node is shut down
        rate.sleep()  # Use rospy.Rate to slow down the subscriber
