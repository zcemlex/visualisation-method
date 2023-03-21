# -*- coding: utf-8 -*-
"""
Created on Thu Mar  9 12:28:24 2023

@author: Admin
"""

import os
import csv
import math
import time
from sksurgerynditracker.nditracker import NDITracker
import numpy as np
import open3d as o3d
import serial
import tkinter as tk

settings_aurora = { "tracker type": "aurora", "ports to use" : [2,1]}

tracker = NDITracker(settings_aurora)
tracker.use_quaternions = "true"
tracker.start_tracking()
#ser = serial.Serial("COM3", baudrate=9600, timeout=0)
#root = tk.Tk()

def reference_sensor():
    # read the data from the sensor
    ref = tracker.get_frame()[3][1]
    # parse the data 

    ref_pos = ref[0][4:7]
    
    return ref_pos

def location_sensor():
    # read the data from the sensor
    data = tracker.get_frame()[3][0]
    # parse the data into position and orientation
    qw = data[0][0]
    qx = data[0][1]
    qy = data[0][2]
    qz = data[0][3]
    pos = data[0][4:7]
    
    return qw, qx, qy, qz, pos

def init_length(ref_pos, pos):

    distance = math.sqrt((pos[0] - ref_pos[0])**2 + (pos[1] - ref_pos[1])**2 + (pos[2] - ref_pos[2])**2)

    return distance

def read_pressure(vis):
    sphere_mesh.paint_uniform_color([0,0,1])
    pressure_value = ser.readline().decode().strip()
    pressure_value=(pressure_value.replace('','0'))
    pressure_float=float(pressure_value)
   #print(pressure_value)
   #print(pressure_float)
    #root.after(100, read_pressure)
    if (pressure_float)>15.0:
        sphere_mesh.paint_uniform_color([1,0,0])
    #    print('change')
    vis.update_geometry(sphere_mesh)
    vis.update_renderer()
    
def read_location(vis):
    #function should read location and move arrow
    #compare with ref location and move in difference with that
    # read the data from the sensor
    ref = tracker.get_frame()[3][1]
    # parse the data 

    ref_pos = ref[0][4:7]
    data = tracker.get_frame()[3][0]
    # parse the data into position and orientation
    qw = data[0][0]
    qx = data[0][1]
    qy = data[0][2]
    qz = data[0][3]
    pos = data[0][4:7]
    # read data from the USB ports
    #p_value = ser.readline().decode().strip()
    #if not qw: #isinstance(qw,float):
    #R=o3d.geometry.get_rotation_matrix_from_xyz([roll,pitch,yaw])
    #arrow_mesh.rotate(R, center=[0,0,0])
    #vis.update_geometry(arrow_mesh)
    #vis.update_renderer()
    #print('error ',R,'qw= ',qw)
# convert quaternion to Euler angles

    roll_deg = math.atan2(2 * (qw*qx + qy*qz), 1 - 2 * (qx**2 + qy**2))
    pitch_deg = math.asin(2 * (qw*qy - qx*qz))
    yaw_deg = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy**2 + qz**2))
    #if roll_deg='nan':
    #roll_deg, pitch_deg, yaw_deg= 128.2834787,	8.733157282, -55.04359084
    
    roll, pitch, yaw= np.radians([roll_deg, pitch_deg, yaw_deg])
    #if roll='nan':
    #    roll,pitch,yaw
    R=o3d.geometry.get_rotation_matrix_from_xyz([roll,pitch,yaw])
    arrow_end=np.array(pos)
    arrow_mesh.translate(pos-[0,0,0],relative=False)
    arrow_mesh.rotate(R, center=pos)
    vis.update_geometry(arrow_mesh)
    vis.update_renderer()
    print('done ',R,'qw= ',qw)


#root.after(100, read_pressure)
#root.mainloop()
bbox=o3d.geometry.AxisAlignedBoundingBox([-1000,-1000,-1000],[1000,1000,1000])

#print(o3d.__version__)
#Load mesh, together with setting the flag for post-processing to True, so the texture and material will be loaded
mesh = o3d.io.read_triangle_mesh('Prostate Phantom Multimaterial 2.obj')

print(mesh)
print('Vertices:')
print(np.asarray(mesh.vertices))
print('Triangles:')
print(np.asarray(mesh.triangles))

# We can extract information from the mesh like faces, UVs and texture
mesh_faces = mesh.triangles
mesh_uvs = mesh.triangle_uvs
texture = mesh.textures

#finding mesh outline
line_set=o3d.geometry.LineSet.create_from_triangle_mesh(mesh)

#hull,_=mesh.compute_convex_hull()
lines=[]
for i in range(len(line_set.lines)):
    line=line_set.lines[i]
    if i==0:
        lines.append(line)
    elif line[0]!=lines[-1][1]:
        lines.append(line)
        
mesh.paint_uniform_color([1, 0.71, 0.76]) # set the mesh color to pink
line_set=o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(line_set.points),
    lines=o3d.utility.Vector2iVector(lines)
)
                                                            
line_set.paint_uniform_color([1, 0, 0])


# We create a visualizer object that will contain references to the created window, the 3D objects and will listen to callbacks for key presses, animations, etc.
#o3d.visualization.draw_geometries([mesh, line_set])
vis = o3d.visualization.Visualizer()

sphere_mesh=o3d.geometry.TriangleMesh.create_sphere(radius=5)
#sphere_mesh.compute_vertex_normals() #??
sphere_mesh.paint_uniform_color([0,0,0.5]) #set intitial colour
sphere_mesh.translate((80,0,0)) #set placement in top right corner
vertices= np.asarray(sphere_mesh.vertices)
centre=vertices.mean(axis=0)
print('centre is ',centre)

arrow_mesh= o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.6, cone_radius=1.5, cylinder_height=10, cone_height=5)
# Set arrow position and orientation
arrow_mesh.paint_uniform_color([0,0.3,0])
arrow_mesh.translate((0, 0, 0))

# New window, where we can set the name, the width and height, as well as the position on the screen
vis.create_window(window_name='Prostate Phantom', width=800, height=600)

#color_map=o3d.visualization.ColorMapJet()

# We call add_geometry to add a mesh or point cloud to the visualizer
vis.add_geometry(mesh)
vis.add_geometry(line_set)
vis.add_geometry(sphere_mesh)
vis.add_geometry(arrow_mesh)
vis.get_render_option().background_color=[0.9,0.9,0.9]

# We run the visualizer
try:
    while True:
        #vis.register_animation_callback(read_pressure)
        vis.register_animation_callback(read_location)
        vis.run()
        if vis.destroy_window():
            tracker.stop_tracking()
            break
except KeyboardInterrupt:
    print('exit')
#run the sensor reading?/ call the functions
# Once the visualizer is closed destroy the window and clean up
