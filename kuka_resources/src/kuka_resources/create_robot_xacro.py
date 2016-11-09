#! /usr/bin/env python

import sys
import xacro
import numpy as np

import xml.etree.ElementTree as ET
import xml.dom.minidom

def xacro_macro(doc, name, params):
    return ET.SubElement(doc, 'xacro:macro', {'name' : name, 'params' : params})

def xacro_include(doc, filename):
    return ET.SubElement(doc, 'xacro:include', {'filename' : filename})

def xacro_property(doc, name, value):
    return ET.SubElement(doc, 'xacro:property', {'name' : name, 'value' : value})

def xacro_origin(doc, xyz, rpy):
    return ET.SubElement(doc, 'origin', {'xyz' : xyz, 'rpy' : rpy})

def xacro_axis(doc, xyz):
    return ET.SubElement(doc, "axis", {'xyz' : xyz})

def xacro_limit(doc, effort, upper, lower, velocity):
    return ET.SubElement(doc, "limit", {"effort" : effort,
                                        "lower" : lower,
                                        "upper" : upper,
                                        "velocity" : velocity})

def kuka_link_macro(doc, link_name, color, origin=['0 0 0', '0 0 0']):
    link = ET.SubElement(doc, 'xacro:kuka_link', {'link_name' : link_name, 'color' : color})
    xacro_origin(link, origin[0], origin[1])
    return link

def kuka_joint_macro(doc, joint_name, parent_link, child_link, origin, axis, limit=None):
    joint = ET.SubElement(doc, 'xacro:kuka_joint', {'joint_name': joint_name,
                                                    'parent_link' : parent_link,
                                                    'child_link' : child_link})
    xacro_origin(joint, origin['xyz'], origin['rpy'])
    xacro_axis(joint, axis['xyz'])
    if limit is not None:
        xacro_limit(joint, limit['effort'], limit['upper'],
                    limit['lower'], limit['velocity'])

def add_revolute_joint(doc, joint_name, parent_link, child_link, origin,
                       axis, limit=None):
    joint = ET.SubElement(doc, 'joint', {'name' : "${prefix}" + joint_name,
                                         'type' :'revolute'})
    xacro_origin(joint, origin['xyz'], origin['rpy'])
    ET.SubElement(joint, 'parent', {'link' : "${prefix}" + parent_link})
    ET.SubElement(joint, 'child', {'link' : "${prefix}" + child_link})
    xacro_axis(joint, axis['xyz'])
    if limit is not None:
        xacro_limit(joint, limit['effort'], limit['upper'],
                    limit['lower'], limit['velocity'])

def add_fixed_joint(doc, joint_name, parent_link, child_link, origin):
    joint = ET.SubElement(doc, 'joint', {'name' : "${prefix}" + joint_name, 'type' :'fixed'})
    xacro_origin(joint, origin['xyz'], origin['rpy'])
    ET.SubElement(joint, 'parent', {'link' : "${prefix}" + parent_link})
    ET.SubElement(joint, 'child', {'link' : "${prefix}" + child_link})


def add_link(doc, link_name, color=None, origin=None, do_visual=False, do_collision=False):
    link = ET.SubElement(doc, 'link', {'name' : "${prefix}" + link_name})
    # Visual mesh
    if do_visual:
        visual = ET.SubElement(link, 'visual')
        if origin is not None:
            xacro_origin(visual, origin['xyz'], origin['rpy'])
        geometry = ET.SubElement(visual, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh',
                            {'filename' : "package://" + "${package_name}" + "/meshes/" \
                            + "${robot_type}" + "/visual/{link_name}.stl".format(link_name=link_name),
                            'scale' : "${mesh_scale}"})
        if color is not None:
            material = ET.SubElement(visual, 'material', {'name' : color})
            color = ET.SubElement(material, 'color', {'rgba' : kuka_colors[color]})
    # Collision mesh
    if do_collision:
        collision = ET.SubElement(link, 'collision')
        if origin is not None:
            xacro_origin(collision, origin['xyz'], origin['rpy'])
        geometry = ET.SubElement(collision, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh',
                            {'filename' : "package://" + "${package_name}" + "/meshes/" \
                            + "${robot_type}" + "/visual/{link_name}.stl".format(link_name=link_name),
                            'scale' : "${mesh_scale}"})

def add_motor(doc, robot_definition, parent_link_number, motor_number):
    robot = robot_definition
    if parent_link_number == 1:
        motor_origin = {'xyz' : '0 0 {}'.format(-robot.L01Z), 'rpy' : '0 0 0'}
    if parent_link_number == 3:
        motor_origin = {'xyz' : '{0} 0 {1}'.format(-robot.L12X, -(robot.L01Z+robot.L23Z) ), 'rpy' : '0 0 0'}
    add_link(doc,
             "link_{0}_motor{1}".format(parent_link_number, motor_number),
             'kuka_black', motor_origin, True, True)
    add_fixed_joint(doc,
                    "joint_{0}_motor{1}".format(parent_link_number, motor_number),
                    "link_{0}".format(parent_link_number),
                    'link_{0}_motor{1}'.format(parent_link_number, motor_number),
                    {'xyz' : '0 0 0', 'rpy' : '0 0 0'})

def kuka_motor_macro(doc, parent_link, motor_number, origin):
    motor = ET.SubElement(doc, 'xacro:kuka_motor',
                          {'parent_link_number' : parent_link_number,
                           'motor_number' : motor_number})
    xacro_origin(motor, origin['xyz'], origin['rpy'])
    return motor

def create_kuka_robot_xacro(robot_definition):
    robot = robot_definition

    doc = ET.Element('robot', {'xmlns:xacro' : 'http://www.ros.org/wiki/xacro'})

    xacro_property(doc, 'mesh_scale', robot.mesh_scale)
    xacro_property(doc, 'package_name', robot.package_name)
    xacro_property(doc, 'robot_type', robot.robot_type)

    robot_macro = xacro_macro(doc, 'kuka_{0}'.format(robot.robot_type), "prefix")

    joint_origins = [
        # Joint 1 (A1)
        {'xyz' : '{x} {y} {z}'.format(x=0, y=0, z=robot.L01Z), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        # Joint 2 (A2)
        {'xyz' : '{x} {y} {z}'.format(x=robot.L12X, y=0, z=0), 'rpy' : '{r} {p} {y}'.format(r=0, p=np.deg2rad(90), y=0)},
        # Joint 3 (A3)
        {'xyz' : '{x} {y} {z}'.format(x=0, y=0, z=robot.L23Z), 'rpy' : '{r} {p} {y}'.format(r=0, p=np.deg2rad(-90), y=0)},
        # Joint 4 (A4)
        {'xyz' : '{x} {y} {z}'.format(x=robot.L34X, y=0, z=robot.L34Z), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        # Joint 5 (A5)
        {'xyz' : '{x} {y} {z}'.format(x=0, y=0, z=0), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        # Joint 6 (A6)
        {'xyz' : '{x} {y} {z}'.format(x=robot.L56X, y=0, z=0), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)}]

    link_origins = [
        {'xyz' : '{x} {y} {z}'.format(x=0, y=0, z=0), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=0, y=0, z=-robot.L01Z), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=-robot.L12X, y=0, z=-robot.L01Z), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=-robot.L12X, y=0, z=-(robot.L01Z+robot.L23Z)), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=-(robot.L12X+robot.L34X), y=0, z=-(robot.L01Z+robot.L23Z+robot.L34Z)), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=-(robot.L12X+robot.L34X), y=0, z=-(robot.L01Z+robot.L23Z+robot.L34Z)), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=-(robot.L12X+robot.L34X+robot.L56X), y=0, z=-(robot.L01Z+robot.L23Z+robot.L34Z)), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)}]

    # 6 rotation axes for the 6 revolute joints
    joint_rotation_axes = [
        # Joint 1 (A1)
        {'xyz' : '0 0 -1'},
        # Joint 2 (A2)
        {'xyz' : '0 1 0'},
        # Joint 3 (A3)
        {'xyz' : '0 1 0'},
        # Joint 4 (A4)
        {'xyz' : '-1 0 0'},
        # Joint 5 (A5)
        {'xyz' : '0 1 0'},
        # Joint 6 (A6)
        {'xyz' : '-1 0 0'}]

    n = 6
    for i in range(n+1):
        # add links
        add_link(robot_macro, robot.link_names[i], robot.link_colors[i], link_origins[i], True, True)

    for i in range(n):
        # add joints
        add_revolute_joint(robot_macro,
                           joint_name = robot.joint_names[i],
                           parent_link = robot.link_names[i],
                           child_link = robot.link_names[i+1],
                           origin = joint_origins[i],
                           axis = joint_rotation_axes[i],
                           limit=robot.limits[i])
        # add motors
        if robot.motors[i] is not None:
            for motor_number in robot.motors[i]:
                add_motor(robot_macro, robot, i, motor_number)
    # add end effector frame
    add_link(robot_macro, robot.link_names[-1])
    add_fixed_joint(robot_macro, robot.joint_names[-1],
                    robot.link_names[-2], robot.link_names[-1], {'xyz' : '0 0 0', 'rpy' : '0 0 0'})

    doc_string = ET.tostring(doc)
    doc = xml.dom.minidom.parseString(doc_string)
    #xacro.eval_self_contained(doc)
    print(doc.toprettyxml(indent='  '))

    #return doc

kuka_colors = {'kuka_orange' : '1.0 0.5 0.0 1.0', 'kuka_black' : '0.05 0.05. 0.05 1.0'}

def get_joint_origins():
    joint_origins = [
        # Joint 1 (A1)
        {'xyz' : '{x} {y} {z}'.format(x=0, y=0, z=L01Z), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        # Joint 2 (A2)
        {'xyz' : '{x} {y} {z}'.format(x=L12X, y=0, z=0), 'rpy' : '{r} {p} {y}'.format(r=0, p=np.deg2rad(90), y=0)},
        # Joint 3 (A3)
        {'xyz' : '{x} {y} {z}'.format(x=0, y=0, z=L23Z), 'rpy' : '{r} {p} {y}'.format(r=0, p=np.deg2rad(-90), y=0)},
        # Joint 4 (A4)
        {'xyz' : '{x} {y} {z}'.format(x=L34X, y=0, z=L34Z), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        # Joint 5 (A5)
        {'xyz' : '{x} {y} {z}'.format(x=0, y=0, z=0), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        # Joint 6 (A6)
        {'xyz' : '{x} {y} {z}'.format(x=L56X, y=0, z=0), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)}]
    return joint_origins

def get_link_origins():
    link_origins = [
        {'xyz' : '{x} {y} {z}'.format(x=0, y=0, z=0), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=0, y=0, z=-L01Z), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=-L12X, y=0, z=-L01Z), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=-L12X, y=0, z=-(L01Z+L23Z)), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=-(L12X+L34X), y=0, z=-(L01Z+L23Z+L34Z)), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=-(L12X+L34X), y=0, z=-(L01Z+L23Z+L34Z)), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)},
        {'xyz' : '{x} {y} {z}'.format(x=-(L12X+L34X+L56X), y=0, z=-(L01Z+L23Z+L34Z)), 'rpy' : '{r} {p} {y}'.format(r=0, p=0, y=0)}]
    return link_origins

def get_joint_rotation_axes():
    # 6 rotation axes for the 6 revolute joints
    joint_rotation_axes = [
        # Joint 1 (A1)
        {'xyz' : '0 0 -1'},
        # Joint 2 (A2)
        {'xyz' : '0 1 0'},
        # Joint 3 (A3)
        {'xyz' : '0 1 0'},
        # Joint 4 (A4)
        {'xyz' : '-1 0 0'},
        # Joint 5 (A5)
        {'xyz' : '0 1 0'},
        # Joint 6 (A6)
        {'xyz' : '-1 0 0'}]
    return joint_rotation_axes


if __name__ == '__main__':

    pass
    #doc = create_kuka_robot_xacro()
    #tree = ET.ElementTree(doc)
    #tree.write('../urdf/kr5arc_macro.xacro')


    #import xml.dom.minidom
    #newdoc = open('../urdf/kr5arc_macro.xacro')
    #while(newdoc.closed):
        #pass
    #dom = xml.dom.minidom.parse(newdoc)
    #print(dom.toprettyxml())
    #newdoc.close()

