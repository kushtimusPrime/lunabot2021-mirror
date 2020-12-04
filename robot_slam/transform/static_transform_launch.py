#!/usr/bin/python3
import json

LAUNCH_FILENAME = 'static_broadcaster.launch'
STATIC_FILENAME = 'tf_file_static.json'
HEADER = '<launch>'
FOOTER = '</launch>'

def node_pkg(name, args):
    return '<node pkg="tf2_ros" type="static_transform_publisher" name="' + name + '" args="' + args + '" />'

if __name__ == '__main__':
    launch_file = open(LAUNCH_FILENAME, 'w')
    launch_file.write(HEADER+'\n')

    with open(STATIC_FILENAME) as f:
        transforms = json.load(f)['transforms']
        
    for transform in transforms:
        frame_id = transform['frame_id']
        child_frame_id = transform['child_id']
        qx = str(transform['rot_x'])
        qy = str(transform['rot_y'])
        qz = str(transform['rot_z'])
        qw = str(transform['rot_w'])
        x = str(transform['trans_x'])
        y = str(transform['trans_y'])
        z = str(transform['trans_z'])
        args = x + ' ' + y + ' ' + z + ' ' + qx + ' ' + qy + ' ' + qz + ' ' + qw + ' ' + frame_id + ' ' + child_frame_id
        launch_file.write('\t'+node_pkg(frame_id + '_' + child_frame_id, args)+'\n')

    launch_file.write(FOOTER)
    launch_file.close()
