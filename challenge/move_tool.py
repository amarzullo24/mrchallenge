# Import the Client from ambf_comm package
from ambf_client import Client
import os
import rospy
import time
import argparse

parser = argparse.ArgumentParser(description='Tool Mover for the MS robotic challenge')
parser.add_argument('--traj-name', type=str, help='trajectory file name', required=True)
args = parser.parse_args()

def main():

    # Create a instance of the client
    _client = Client(client_name='traj_worker')

    # Connect the client which in turn creates callable objects from ROS topics
    # and initiates a shared pool of threads for bidrectional communication 
    _client.connect()

    # You can print the names of objects found
    print(_client.get_obj_names())

    topic_name = "/ambf/env/psm/toolpitchlink"
    psm = _client.get_obj_handle(topic_name)

    def get_point(traj_name):
        path = os.path.join("trajectories","{}.txt".format(traj_name))
        while(True):
            with open(path, "r") as fin:
                traj = fin.readlines()
            
            for line in traj:
                vals = line.rstrip()[1:-1].split(', ')
                yield (float(vals[0]), float(vals[1]), float(vals[2]))

    iterator = iter(get_point(args.traj_name))
    
    cur_pos = psm.get_pos() # xyx position in World frame
    cur_rot = psm.get_rot() # Quaternion in World frame
    cur_rpy = psm.get_rpy() # Fixed RPY in World frame
    
    while not rospy.is_shutdown():
    
        x,y,z = next(iterator)
        psm.set_pos(cur_pos.x + x, cur_pos.y + y, cur_pos.z + z)
        
        time.sleep(1)

    # Lastly to cleanup
    _client.clean_up()

if __name__ == "__main__":
    main()