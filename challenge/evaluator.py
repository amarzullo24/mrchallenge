import os
import time
import json
import rospy
import PyKDL
import numpy as np
from ambf_client import Client

def toKDLVector(point):
    return PyKDL.Vector(point.x, point.y, point.z)

def toKDLFrame(point, rpy):
    rot = PyKDL.Rotation.RPY(*rpy)
    pos = toKDLVector(point)
    return PyKDL.Frame(rot, pos)

class Evaluator():
    def __init__(self, team_name, _client, verbose=True):
        self.team_name = team_name
        self.client = _client

        self.psm = _client.get_obj_handle("/ambf/env/psm/toolpitchlink")
        self.ecm = _client.get_obj_handle("/ambf/env/ecm/toollink")
        self.camera = _client.get_obj_handle("/ambf/env/cameras/endo_camera")
    
        self.ecm_base = _client.get_obj_handle("/ambf/env/ecm/baselink")
        self.psm_base = _client.get_obj_handle("/ambf/env/psm/baselink")

        self.log_dict = {}
        self.log_dict['theta'] = []
        self.log_dict['camera'] = []

        if not os.path.exists(team_name):
            os.makedirs(team_name)

        self.out_path = os.path.join(team_name, 'log.json')
        self.verbose = verbose
        
    def log(self):
        psm_pos = self.psm.get_pos() # xyx position in World frame
        psm_rpy = self.psm.get_rpy() # Fixed RPY in World frame

        ecm_pos = self.ecm.get_pos() # xyx position in World frame
        ecm_rpy = self.ecm.get_rpy() # Fixed RPY in World frame

        cam_pos = self.camera.get_pos() # xyx position in ECM frame
        cam_rpy = self.camera.get_rpy() # Fixed RPY in ECM frame

        # convert to KDL
        psm_kdl = toKDLFrame(psm_pos, psm_rpy)
        ecm_kdl = toKDLFrame(ecm_pos, ecm_rpy)
        cam_kdl = toKDLFrame(cam_pos, cam_rpy)

        ecm_base_kdl = toKDLFrame(self.ecm_base.get_pos(), self.ecm_base.get_rpy())
        psm_base_kdl = toKDLFrame(self.psm_base.get_pos(), self.psm_base.get_rpy())
        
        # convert wrt global
        cam_wrt_glob = ecm_base_kdl * cam_kdl

        v1 = (ecm_kdl.p - psm_kdl.p)
        v2 = (ecm_kdl.p - cam_wrt_glob.p)

        res = PyKDL.dot(v1,v2) / (v1.Norm() * v2.Norm() + 1e-99)
        theta = np.arccos(res)

        # store values
        self.log_dict['theta'].append(theta)
        self.log_dict['camera'].append(str(cam_wrt_glob))

        if self.verbose:
            print('current angle: ' + str(theta))

    def write_all(self):
        if self.verbose:
            print('saving logs...')

        with open(self.out_path, 'w') as fp:
            json.dump(self.log_dict, fp)

        if self.verbose:
            print('done.')
            

def main():
    # Create a instance of the client
    _client = Client()

    # Connect the client which in turn creates callable objects from ROS topics
    # and initiates a shared pool of threads for bidrectional communication 
    _client.connect()

    evaluator = Evaluator('daje', _client)
    while not rospy.is_shutdown():

        evaluator.log()
        time.sleep(1)

    # commit all the logs to file
    evaluator.write_all()

    # Lastly to cleanup
    _client.clean_up()

if __name__ == "__main__":
    main()