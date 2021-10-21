import os
import time
import json
import rospy
import PyKDL
import argparse
import numpy as np
from evaluator import Evaluator
from ambf_client import Client

parser = argparse.ArgumentParser(description='AI for the MS robotic challenge')
parser.add_argument('--team-name', type=str, help='the name of the team', required=True)

class Brain:
    def __init__(self, team_name):
	""" init function. Used to initialize main parameters """

        # Create a instance of the client
        self._client = Client()

        # Connect the client which in turn creates callable objects from ROS topics
        # and initiates a shared pool of threads for bidrectional communication 
        self._client.connect()

        self.team_name = team_name

    def run(self):
	""" here is the main loop, where all the actions should be computed """
        evaluator = Evaluator(self.team_name, self._client)
        
	while not rospy.is_shutdown():
	    
	    # here the AI code

            evaluator.log()
            time.sleep(1)

        # commit all the logs to file
        evaluator.write_all()

        # Lastly to cleanup
        self._client.clean_up()

if __name__ == "__main__":
    args = parser.parse_args()
    print('Everything begins...')
    print('Loading AI for ' + args.team_name)

    brain = Brain(args.team_name)
    brain.run()
