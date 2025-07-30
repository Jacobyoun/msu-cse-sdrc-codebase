from subprocess import Popen, PIPE
from time import sleep
import os
import signal
import sys

class StartCar:
    """
    StartCar class that runs a command to run the roslaunch command that starts all of the nodes
    ---
    Attributes:
    process : Only uses one process
    start_command : list of strings that contains the command args to start the rosnodes
    ---
 	Member Functions:
 	__init__ (self) : Initialize the class
    start_process : Sleeps for 15 seconds than calls the start command
    """

    def __init__(self):
        """Initialize all of the class attributes"""
        self.process = None  # Single process instead of multiple

        self.start_command = [
            "gnome-terminal", "--", "bash", "-c",
            "echo 'Random123' | sudo -S bash -c \""
            "source /opt/ros/humble/setup.bash; "
            "source $HOME/ros2_ws/install/setup.bash; "
            "ros2 launch start_car start_car_launch.py; "
            "exec bash\""
        ]

    def start_process(self):
        """Start all nodes in a single terminal after 15 seconds when called"""
        # Sleep for 15 seconds so that the user has time to unplug the moust and plug in a sensor
        sleep(15)
        
        # Start the ndoes
        self.process = Popen(self.start_command, stdout=PIPE, stderr=PIPE, text=True, preexec_fn=os.setsid)

        # Capture and print output for debugging
        stdout, stderr = self.process.communicate()
        
        if stdout:
            print("STDOUT:\n", stdout)
        if stderr:
            print("STDERR:\n", stderr)


# Global instance
car_starter = StartCar()


def main():
    print("Starting ROS2 nodes in 15 seconds via launch file...\n")
    print("Unplug the mouse and plug in your final sensor now.")

    car_starter.start_process()

if __name__ == '__main__':
    main()