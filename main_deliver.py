"""
This module implements a control system for an autonomous vehicle, integrating different components of perception, control, and decision-making.

Imported Modules:
- Perception.camera: manages the capture of images from the camera.
- Perception.line_detection: performs line detection on captured images.
- Control.stanley_controller: implements the Stanley controller for navigation.
- Control.pid_control: provides a PID controller for motion adjustments.
- arduino_interface: interface for communication with the Arduino.
- Decision.state_machine: manages the states of the system.
- Control.high_level_controller: coordinates high-level actions of the vehicle.
- Decision.mission_planner: plans the mission and generates the vehicle's trajectory.
- Perception.corner_detection: detects corners in the images.
- Perception.obstacle_detection: identifies obstacles in the vehicle's path.
- multiprocessing: utilizes multiple processes to perform tasks simultaneously.

Main Functions:
- run_obstacle_detection(detected_obstacle_queue, obstacle_queue):
    Function that executes obstacle detection. It retrieves information about detected obstacles from an input queue and places the results in an output queue.

- run_line_detection(line_image_queue, line_queue):
    Function that executes line detection on images received from a queue. The detection result is placed in a queue for further processing.

- run_corner_detection(corner_image_queue, corner_queue):
    Function that executes corner detection on images received from a queue. The results are placed in a queue for use in other parts of the system.

The entry point of the module is the `if __name__ == "__main__":` block, which initializes all components and manages the control logic of the autonomous vehicle.

The system operates in a continuous loop, where it checks the current state of the system and executes appropriate actions based on line detection, corner detection, and obstacle detection. The vehicle's state is managed by a state machine, which determines the next step to be executed.

The vehicle interacts with the environment through visual feature detection and obstacle recognition, using control algorithms to perform maneuvers and reach specific destinations.
"""


from Perception.camera import Camera
from Perception.line_detection import LineDetection
from Control.stanley_controller import StanleyController
from Control.pid_control import PID
from arduino_interface import ArduinoInterface
from Decission.state_machine import StateMachine
from Control.high_level_controller import HighLevelController
from Decission.mission_planner import Graph
from Perception.corner_detection import CornerDetection
from Perception.obstacle_detection import ObstacleDetection
from multiprocessing import Queue, Process
import time

def run_obstacle_detection(detected_obstacle_queue, obstacle_queue):
    while True:
        a = detected_obstacle_queue.get()
        obstacles = obstacle_detector.get_detected_obstacle()
        obstacle_queue.put(obstacles)
        # time.sleep(0.01)


def run_line_detection(line_image_queue, line_queue):
    # line_detector = LineDetection()
    while True:
        t0 = time.time()
        line = line_detector.get_line_detected(line_image_queue.get())
        # print(time.time() - t0)
        line_queue.put(line)
        # time.sleep(0.01)

def run_corner_detection(corner_image_queue, corner_queue):
    # corner_detector = CornerDetection()
    while True:
        corner = corner_detector.get_coner_detected(corner_image_queue.get())
        corner_queue.put(corner)
        # time.sleep(0.01)


if __name__=="__main__":
    arduino_interface = ArduinoInterface()
    arduino_interface.stop()
    camera = Camera()
    line_detector = LineDetection()
    corner_detector = CornerDetection()

    obstacle_detector = ObstacleDetection(arduino_interface)

    stanley_controller = StanleyController()
    pid = PID()

    state_machine = StateMachine()
    high_level_controller = HighLevelController(arduino_interface)
    origin = "A"
    dest = "C"
    mission_planner = Graph(5, 5, origin, dest)
    mission_planner.set_trajectory()
    next_action = 0
    full_stop = 0
    # manager = Manager()
    # obstacle_queue = manager.Value('i', True)
    # detected_obstacle_queue = Queue()
    # obstacle_queue = Queue()
    # obstacle_process = Process(target=run_obstacle_detection, args=[detected_obstacle_queue, obstacle_queue])
    # obstacle_process.start()

    line_image_queue = Queue()
    line_queue = Queue()
    line_process = Process(target=run_line_detection, args=(line_image_queue, line_queue))
    line_process.start()

    corner_image_queue = Queue()
    corner_queue = Queue()
    corner_process = Process(target=run_corner_detection, args=(corner_image_queue, corner_queue))
    corner_process.start()
    t_corner = time.time()
    n = 0
    obstacles = False


    while True:
        if state_machine.return_state() == 0:
            next_action = mission_planner.get_next_action()
            print(next_action)
            state_machine.STATE = 3

        elif state_machine.return_state() == 1:
            n += 1
            t0 = time.time()
            image = camera.get_image()
            # print(time.time()-t0)
            # t0 = time.time()
            # point_to_be_followed = line_detector.get_line_detected(image)
            # print(time.time()-t0)
            line_image_queue.put(image)
            corner_image_queue.put(image)

            point_to_be_followed = line_queue.get()
            corners = corner_queue.get()
            if corners:
                print("F")
            if time.time() - t_corner < 4:
                # print(t_corner)
                corners = False
            if corners:
                print(corners)
            if n == 3:
                obstacles = obstacle_detector.get_detected_obstacle()
                n = 0
            # corners = False
            # t0 = time.time()
            # obstacles = obstacle_detector.get_detected_obstacle()
            # obstacles = False
            # print(time.time()-t0)
            # time.sleep(0.01)
            high_level_controller.update_error(point_to_be_followed)
            
            high_level_controller.perform_action()
            state_machine.decide_state(corners, obstacles)

            
            # print(time.time()-t0)

        elif state_machine.return_state() == 2:
            high_level_controller.set_action(4)
            print("Obstacle Detected!")
            n = 0
            obstacles = False
            high_level_controller.perform_action()
            mission_planner.obstacle_found()
            state_machine.STATE = 1
            high_level_controller.set_action(0)


        elif state_machine.return_state() == 3:
            print("Started Action")
            # t_corner = time.time()
            print(t_corner)
            high_level_controller.set_action(next_action)
            high_level_controller.perform_action()
            if next_action == 1:
                state_machine.STATE = 4
                full_stop += 1
            else:
                state_machine.STATE = 1
                high_level_controller.set_action(0)
            print("Finished Action")
            t_corner = time.time()
        elif state_machine.return_state() == 4:
            if full_stop == 2:
                time.sleep(0.1)
            else:
                final_dir = mission_planner.current_direction
                mission_planner = Graph(5, 5, dest, origin)
                mission_planner.current_direction = final_dir
                mission_planner.set_trajectory()
                state_machine.STATE = 0







