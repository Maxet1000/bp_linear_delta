from Xeryon import *

"""
The calculation of the inverse kinematics is taken from
R.L. Williams II, “The Delta Parallel Robot: Kinematics Solutions”,
Internet Publication, www.ohio.edu/people/williar4/html/pdf/DeltaKin.pdf
January 2016
"""

class DeltaRobot:

    """
    Initialises a DeltaRobot object with the following parameters:
    leg_length: the length of the legs of the robot
    d_actuators: the distance between the actuators, expressed as the length of a side of the equilateral triangle
                 between the actuators
    d_end_effector: a correction for the size of the end effector, expressed as the length of a side of the equilateral
                    triangle between the points in the middle of the joint connections to the end effector
    actuator_range: the range of motion of the actuator
    act1_inverted: expresses if the actuator's position is inverted
    act2_inverted: expresses if the actuator's position is inverted
    act3_inverted: expresses if the actuator's position is inverted
    """
    def __init__(self, leg_length, d_actuators, d_end_effector, actuator_range, act1_inverted, act2_inverted,
                 act3_inverted):
        self.leg_length = leg_length
        self.d_actuators = d_actuators
        self.d_end_effector = d_end_effector
        self.actuator_range = actuator_range
        self.act1_inverted = act1_inverted
        self.act2_inverted = act2_inverted
        self.act3_inverted = act3_inverted

        # Parameters used for inverse kinematics
        WB = math.sqrt(3) / 6 * d_actuators
        UB = math.sqrt(3) / 3 * d_actuators
        WP = math.sqrt(3) / 6 * d_end_effector
        UP = math.sqrt(3) / 3 * d_end_effector
        self.a = (d_actuators - d_end_effector) / 2
        self.b = WB - WP
        self.c = UP - UB

        # Calculate the working radius
        v = math.sqrt(3) / 3 * (d_actuators - d_end_effector)
        self.working_radius = (leg_length*math.sqrt(1 - (leg_length*math.sin(2*math.atan((2*leg_length*actuator_range + math.sqrt(4*leg_length**2*actuator_range**2 + 16*leg_length**2*v**2 - actuator_range**4 - 8*actuator_range**2*v**2 - 16*v**4))/(4*leg_length*v + actuator_range**2 + 4*v**2))) - actuator_range)**2/leg_length**2) - v)

    """
    Calculates the inverse kinematics of the entire deltaRobot.
    """
    def inverseKinematics(self, x, y, z):
        c1 = x * x + y * y + z * z + self.a * self.a + self.b * self.b + 2 * self.a * x + 2 * self.b * y - self.leg_length * self.leg_length
        c2 = x * x + y * y + z * z + self.a * self.a + self.b * self.b - 2 * self.a * x + 2 * self.b * y - self.leg_length * self.leg_length
        c3 = x * x + y * y + z * z + self.c * self.c + 2 * self.c * y - self.leg_length * self.leg_length
        if (z * z < c1) or (z * z < c2) or (z * z < c3) and (x * x + y * y < self.working_radius * self.working_radius):
            print("error in inverse position kinematics")
            return [0, 0, 0]
        else:
            l1 = -z + math.sqrt(z * z - c1)
            l2 = -z + math.sqrt(z * z - c2)
            l3 = -z + math.sqrt(z * z - c3)
            return [l1, l2, l3]

    """
    Calculates the inverse kinematics of for the first actuator, avoiding repeating calculations between threads.
    """
    def inverseKinematicsAct1(self, x, y, z):
        c1 = x * x + y * y + z * z + self.a * self.a + self.b * self.b + 2 * self.a * x + 2 * self.b * y - self.leg_length * self.leg_length
        l1 = -z + math.sqrt(z * z - c1)
        return l1

    """
    Calculates the inverse kinematics of for the second actuator, avoiding repeating calculations between threads.
    """
    def inverseKinematicsAct2(self, x, y, z):
        c2 = x * x + y * y + z * z + self.a * self.a + self.b * self.b - 2 * self.a * x + 2 * self.b * y - self.leg_length * self.leg_length
        l2 = -z + math.sqrt(z * z - c2)
        return l2

    """
    Calculates the inverse kinematics of for the third actuator, avoiding repeating calculations between threads.
    """
    def inverseKinematicsAct3(self, x, y, z):
        c3 = x * x + y * y + z * z + self.c * self.c + 2 * self.c * y - self.leg_length * self.leg_length
        l3 = -z + math.sqrt(z * z - c3)
        return l3

    """
    Makes a correction to get the actuator position, position 'zero' is seen as the middle of the Xeryon actuators.
    """
    def getActuatorPosition(self, inverse_kinematics_result, act_inverted: bool = True):
        extension = (-1) ** act_inverted * (inverse_kinematics_result - self.actuator_range / 2)
        return extension

    """
    Calculates the relative speed based on the given maximum speed, which would be the speed for moving over the full 
    length of the actuator.
    """
    def calculateRelativeSpeed(self, desiredPos, currentPos, maxSpeed):
        return abs(desiredPos - currentPos) / self.actuator_range * maxSpeed


def moveThroughPoints_act1_smooth(delta_robot: DeltaRobot, point_list, maxSpeed, barrier):
    for index, point in enumerate(point_list):
        start_time = time.time()
        inverse_kin_result = delta_robot.inverseKinematicsAct1(point[0], point[1], point[2])
        actuator_position = delta_robot.getActuatorPosition(inverse_kin_result, delta_robot.act1_inverted)
        currentPos = axis1.getEPOS()
        if index != 0:
            current_speed = delta_robot.calculateRelativeSpeed(actuator_position, currentPos, maxSpeed)
            axis1.setSpeed(current_speed)
            if currentPos - actuator_position > 0:
                axis1.startScan(-1)
            else:
                axis1.startScan(1)
            elapsed_time = time.time() - start_time
            time_to_move = abs(actuator_position - currentPos)/current_speed
            time_to_sleep = time_to_move - elapsed_time
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
        else:
            axis1.setDPOS(actuator_position)
        barrier.wait()

    axis1.setDPOS(axis1.getEPOS())


def moveThroughPoints_act2_smooth(delta_robot: DeltaRobot, point_list, maxSpeed, barrier, firstRound, lastRound):
    for index, point in enumerate(point_list):
        start_time = time.time()
        inverse_kin_result = delta_robot.inverseKinematicsAct2(point[0], point[1], point[2])
        actuator_position = delta_robot.getActuatorPosition(inverse_kin_result, delta_robot.act2_inverted)
        currentPos = axis2.getEPOS()
        if index != 0:
            current_speed = delta_robot.calculateRelativeSpeed(actuator_position, currentPos, maxSpeed)

            axis2.setSpeed(current_speed)
            if currentPos - actuator_position > 0:
                axis2.startScan(-1)
            else:
                axis2.startScan(1)

            elapsed_time = time.time() - start_time
            time_to_move = abs(actuator_position - currentPos)/current_speed
            time_to_sleep = time_to_move - elapsed_time
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
        else:
            axis2.setDPOS(actuator_position)
        barrier.wait()

    axis2.setDPOS(axis2.getEPOS())


def moveThroughPoints_act3_smooth(delta_robot: DeltaRobot, point_list, maxSpeed, barrier):
    for index, point in enumerate(point_list):
        start_time = time.time()
        inverse_kin_result = delta_robot.inverseKinematicsAct3(point[0], point[1], point[2])
        actuator_position = delta_robot.getActuatorPosition(inverse_kin_result, delta_robot.act3_inverted)
        currentPos = axis3.getEPOS()
        if index != 0:
            current_speed = delta_robot.calculateRelativeSpeed(actuator_position, currentPos, maxSpeed)

            axis3.setSpeed(current_speed)
            if currentPos - actuator_position > 0:
                axis3.startScan(-1)
            else:
                axis3.startScan(1)

            elapsed_time = time.time() - start_time
            time_to_move = abs(actuator_position - currentPos)/current_speed
            time_to_sleep = time_to_move - elapsed_time
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
        else:
            axis3.setDPOS(actuator_position)
        barrier.wait()

    axis3.setDPOS(axis3.getEPOS())


def moveThroughPoints_smooth(point_list, speed, maxSpeed, firstRound, lastRound):
    axis1.setSpeed(speed)
    axis2.setSpeed(speed)
    axis3.setSpeed(speed)

    barrier = threading.Barrier(3)
    thread1 = threading.Thread(target=moveThroughPoints_act1_smooth,
                               args=(delta_robot, point_list, maxSpeed, barrier, firstRound, lastRound))
    thread2 = threading.Thread(target=moveThroughPoints_act2_smooth,
                               args=(delta_robot, point_list, maxSpeed, barrier, firstRound, lastRound))
    thread3 = threading.Thread(target=moveThroughPoints_act3_smooth,
                               args=(delta_robot, point_list, maxSpeed, barrier, firstRound, lastRound))

    thread1.start()
    thread2.start()
    thread3.start()

    thread1.join()
    thread2.join()
    thread3.join()

def moveThroughPoints_act1(delta_robot: DeltaRobot, point_list, speed, barrier):
    axis1.setSpeed(speed)
    for point in point_list:
        inverse_kin_result = delta_robot.inverseKinematicsAct1(point[0], point[1], point[2])
        actuator_position = delta_robot.getActuatorPosition(inverse_kin_result, delta_robot.act1_inverted)
        axis1.setDPOS(actuator_position)
        barrier.wait()


def moveThroughPoints_act2(delta_robot: DeltaRobot, point_list, speed, barrier):
    axis2.setSpeed(speed)
    for point in point_list:
        inverse_kin_result = delta_robot.inverseKinematicsAct2(point[0], point[1], point[2])
        actuator_position = delta_robot.getActuatorPosition(inverse_kin_result, delta_robot.act2_inverted)
        axis2.setDPOS(actuator_position)
        barrier.wait()


def moveThroughPoints_act3(delta_robot: DeltaRobot, point_list, speed, barrier):
    axis3.setSpeed(speed)
    for point in point_list:
        inverse_kin_result = delta_robot.inverseKinematicsAct3(point[0], point[1], point[2])
        actuator_position = delta_robot.getActuatorPosition(inverse_kin_result, delta_robot.act3_inverted)
        axis3.setDPOS(actuator_position)
        barrier.wait()


def moveThroughPoints(point_list, speed):
    barrier = threading.Barrier(3)
    thread1 = threading.Thread(target=moveThroughPoints_act1, args=(delta_robot, point_list, speed, barrier))
    thread2 = threading.Thread(target=moveThroughPoints_act2, args=(delta_robot, point_list, speed, barrier))
    thread3 = threading.Thread(target=moveThroughPoints_act3, args=(delta_robot, point_list, speed, barrier))

    thread1.start()
    thread2.start()
    thread3.start()

    thread1.join()
    thread2.join()
    thread3.join()


delta_robot = DeltaRobot(70.0, 84.497542228927, 13.3842608598, 45.0, False, True, True)

controller1 = Xeryon("COM3", 115200)
axis1 = controller1.addAxis(Stage.XLS_1250, "A")
controller1.start()
axis1.findIndex()
axis1.setUnits(Units.mm)
axis1.setSpeed(20)

controller2 = Xeryon("COM7", 115200)
axis2 = controller2.addAxis(Stage.XLS_1250, "B")
controller2.start()
axis2.findIndex()
axis2.setUnits(Units.mm)
axis2.setSpeed(20)

controller3 = Xeryon("COM5", 115200)
axis3 = controller3.addAxis(Stage.XLS_1250, "C")
controller3.start()
axis3.findIndex()
axis3.setUnits(Units.mm)
axis3.setSpeed(20)


# move between the corners of a pentagon
point_list_pentagon = [[15.0, 0.0, 23.135538697791706], [4.635254915624212, 14.265847744427303, 23.135538697791706],
              [-12.13525491562421, 8.816778784387099, 23.135538697791706],
              [-12.135254915624213, -8.816778784387095, 23.135538697791706],
              [4.635254915624208, -14.265847744427305, 23.135538697791706]]

for i in range(5):
    moveThroughPoints(point_list_pentagon, 20 * i + 10)
    time.sleep(0.5)

# move up and down
point_list_down = [[0, 0, 34.194800505029775 - 22.5 + 0.5]]
point_list_up = [[0, 0, 34.194800505029775 - 22.5 - 1 + 45]]
moveThroughPoints(point_list_down, 50)
time.sleep(0.5)
moveThroughPoints(point_list_up, 50)
time.sleep(0.5)
moveThroughPoints(point_list_down, 50)
time.sleep(0.5)
moveThroughPoints(point_list_up, 50)
time.sleep(1.5)

# move smoothly in a circular pattern three times, clockwise
point_list = [[15.0, 0.0, 23.135538697791706], [14.488887394336025, 3.882285676537811, 23.135538697791706], [12.99038105676658, 7.499999999999999, 23.135538697791706], [10.606601717798213, 10.606601717798213, 23.135538697791706], [7.500000000000002, 12.990381056766578, 23.135538697791706], [3.8822856765378146, 14.488887394336023, 23.135538697791706], [9.18485099360515e-16, 15.0, 23.135538697791706], [-3.8822856765378093, 14.488887394336025, 23.135538697791706], [-7.4999999999999964, 12.99038105676658, 23.135538697791706], [-10.606601717798211, 10.606601717798213, 23.135538697791706], [-12.990381056766577, 7.500000000000005, 23.135538697791706], [-14.488887394336023, 3.882285676537815, 23.135538697791706], [-15.0, 1.83697019872103e-15, 23.135538697791706], [-14.488887394336027, -3.8822856765378053, 23.135538697791706], [-12.990381056766582, -7.499999999999996, 23.135538697791706], [-10.606601717798219, -10.606601717798206, 23.135538697791706], [-7.500000000000007, -12.990381056766577, 23.135538697791706], [-3.8822856765378226, -14.488887394336022, 23.135538697791706], [-2.7554552980815444e-15, -15.0, 23.135538697791706], [3.8822856765378044, -14.488887394336027, 23.135538697791706], [7.49999999999999, -12.990381056766585, 23.135538697791706], [10.60660171779821, -10.606601717798215, 23.135538697791706], [12.990381056766577, -7.500000000000007, 23.135538697791706], [14.488887394336022, -3.8822856765378235, 23.135538697791706], [15.0, 0.0, 23.135538697791706], [14.488887394336025, 3.882285676537811, 23.135538697791706], [12.99038105676658, 7.499999999999999, 23.135538697791706], [10.606601717798213, 10.606601717798213, 23.135538697791706], [7.500000000000002, 12.990381056766578, 23.135538697791706], [3.8822856765378146, 14.488887394336023, 23.135538697791706], [9.18485099360515e-16, 15.0, 23.135538697791706], [-3.8822856765378093, 14.488887394336025, 23.135538697791706], [-7.4999999999999964, 12.99038105676658, 23.135538697791706], [-10.606601717798211, 10.606601717798213, 23.135538697791706], [-12.990381056766577, 7.500000000000005, 23.135538697791706], [-14.488887394336023, 3.882285676537815, 23.135538697791706], [-15.0, 1.83697019872103e-15, 23.135538697791706], [-14.488887394336027, -3.8822856765378053, 23.135538697791706], [-12.990381056766582, -7.499999999999996, 23.135538697791706], [-10.606601717798219, -10.606601717798206, 23.135538697791706], [-7.500000000000007, -12.990381056766577, 23.135538697791706], [-3.8822856765378226, -14.488887394336022, 23.135538697791706], [-2.7554552980815444e-15, -15.0, 23.135538697791706], [3.8822856765378044, -14.488887394336027, 23.135538697791706], [7.49999999999999, -12.990381056766585, 23.135538697791706], [10.60660171779821, -10.606601717798215, 23.135538697791706], [12.990381056766577, -7.500000000000007, 23.135538697791706], [14.488887394336022, -3.8822856765378235, 23.135538697791706], [15.0, 0.0, 23.135538697791706], [14.488887394336025, 3.882285676537811, 23.135538697791706], [12.99038105676658, 7.499999999999999, 23.135538697791706], [10.606601717798213, 10.606601717798213, 23.135538697791706], [7.500000000000002, 12.990381056766578, 23.135538697791706], [3.8822856765378146, 14.488887394336023, 23.135538697791706], [9.18485099360515e-16, 15.0, 23.135538697791706], [-3.8822856765378093, 14.488887394336025, 23.135538697791706], [-7.4999999999999964, 12.99038105676658, 23.135538697791706], [-10.606601717798211, 10.606601717798213, 23.135538697791706], [-12.990381056766577, 7.500000000000005, 23.135538697791706], [-14.488887394336023, 3.882285676537815, 23.135538697791706], [-15.0, 1.83697019872103e-15, 23.135538697791706], [-14.488887394336027, -3.8822856765378053, 23.135538697791706], [-12.990381056766582, -7.499999999999996, 23.135538697791706], [-10.606601717798219, -10.606601717798206, 23.135538697791706], [-7.500000000000007, -12.990381056766577, 23.135538697791706], [-3.8822856765378226, -14.488887394336022, 23.135538697791706], [-2.7554552980815444e-15, -15.0, 23.135538697791706], [3.8822856765378044, -14.488887394336027, 23.135538697791706], [7.49999999999999, -12.990381056766585, 23.135538697791706], [10.60660171779821, -10.606601717798215, 23.135538697791706], [12.990381056766577, -7.500000000000007, 23.135538697791706], [14.488887394336022, -3.8822856765378235, 23.135538697791706]]
moveThroughPoints_smooth(point_list, 20, 100)

# move smoothly in a circular pattern three times, counterclockwise
point_list_inverted_circle = list(reversed(point_list))
moveThroughPoints_smooth(point_list_inverted_circle, 40, 300)
