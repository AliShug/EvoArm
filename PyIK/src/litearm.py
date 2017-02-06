from __future__ import print_function

import numpy as np
import struct

import yaml

import solvers
import pid
from util import *

MOTORSPEED = 0.9
MOTORMARGIN = 1
MOTORSLOPE = 30

ERRORLIM = 5.0

class ServoMapping:
    """Given a directional ratio and offset, maps between a servo position and an arm pose angle."""
    def __init__(self, ratio = 1.0, offset = 0.0, limits = [0, 300]):
        self.ratio = ratio
        self.offset = offset
        self.limits = limits

    def fromServo(self, servo_angle):
        return servo_angle * self.ratio + self.offset

    def toServo(self, arm_angle):
        return (arm_angle - self.offset) / self.ratio

class ArmConfig:
    """Loads and stores an arm's proportions, limits and other configuration data"""
    def __init__(self):
        pass

    def loadConfigFile(self, config_file):
        file = open(config_file)
        config = yaml.load(file.read())
        file.close()
        self.loadConfigData(config)

    def loadConfigData(self, config):
        self.name = config['name']

        arm = config['config']
        self.main_length = arm['elevator_length']
        self.forearm_length = arm['forearm_length']
        self.linkage_length = arm['linkage_length']
        self.lower_actuator_length = arm['lower_actuator_length']
        self.upper_actuator_length = arm['upper_actuator_length']
        self.wrist_length = arm['wrist_length']
        self.shoulder_offset = arm['shoulder_offset']

        self.servos = {}
        for s in arm['servos']:
            mapping = ServoMapping(s['ratio'], s['offset'], s['limits'])
            self.servos[s['name']] = mapping

class ArmPose:
    """
    Defines a physical configuration of a LiteArm-style robot arm.
    Internal angles are relative to vertical (elevator/actuator) or straight
    forward (swing), and are stored in radians. Extracted servo angles range
    0-300 and are measured in degrees.

    Provides methods for:
        - finding the required servo angles to reach the pose
        - checking the validity of the pose
    """
    structFormat = 'fffff'

    def __init__(self,
                 arm_config,
                 swing_angle,
                 elevator_angle,
                 actuator_angle,
                 elbow_angle,
                 elbow2D,
                 wrist2D,
                 effector2D,
                 effector,
                 wrist_x,
                 wrist_y):
        self.cfg = arm_config
        self.angles = {}
        self.angles['swing'] = swing_angle
        self.angles['elevator'] = elevator_angle
        self.angles['actuator'] = actuator_angle
        self.elbow_angle = elbow_angle
        # Joints in the arm
        shoulder = rotate(self.cfg.shoulder_offset, swing_angle)
        self.shoulder2D = [self.cfg.shoulder_offset[1], 0]
        self.shoulder = [shoulder[0], 0, shoulder[1]]
        self.wrist2D = wrist2D
        self.effector2D = effector2D
        self.effector = effector
        # Construct the 3D elbow & wrist positions from the 2D (planar) IK
        # solution
        arm_vec = effector - self.shoulder
        arm_vec[1] = 0
        self.elbow2D = elbow2D
        self.elbow = self.shoulder + normalize(arm_vec)*elbow2D[0]
        self.elbow[1] = elbow2D[1]
        self.wrist = self.effector - normalize(arm_vec)*arm_config.wrist_length
        # Wrist pose
        self.angles['wrist_x'] = wrist_x
        self.angles['wrist_y'] = wrist_y

    def getServoAngle(self, servo_name):
        angle = degrees(self.angles[servo_name])
        return self.cfg.servos[servo_name].toServo(angle)

    def setServoAngle(self, servo_name, angle):
        adjusted = self.cfg.servos[servo_name].fromServo(angle)
        self.angles[servo_name] = radians(adjusted)

    def armDiffAngle(self):
        return degrees(self.angles['elevator'] - self.angles['actuator'])

    def checkServoAngle(self, servo_name):
        servo = self.cfg.servos[servo_name]
        angle = self.getServoAngle(servo_name)
        return angle >= servo.limits[0] and angle <= servo.limits[1]

    def checkServoAngles(self):
        """Checks that the servo angles for the pose are within defined limits; SKIPS WRIST SERVOS!"""
        valid = True
        for name in self.cfg.servos.keys():
            if name in ('wrist_x', 'wrist_y'):
                continue
            valid = valid and self.checkServoAngle(name)
        return valid
        #return True

    def checkDiff(self):
        angle = self.armDiffAngle()
        return angle >= 44 and angle <= 175

    def checkForearm(self):
        angle = degrees(self.elbow_angle + self.angles['elevator'])
        return angle < 200 and angle > 80

    def checkPositioning(self):
        # When Y>0 Forearm always faces outwards
        if self.wrist2D[1] > 0 and self.wrist2D[0] < self.elbow2D[0]:
            return False
        # No valid positions X<=0
        if self.wrist2D[0] <= 0:
            return False
        # Effector height range
        if self.effector[1] > 180 or self.effector[1] < -200:
            return False
        return True

    def checkClearance(self):
        return (self.checkDiff() and self.checkServoAngles() and
                self.checkPositioning() and self.checkForearm())

    def serialize(self):
        """Returns a packed struct holding the pose information"""
        return struct.pack(
            ArmPose.structFormat,
            self.angles['swing'],
            self.angles['elevator'],
            self.elbow_angle,
            self.angles['wrist_x'],
            self.angles['wrist_y']
        )

class ArmController:
    def __init__(self,
                 servo_swing,
                 servo_shoulder,
                 servo_elbow,
                 servo_wrist_x,
                 servo_wrist_y,
                 arm_config,
                 motion_enable = False):
        # Solvers are responsible for calculating the target servo positions to
        # reach a given goal position
        self.ik = solvers.IKSolver(
            arm_config.main_length,
            arm_config.forearm_length,
            arm_config.wrist_length,
            arm_config.shoulder_offset)
        self.physsolver = solvers.PhysicalSolver(
            arm_config.main_length,
            arm_config.linkage_length,
            arm_config.lower_actuator_length,
            arm_config.upper_actuator_length)
        # Servos
        self.servos = {}
        self.servos["swing"] = servo_swing
        self.servos["shoulder"] = servo_shoulder
        self.servos["elbow"] = servo_elbow
        self.servos["wrist_x"] = servo_wrist_x
        self.servos["wrist_y"] = servo_wrist_y
        for key, servo in self.servos.iteritems():
            if servo is None:
                print ("Warning: {0} servo not connected".format(key))
            else:
                # Initialise a PID controller for the servo
                if servo.protocol == 1:
                    servo.setGoalSpeed(-MOTORSPEED)
                    servo.data['pid'] = pid.PIDControl(2.4, 0, 0.4)
                else:
                    servo.setGoalSpeed(0)
                servo.data['error'] = 0.0
                # Make sure the goal speed is set
                servo.setTorqueEnable(1)
                if servo.protocol == 1:
                    print("Setting slope")
                    servo.setCWMargin(MOTORMARGIN)
                    servo.setCCWMargin(MOTORMARGIN)
                    servo.setCWSlope(MOTORSLOPE)
                    servo.setCCWSlope(MOTORSLOPE)
        # Store parameters
        self.motion_enable = True
        self.enableMovement(False)
        self.cfg = arm_config
        # Dirty flags for stored poses
        self.ik_pose = None
        self.ik_dirty = True
        self.real_pose = None
        self.real_dirty = True
        # Current target pose
        self.target_pose = None

    def enableMovement(self, enable):
        changed = False
        if enable and not self.motion_enable:
            print ("Warning: Arm enabled")
            self.motion_enable = True
            changed = True
        elif not enable:
            self.motion_enable = False
            changed = True
        if changed:
            # Set servos on/off
            for name, servo in self.servos.items():
                if servo is not None:
                    servo.setTorqueEnable(self.motion_enable)

    def setWristGoalPosition(self, pos):
        self.ik.setGoal(pos)
        self.ik_dirty = True

    def setWristGoalDirection(self, normal):
        self.ik.setWristDir(normal)
        self.ik_dirty = True

    def getIKPose(self):
        if self.ik_dirty and self.ik.valid:
            # Construct geometry of arm from IK state
            main_arm = self.ik.elbow - self.ik.originpl
            arm_vert_angle = sigangle(main_arm, vertical)
            forearm = self.ik.wristpl - self.ik.elbow
            elbow_angle = angle_between(main_arm, forearm)
            # Solve actuator angle for given elbow angle
            # Base angle is between the main arm and actuator
            base_angle = self.physsolver.inverse_forearm(elbow_angle)
            actuator_angle = arm_vert_angle - base_angle

            self.ik_pose = ArmPose(
                self.cfg,
                swing_angle = self.ik.swing,
                # angles from vertical
                elevator_angle = arm_vert_angle,
                actuator_angle = actuator_angle,
                # angle between the main arm and forearm
                elbow_angle = elbow_angle,
                elbow2D = self.ik.elbow,
                wrist2D = self.ik.wristpl,
                effector2D = self.ik.goalpl,
                effector = self.ik.goal,
                wrist_x = self.ik.wrist_x,
                wrist_y = self.ik.wrist_y
            )
        return self.ik_pose

    def pollServos(self):
        """Poll the real-world servo positions"""
        for servo in self.servos.itervalues():
            if servo is not None:
                newPos = servo.getPosition()
                if type(newPos) is float:
                    servo.data['pos'] = newPos

    def clearPositionError(self):
        """Clears the servo's position-error accumulators"""
        for servo in self.servos.itervalues():
            if servo is not None and servo.protocol == 1:
                servo.data['error'] = 0.0

    def getRealPose(self):
        """Retrieve the real-world arm pose, or None if not all servos are
        connected.
        """
        if any([servo is None for servo in self.servos.itervalues()]):
            return None

        # This whole function is essentially just FK based on the known servo
        # angles
        swing_servo = self.servos['swing'].data['pos']
        elevator_servo = self.servos['shoulder'].data['pos']
        actuator_servo = self.servos['elbow'].data['pos']
        wrist_x_servo = self.servos['wrist_x'].data['pos']
        wrist_y_servo = self.servos['wrist_y'].data['pos']

        # Find the internal arm-pose angles for the given servo positions
        swing_angle = self.cfg.servos['swing'].fromServo(swing_servo)
        swing_angle = radians(swing_angle)
        elevator_angle = self.cfg.servos['elevator'].fromServo(elevator_servo)
        elevator_angle = radians(elevator_angle)
        actuator_angle = self.cfg.servos['actuator'].fromServo(actuator_servo)
        actuator_angle = radians(actuator_angle)
        wrist_x_angle = self.cfg.servos['wrist_x'].fromServo(wrist_x_servo)
        wrist_x_angle = radians(wrist_x_angle)
        wrist_y_angle = self.cfg.servos['wrist_y'].fromServo(wrist_y_servo)
        wrist_y_angle = radians(wrist_y_angle)
        # Solve elbow angle for given actuator and elevator angles
        # (this is the angle from the elevator arm's direction to the forearm's)
        elbow_angle = self.physsolver.solve_forearm(elevator_angle, actuator_angle)

        # FK positions from config and angles
        offset = self.cfg.shoulder_offset
        shoulder2D = np.array([offset[1], 0])
        elbow2D = shoulder2D + rotate(vertical, elevator_angle)*self.cfg.main_length
        wrist2D = elbow2D + rotate(vertical, elevator_angle + elbow_angle)*self.cfg.forearm_length
        effector2D = wrist2D + [self.cfg.wrist_length, 0]
        # 3D Effector calculation is a little more involved
        td = rotate([offset[0], effector2D[0]], swing_angle)
        effector = np.array([td[0], effector2D[1], td[1]])

        pose = ArmPose(
            self.cfg,
            swing_angle, elevator_angle, actuator_angle,
            elbow_angle, elbow2D, wrist2D, effector2D,
            effector, wrist_x_angle, wrist_y_angle)
        return pose

    def setTargetPose(self, new_pose):
        self.target_pose = new_pose

    def tick(self):
        if self.target_pose is not None:
            if self.motion_enable:
                # Drive servos
                gain = 0.1
                if self.servos['swing'] is not None:
                    s = self.servos['swing']
                    pos = s.data['pos']
                    target = self.target_pose.getServoSwing()
                    # err = min(10, pos-target)
                    # s.data['error'] += err*gain
                    s.setGoalPosition(target)
                if self.servos['shoulder'] is not None:
                    s = self.servos['shoulder']
                    # cumulative error
                    pos = s.data['pos']
                    target = self.target_pose.getServoElevator()
                    err = min(10, pos-target)
                    s.data['error'] += err*gain
                    s.data['error'] = np.clip(s.data['error'], -ERRORLIM, ERRORLIM)
                    s.setGoalPosition(target - s.data['error'])
                if self.servos['elbow'] is not None:
                    s = self.servos['elbow']
                    pos = s.data['pos']
                    target = self.target_pose.getServoActuator()
                    err = min(10, pos-target)
                    s.data['error'] += err*gain
                    s.data['error'] = np.clip(s.data['error'], -ERRORLIM, ERRORLIM)
                    s.setGoalPosition(target - s.data['error'])

                if self.servos['wrist_x'] is not None:
                    self.servos['wrist_x'].setGoalPosition(self.target_pose.getServoWristX())
                if self.servos['wrist_y'] is not None:
                    self.servos['wrist_y'].setGoalPosition(self.target_pose.getServoWristY())
