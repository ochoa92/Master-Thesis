#!/usr/bin/python

import roslib; roslib.load_manifest('thesis_demo')
import rospy

from ctypes import *

"""
Global variables and Structures
"""
class FeedbackGain(Structure):
    _fields_ = [("Actuator1", c_float),
                ("Actuator2", c_float),
                ("Actuator3", c_float),
                ("Actuator4", c_float),
                ("Actuator5", c_float),
                ("Actuator6", c_float)]


class AngularForceInfo(Structure):
    _fields_ = [("Actuator1", c_float),
                ("Actuator2", c_float),
                ("Actuator3", c_float),
                ("Actuator4", c_float),
                ("Actuator5", c_float),
                ("Actuator6", c_float)]


class AngularForce(Structure):
    _fields_ = [("Actuators", AngularForceInfo)]


class ActuatorDamping(Structure):
    _fields_ = [("Actuator1", c_float),
                ("Actuator2", c_float),
                ("Actuator3", c_float),
                ("Actuator4", c_float),
                ("Actuator5", c_float),
                ("Actuator6", c_float)]


NO_ERROR_KINOVA = 1


class KinovaAPI(object):
    def __init__(self):

        """
        Create the hooks for the API
        """
        self.kinova = CDLL("Kinova.API.USBCommandLayerUbuntu.so")
        self.InitAPI = self.kinova.InitAPI
        self.CloseAPI = self.kinova.CloseAPI
        self.MoveHome = self.kinova.MoveHome
        self.InitFingers = self.kinova.InitFingers
        self.SwitchTrajectoryTorque = self.kinova.SwitchTrajectoryTorque
        self.SetTorqueSafetyFactor = self.kinova.SetTorqueSafetyFactor
        self.SetTorqueSafetyFactor.argtypes = [POINTER(c_float)]
        self.SetTorqueVibrationController = self.kinova.SetTorqueVibrationController
        self.SetTorqueVibrationController.argtypes = [POINTER(c_float)]
        self.SetTorqueActuatorGain = self.kinova.SetTorqueActuatorGain
        self.SetTorqueActuatorGain.argtypes = [POINTER(FeedbackGain)]
        self.GetAngularForceGravityFree = self.kinova.GetAngularForceGravityFree
        self.GetAngularForceGravityFree.argtypes = [POINTER(AngularForce)]
        self.SetTorqueActuatorDamping = self.kinova.SetTorqueActuatorDamping
        self.SetTorqueActuatorDamping.argtypes = [POINTER(ActuatorDamping)]

    def send_torque_actuator_gain(self, cmds):
        gain = FeedbackGain()
        api_stat = self.SetTorqueActuatorGain(byref(gain))

        if NO_ERROR_KINOVA == api_stat:
            gain.Actuator1 = cmds[0]
            gain.Actuator2 = cmds[1]
            gain.Actuator3 = cmds[2]
            gain.Actuator4 = cmds[3]
            gain.Actuator5 = cmds[4]
            gain.Actuator6 = cmds[5]


    def get_angular_force_gravity_free(self):
        eff = AngularForce()
        api_stat = self.GetAngularForceGravityFree(byref(eff))

        if NO_ERROR_KINOVA == api_stat:
            ret = [eff.Actuators.Actuator1,
                   eff.Actuators.Actuator2,
                   eff.Actuators.Actuator3,
                   eff.Actuators.Actuator4,
                   eff.Actuators.Actuator5,
                   eff.Actuators.Actuator6]

        else:
            rospy.loginfo("Kinova API failed: GetAngularForceGravityFree (%d)", api_stat)
            ret = []

        return ret


    def send_torque_actuator_damping(self, cmds):
        damping = ActuatorDamping()
        api_stat = self.SetTorqueActuatorDamping(byref(damping))

        if NO_ERROR_KINOVA == api_stat:
            damping.Actuator1 = cmds[0]
            damping.Actuator2 = cmds[1]
            damping.Actuator3 = cmds[2]
            damping.Actuator4 = cmds[3]
            damping.Actuator5 = cmds[4]
            damping.Actuator6 = cmds[5]


def main():

    api = KinovaAPI()

    result = api.InitAPI()
    if result == 1:
        print("\nI N I T I A L I Z A T I O N   C O M P L E T E D")

        # Set to position mode
        api.SwitchTrajectoryTorque(0)

        # Move to home position
        api.MoveHome()

        # Set the safety factor to 1
        api.SetTorqueSafetyFactor(c_float(1))

        # Set the Vibration controller to 0.5
        api.SetTorqueVibrationController(c_float(0.5))

        # Switch to torque control
        api.SwitchTrajectoryTorque(1)
        print("\nThe robot will switch to torque control mode and move. Be cautious.")

        FeedbackGain = [0, 0, 0, 0, 0, 0]   # feedback gain in torque mode
        api.send_torque_actuator_gain(FeedbackGain)

        AngularForce = api.get_angular_force_gravity_free()
        print "\nAngularForce: ", AngularForce

        ActuatorDamping = [0, 0, 0, 0, 0, 0]    # set the actuator Actuator Damping
        api.send_torque_actuator_damping(ActuatorDamping)

    else:
        print("* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *")

    print("\nC L O S I N G   A P I")
    api.CloseAPI()


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
