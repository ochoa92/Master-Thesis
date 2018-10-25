//============================================================================
// Name        : kinova_api_funcs.cpp
// Author      : Hélio Ochoa
// Version     : 0.0
// Description : A cpp file to acess api functions
//============================================================================

#include <iostream>
#include <dlfcn.h> //Ubuntu
#include "kinova/Kinova.API.USBCommandLayerUbuntu.h"
#include "kinova/Kinova.API.USBCommLayerUbuntu.h"
#include "kinova/KinovaTypes.h"
#include <unistd.h>

using namespace std;

int main(int argc, char **argv)
{

    int result;
    int programResult = 0;

    //Handle for the library's command layer.
    void * commandLayer_handle;

    //Function pointers to the functions we need
    int (*MyInitAPI)();
    int (*MyCloseAPI)();
    int (*MyMoveHome)();
    int (*MySwitchTrajectoryTorque)(GENERALCONTROL_TYPE);
    int (*MySetTorqueActuatorGain)(float Command[COMMAND_SIZE]);
    int (*MySetTorqueControlType)(TORQUECONTROL_TYPE type);
    int (*MySetTorqueSafetyFactor)(float factor);
    int (*MySetTorqueVibrationController)(float value);
    int (*MyEraseAllTrajectories)();

    //We load the library
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

    //We load the functions from the library
    MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
    MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
    MyMoveHome = (int(*)()) dlsym(commandLayer_handle, "MoveHome");
    MySwitchTrajectoryTorque = (int(*)(GENERALCONTROL_TYPE)) dlsym(commandLayer_handle, "SwitchTrajectoryTorque");
    MySetTorqueActuatorGain = (int(*)(float Command[COMMAND_SIZE])) dlsym(commandLayer_handle, "SetTorqueActuatorGain");
    MySetTorqueControlType = (int(*)(TORQUECONTROL_TYPE)) dlsym(commandLayer_handle, "SetTorqueControlType");
    MySetTorqueSafetyFactor = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueSafetyFactor");
    MySetTorqueVibrationController = (int(*)(float)) dlsym(commandLayer_handle, "SetTorqueVibrationController");
    MyEraseAllTrajectories = (int (*)()) dlsym(commandLayer_handle,"EraseAllTrajectories");

    //Verify that all functions has been loaded correctly
    if((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyEraseAllTrajectories == NULL))
    {
        cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
        programResult = 0;
    }
    else
    {
        cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

        result = (*MyInitAPI)();
        cout << "Initialization's result :" << result << endl;

        result = (*MyEraseAllTrajectories)();
        cout << "All trajectories have been deleted." << endl;

        // If the API is initialized and the communication with the robot is working
        if (result == 1)
        {
            cout << "API initialization worked" << endl;
            cout << "The robot will swich to torque control mode and move. Be cautious." << endl;

            // Set to position mode
            MySwitchTrajectoryTorque(POSITION);

            // Move to home position
            MyMoveHome();

            // Set the torque control type to Direct Torque Control
            MySetTorqueControlType(DIRECTTORQUE);

            // Set the safety factor to 1
            MySetTorqueSafetyFactor(1.0);

            // Set the Vibration controller to 0.5
            MySetTorqueVibrationController(1.0);

            // Switch to torque control
            // (Here we switch before sending torques. The switch is possible because the gravity torques are already taken into account.)
            MySwitchTrajectoryTorque(TORQUE);

            // Initialize the torque commands
            float FeedbackGain[COMMAND_SIZE]; //feedback gain in torque mode
            for (int i = 0; i < COMMAND_SIZE; i++)
            {
                FeedbackGain[i] = 0.0;
            }

            // Send the torque actuator Gains
            MySetTorqueActuatorGain(FeedbackGain);

            // Wait
            //usleep(2000000);
            // Switch back to position
            //MySwitchTrajectoryTorque(POSITION);

        }
        else
        {
            cout << "API initialization failed" << endl;
        }
        cout << endl << "C L O S I N G   A P I" << endl;
        result = (*MyCloseAPI)();
        programResult = 1;
    }

    dlclose(commandLayer_handle);

    return programResult;
}
