package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class for controlling load/unload motor on arm of 2018 robot.
 */

public class Loader
{
    // TODO: declare motor

    // TODO: Create constants for different arm positions

    // TODO: Create timer for running in auton

    // Constructor
    // TODO: Initialize HW and variables here
    public Loader( HardwareMap ahwMap )
    {
        // TODO: set up motor object using ahwMap.dcMotor.get( )

        // TODO: Setup motor (set direction, run mode, use encoder, etc.)
    }


    // Method to load in tele op
    public void TeleopLoadSamples( boolean load )
    {
        // TODO: when load is true, run the motor to load, when false, turn motor off
    }

    // Method to unload in tele op
    public void TeleopUnloadSamples( boolean unload )
    {
        // TODO: when unload is true, run the motor to unload, when false, turn motor off
    }

    // Method to unload in auton
    public void AutonUnload( )
    {
    }

    // Method to load in auton
    public void AutonLoad( )
    {
    }

}
