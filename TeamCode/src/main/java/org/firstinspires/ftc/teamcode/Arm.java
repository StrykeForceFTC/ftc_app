package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class for controlling two motors on arm of 2018 robot. One motor raises & lowers the arm and
 * the other rotates it.
 */

public class Arm
{
    // TODO: declare motors

    // TODO: Create constants for different arm positions

    // Constructor
    // TODO: Initialize HW and variables here
    public Arm( HardwareMap ahwMap )
    {
        // TODO: set up motor objects using ahwMap.dcMotor.get( )

        // TODO: Setup motors (set direction, run mode, use encoder, etc.)
    }


    /*
     * Method to provide continuous control of Arm in auton. After choosing a position, this method monitors whether a position has
     * been reached.
     */
    public void Control( )
    {

    }

    // Method to set desired position as load samples
    public void LoadSamplesPositionSet( )
    {
    }

    // Method to set desired position as offload samples
    public void OffloadSamplesPositionSet( )
    {
    }

    // Method to set desired position as hang
    public void HangPositionSet( )
    {
    }

    // TODO: Create methods for use in auton

    /*
    ** Debug methods for getting encoder values and moving motors
     */


    public int LiftEncoderValue( )
    {
        // TODO: Make return encoder value
        return 0;
    }

    public int WristEncoderValue( )
    {
        // TODO: Make return encoder value
        return 0;
    }

    public void LiftMotorPowerSet( double power )
    {
        // TODO: make set lift motor power
        // NOTE: to use methods that set power, may need to change motor run mode
    }

    public void WristMotorPowerSet( double power )
    {
        // TODO: make set wrist motor power
    }

}
