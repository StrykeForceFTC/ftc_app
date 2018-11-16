package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class for controlling load/unload motor on arm of 2018 robot.
 */

public class Loader {
    // motor to spin lodar
    private DcMotor loadmotor = null;

    // Constants for speed
    private static final double LOAD_POWER = 0.9;
    private static final double UNLOAD_POWER = -0.9;
    private static final double LOAD_MS = 900;
    private static final double UNLOAD_MS = 900;

    // timer for auton

    private ElapsedTime runtime = new ElapsedTime();

// run time for auton
    // Constructor

    public Loader(HardwareMap ahwMap) {

        loadmotor = ahwMap.dcMotor.get("loader");

        loadmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        loadmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    // Method to load in tele op
    public void TeleopLoad() {
        // run motar at load power to load stuff
        loadmotor.setPower(LOAD_POWER);
    }

    // Method to unload in tele op
    public void TeleopUnloadSamples() {
        // run motor at load power to unload stuff
        loadmotor.setPower(UNLOAD_POWER);
    }

    //Method to stop in teleop
    public void teleopstop() {  //stop motor to stop loading
        loadmotor.setPower(0);
    }


    // Method to unload in auton
    public void AutonUnload() {
        TeleopUnloadSamples();
        Delay_ms(UNLOAD_MS);
        teleopstop();
    }

    // Method to load in auton
    public void AutonLoad() {
        TeleopLoad();
        Delay_ms(LOAD_MS);
        teleopstop();
    }


    private void Delay_ms( double milliseconds ) {
        try {
            Thread.sleep((long) (milliseconds));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}
