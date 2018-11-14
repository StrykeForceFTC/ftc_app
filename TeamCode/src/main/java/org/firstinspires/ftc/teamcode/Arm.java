package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class for controlling two motors on arm of 2018 robot. One motor raises & lowers the arm and
 * the other rotates it.
 */

public class Arm {

    // TODO: declare motors
    private DcMotor lift;
    //private DcMotor wrist;

    // TODO: Create constants for different arm positions


    // Constructor
    // TODO: Initialize HardWare and variables here
    public Arm(HardwareMap ahwMap) {
        // TODO: set up motor objects using ahwMap.dcMotor.get( )
        lift = ahwMap.dcMotor.get("arm_lift_motor");
        //wrist = ahwMap.dcMotor.get("arm_wrist_motor");

        // TODO: Setup motors (set direction, run mode, use encoder, etc.)
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //wrist.setDirection(DcMotor.Direction.FORWARD);
        //wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public enum lift_pos {
        fulldown, fullup, mid1, mid2
    }

    public void position_lift(lift_pos pos, int speed) {
        lift.setPower(TranslateLiftSpeed(speed));
        lift.setTargetPosition(TranslateLiftPos(pos));
        //wrist.setPower(TranslateWristSpeed);
        //wrist.setTargetPosition(TranslateWristPos(pos));
    }

    public enum lift_dir {
        up, down
    }

    public void adjust_lift(lift_dir dir, double adjustrate, int speed) {
        lift.setPower(TranslateLiftSpeed(speed));
        lift.setTargetPosition(TranslateLiftAdjust(dir, adjustrate));
        //wrist.setPower(TranslateWristSpeed(speed));
        //wrist.setTargetPosition(TranslateWristAdjust(dir, adjustrate));
    }

    public boolean isInPos()
    {
        if(lift.isBusy())
            return (false);
        return (true);
    }

    private static double MAX_WAIT_TIME = 5.0;

    public void WaitForInPos()
    {   ElapsedTime limitTimer = new ElapsedTime();

        limitTimer.reset();

        while (limitTimer.seconds() < MAX_WAIT_TIME)
        {
            if (isInPos())
                return;
        }

        lift.setTargetPosition(lift.getCurrentPosition());
    }

    /*
     * Method to provide continuous control of Arm in auton. After choosing a position, this method monitors whether a position has
     * been reached.
     */
    public void Control() {

    }

    // Method to set desired position as load samples
    public void LoadSamplesPositionSet() {
    }

    // Method to set desired position as offload samples
    public void OffloadSamplesPositionSet() {
    }

    // Method to set desired position as hang
    public void HangPositionSet() {
    }

    // TODO: Create methods for use in auton

    /*
     ** Debug methods for getting encoder values and moving motors
     */
    private double TranslateLiftSpeed(int speed) {
        if (speed > 10)
            speed = 10;
        if (speed < 0)
            speed = 0;

        return (((double) speed) / 10);
    }

    //private double TranslateWristSpeed(int speed) {
        //if (speed > 10)
            //speed = 10;
        //if (speed < 0)
            //speed = 0;

        //return (((double) speed) / 10);
    //}

    private int TranslateLiftPos(lift_pos pos)
    {
        switch(pos)
        {
            case fulldown:
                return 100;
            case fullup:
                return 4400;
            case mid1:
                return 2000;
            case mid2:
                return 4000;
            default:
                return lift.getCurrentPosition();
        }
    }

    //private int TranslateWristPos(lift_pos pos)
    //{
        //switch(pos)
        //{
            // fulldown:
                //return 5000;
            //case fullup:
                //return 3000;
            //case mid1:
                //return 2000;
            //case mid2:
                //return 4000;
            //default:
                //return lift.getCurrentPosition();
        //}
    //}

    private int MAX_LIFT_ADJUST_VALUE = 300;
    private int MIN_LIFT_ADJUST_VALUE = 20;
    private int MAX_LIFT_POS = 5000;
    private int MIN_LIFT_POS = -5000;

    //private int MAX_WRIST_ADJUST_VALUE = 300;
    //private int MIN_WRIST_ADJUST_VALUE = 20;
    //private int MAX_WRIST_POS = 5000;
    //private int MIN_WRIST_POS = -5000;


    private int LiftPosLimit(int pos)
    {
        if (pos > MAX_LIFT_POS)
            return MAX_LIFT_POS;
        if ((lift.getCurrentPosition() > 10) && (pos < 100))
            return 100;
        if (pos < MIN_LIFT_POS)
            return MIN_LIFT_POS;
        return pos;
    }

    private int TranslateLiftAdjust(lift_dir dir, double adjustrate)
    {
         if (adjustrate > 1)
             adjustrate = 1;
         if (adjustrate < 0)
             adjustrate = 0;

         int offset = (int) (adjustrate * MAX_LIFT_ADJUST_VALUE +.5);

         if (lift.getCurrentPosition() < 5 )
             offset = MIN_LIFT_ADJUST_VALUE;

         switch(dir)
         {
             case up:
                 return LiftPosLimit(lift.getCurrentPosition() - offset);
             case down:
                 return LiftPosLimit(lift.getCurrentPosition() + offset);
             default:
                 return lift.getCurrentPosition();
         }
    }


    public int LiftEncoderValue( )
    {
        // TODO: Make return encoder value
        return (lift.getCurrentPosition());
    }

    public int WristEncoderValue( )
    {
        // TODO: Make return encoder value
        return 0;
    }

}
