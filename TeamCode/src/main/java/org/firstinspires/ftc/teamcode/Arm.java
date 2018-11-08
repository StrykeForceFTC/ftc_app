package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class for controlling two motors on arm of 2018 robot. One motor raises & lowers the arm and
 * the other rotates it.
 */

public class Arm {
    public enum lift_pos {
        fulldown, fullup, mid1, mid2
    }

    public enum lift_dir {
        up, down
    }

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
        //wrist.setTargetPosition(lift.getCurrentPosition());
        //wrist.setDirection(DcMotor.Direction.FORWARD);
        //wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    private double TranslateLiftPower(int power) {
        if (power > 10)
            power = 10;
        if (power < 0)
            power = 0;

        return (power / 10);
    }

    private int TranslateLiftPos(lift_pos pos)
    {
        switch(pos)
        {
            case fulldown:
                return 5000;
            case fullup:
                return 3000;
            case mid1:
                return 2000;
            case mid2:
                return 4000;
            default:
                return lift.getCurrentPosition();
        }
    }

    public void position_lift(lift_pos pos, int power)
    {
        lift.setPower(TranslateLiftPower(power));
        lift.setTargetPosition(TranslateLiftPos(pos));
        //wrist.setPower(TranslateWristPower);
        //wrist.setTargetPosition(TranslateWristPos());
    }

    public void adjust_lift(lift_dir dir, double adjustrate, int power)
    {
        lift.setPower(TranslateLiftPower(power));
        lift.setTargetPosition(TranslateLiftAdjust(dir, adjustrate));
        //wrist.setPower(TranslateWristPower);
        //wrist.setTargetPosition(TranslateWristPos());
    }

    private int MAX_LIFT_ADJUST_VALUE = 5;
    private int MAX_LIFT_POS = 10000;
    private int MIN_LIFT_POS = -10000;


    private int LiftPosLimit(int pos)
    {
        if (pos > MAX_LIFT_POS)
            return MAX_LIFT_POS;
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

         switch(dir)
         {
             case up:
                 return LiftPosLimit(lift.getCurrentPosition() + offset);
             case down:
                 return LiftPosLimit(lift.getCurrentPosition() - offset);
             default:
                 return lift.getCurrentPosition();
         }
    }

    public void Move(int counts, double power)
    {
        lift.setPower(power);
        lift.setTargetPosition(lift.getCurrentPosition() + counts);
        //wrist.setPower(power);
        //wrist.setTargetPosition(wrist.getCurrentPosition() + counts);
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
