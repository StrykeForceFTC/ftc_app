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
    private DcMotor wrist;


    // Constructor - doesn't zero encoders
    public Arm( HardwareMap ahwMap ) {
        this( ahwMap, false );
    }

    // Constructor
    // Initialize HardWare and variables
    public Arm(HardwareMap ahwMap, boolean resetEncoders ) {

        lift = ahwMap.dcMotor.get("arm_lift_motor");
        wrist = ahwMap.dcMotor.get("arm_wrist_motor");

        // Setup motors (set direction, run mode, use encoder, etc.)
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setDirection(DcMotor.Direction.FORWARD);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if ( resetEncoders )
        {
            lift.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
            wrist.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        }

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void Stop( )
    {
        lift.setPower( 0.0 );
        wrist.setPower( 0.0 );
    }

    public enum lift_pos {
        fulldown, fullup, mid1, mid2
    }

    public enum WRIST_POS {
        START, UNLOAD, MOVE, LOAD
    }

    public void position_lift(lift_pos pos, int speed) {
        lift.setPower(TranslateLiftSpeed(speed));
        lift.setTargetPosition(TranslateLiftPos(pos));
    }

    public void position_wrist(WRIST_POS pos, int speed) {

        wrist.setPower(TranslateWristSpeed(speed));
        wrist.setTargetPosition(TranslateWristPos(pos));
    }

    public enum lift_dir {
        up, down
    }

    public enum WRIST_DIR {
        CLOCKWISE, COUNTER_CLOCKWISE
    }

    public void adjust_lift(lift_dir dir, double adjustrate, int speed) {
        lift.setPower(TranslateLiftSpeed(speed));
        lift.setTargetPosition(TranslateLiftAdjust(dir, adjustrate));
    }

    public void adjust_wrist(WRIST_DIR dir, double adjustrate, int speed) {
        wrist.setPower(TranslateWristSpeed(speed));
        wrist.setTargetPosition(TranslateWristAdjust(dir, adjustrate));
    }

    public boolean isInPos()
    {
        if(lift.isBusy())
            return (false);
        return (true);
    }

    public boolean wristIsInPos()
    {
        if(wrist.isBusy())
            return (false);
        return (true);
    }

    private static double MAX_WAIT_TIME = 2.0;

    public void WaitForInPos()
    {   ElapsedTime limitTimer = new ElapsedTime();

        limitTimer.reset();

        while (limitTimer.seconds() < MAX_WAIT_TIME)
        {
            if (isInPos() )
                return;
        }

        lift.setTargetPosition(lift.getCurrentPosition());
    }


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

    private double TranslateWristSpeed(int speed) {
        if (speed > 10)
            speed = 10;
        if (speed < 0)
            speed = 0;

        return (((double) speed) / 10);
    }

    private int TranslateLiftPos(lift_pos pos)
    {
        switch(pos)
        {
            case fulldown:
                return 200;
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

    private int TranslateWristPos(WRIST_POS pos)
    {
        switch(pos)
        {

            case START:            // Folded down at start or end, offset for some margin
                return 200;
            case UNLOAD:           // Leaned a little bit forward of vertical
                return 2700;
            case MOVE:             // Pointed out to front high enough to not get in way when moving
                return 3700;
            case LOAD:             // Pointed towards ground in front of robot
                return 4850;
            default:
                return wrist.getCurrentPosition();
        }
    }

    private int MAX_LIFT_ADJUST_VALUE = 300;
    private int MIN_LIFT_ADJUST_VALUE = 20;
    private int MAX_LIFT_POS = 5000;
    private int MIN_LIFT_POS = 50;

    private int MAX_WRIST_ADJUST_VALUE = 300;
    private int MIN_WRIST_ADJUST_VALUE = 20;
    private int MAX_WRIST_POS = 5200;
    private int MIN_WRIST_POS = 100;


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

    private int WristPosLimit(int pos)
    {
        if (pos > MAX_WRIST_POS)
            return MAX_WRIST_POS;
        if ((wrist.getCurrentPosition() > 10) && (pos < 100))
            return 100;
        if (pos < MIN_WRIST_POS)
            return MIN_WRIST_POS;
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


    private int TranslateWristAdjust(WRIST_DIR dir, double adjustrate)
    {
        if (adjustrate > 1)
            adjustrate = 1;
        if (adjustrate < 0)
            adjustrate = 0;

        int offset = (int) (adjustrate * MAX_WRIST_ADJUST_VALUE +.5);

        if (wrist.getCurrentPosition() < 1000 )
            offset = MIN_WRIST_ADJUST_VALUE;

        switch(dir)
        {
            case CLOCKWISE:
                return WristPosLimit(wrist.getCurrentPosition() + offset);
            case COUNTER_CLOCKWISE:
                return WristPosLimit(wrist.getCurrentPosition() - offset);
            default:
                return wrist.getCurrentPosition();
        }
    }


    public int LiftEncoderValue( )
    {
        return (lift.getCurrentPosition());
    }

    public int WristEncoderValue( )
    {
        return wrist.getCurrentPosition();
    }

}
