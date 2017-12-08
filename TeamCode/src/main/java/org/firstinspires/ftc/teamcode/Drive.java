package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gstaats on 18/09/17.
 */

public class Drive
{
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;

    // Limit timer
    private ElapsedTime limitTimer;
    private ElapsedTime delayTimer;


    // Constants for calculating number of ticks per cm to allow calculation of how many ticks
    // to go a given distance.
    final private static double WHEEL_DIAM_IN = 4.0;
    final private static double IN_2_CM = 2.54;

    // wheel circumference is PI times diameter, but diameter needs to be converted to cm from inches
    final private static double WHEEL_CIRCUM_CM = Math.PI * WHEEL_DIAM_IN * IN_2_CM;
    final private static double TICKS_PER_REV = 1120.0;
    final private static double TICKS_PER_CM = TICKS_PER_REV / WHEEL_CIRCUM_CM;

    // Constants for rotating robot a certain number of degrees. The robot wheels are
    // set at the corners of a 14" square, so this is used to calculate the circumference of
    // the circle that the robot wheels rotatate about. This is used to determine a number of
    // ticks per degree.
    final private static double ROBOT_LENGTH_IN = 21.85;
    final private static double ROBOT_DIAM_CM = Math.sqrt( 2.0 * ( ROBOT_LENGTH_IN * ROBOT_LENGTH_IN ) ) * IN_2_CM;
    final private static double ROBOT_CIRCUM_CM = ROBOT_DIAM_CM * Math.PI;
    final private static double CM_PER_DEGREE = ROBOT_CIRCUM_CM / 360.0;
    final private static double MAX_SECONDS = 4.5;
    final private static double AUTON_MEDIUM_DISTANCE_CM = 15.0;
    final private static double AUTON_SHORT_DISTANCE_CM = 8.0;
    final private static double TARGET_POWER_LONG = 0.4;
    final private static double TARGET_POWER_MEDIUM = 0.15;
    final private static double TARGET_POWER_SHORT = 0.075;
    final private static double INCREASING_FK = 0.3;                  // Increasing power filter constant
    final private static double DECREASING_FK = 0.8;                   // Decreasing power filter constant
    final private static int MOMENTUM_BUFFER_TICKS = 80;              // Stop ~1.5cm early to account for momentum

    private enum DIRECTION { FORWARD, REVERSE, RIGHT, LEFT, CLOCKWISE, COUNTERCLOCKWISE }

    public Drive(DcMotor FL, DcMotor FR, DcMotor RL, DcMotor RR )
    {
        // Define and Initialize Motors
        frontRight = FR;
        frontLeft  = FL;
        rearRight  = RR;
        rearLeft   = RL;

        // Set direction so positive is always forward with respect to
        // the robot. Right side motors need to be set to reverse, because
        // they spin counter-clockwise to move the robot forward.
        frontRight.setDirection( DcMotor.Direction.FORWARD );
        rearRight.setDirection( DcMotor.Direction.FORWARD );
        frontLeft.setDirection( DcMotor.Direction.REVERSE );
        rearLeft.setDirection( DcMotor.Direction.REVERSE );

        // Set all motors to zero power. Don't want robot moving till
        // we're ready.
        frontLeft.setPower( 0 );
        frontRight.setPower( 0 );
        rearLeft.setPower( 0 );
        rearRight.setPower( 0 );

        // Set all motors to run with encoders.

        frontLeft.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        frontRight.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rearLeft.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rearRight.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        // Set motors to not brake
        frontLeft.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
        frontRight.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
        rearLeft.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
        rearRight.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );

        // Set up timers
        limitTimer = new ElapsedTime( ElapsedTime.Resolution.SECONDS );
        delayTimer = new ElapsedTime( ElapsedTime.Resolution.MILLISECONDS );
    }


    /*
     *
     */
    public void MoveSimple( double xLeftRight, double yFwdRev, double rotate )
    {
        // variables for calculating wheel powers
        double frontLeftPower = ( -xLeftRight ) + ( yFwdRev ) + ( rotate );
        double frontRightPower = ( xLeftRight ) + ( yFwdRev ) + (-rotate);
        double rearLeftPower = xLeftRight + yFwdRev + rotate;
        double rearRightPower = (-xLeftRight) + ( yFwdRev ) + (-rotate);

        // Powers can be > 1 using above equations, so scale if they are
        double biggestFront = Math.max( Math.abs( frontLeftPower ), Math.abs( frontRightPower ) );
        double biggestRear = Math.max( Math.abs( rearLeftPower ), Math.abs( rearRightPower ) );
        double biggest = Math.max( biggestFront, biggestRear );

        if ( biggest > 1.0 )
        {
            frontLeftPower = frontLeftPower / biggest;
            frontRightPower = frontRightPower / biggest;
            rearLeftPower = rearLeftPower / biggest;
            rearRightPower = rearRightPower / biggest;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        rearLeft.setPower(rearLeftPower);
        rearRight.setPower(rearRightPower);
    }

    /*
     * Method to move the robot forward a specified distance. For use in auton modes.
     * Parameter is distance to move forward in centimeters.
     */
    public int AutonForward( double distance_cm )
    {
        /*
         This method just uses MoveDistanceWithRamp to move forward the
         distance requested.
         */
        int errorTicks = MoveDistanceWithRamp( distance_cm, DIRECTION.REVERSE);

        // Make sure we stop moving
        MoveSimple( 0.0, 0.0, 0.0 );

        return errorTicks;
    }

    /*
     * Method to move the robot backwards a specified distance. For use in auton modes.
     * Parameter is distance to move backwards in centimeters.
     */
    public int AutonReverse( double distance_cm )
    {
        /*
         This method just uses MoveDistanceWithRamp to move forward the
         distance requested.
         */
        int errorTicks = MoveDistanceWithRamp( distance_cm, DIRECTION.REVERSE);

        // Make sure we stop moving
        MoveSimple( 0.0, 0.0, 0.0 );

        return errorTicks;
    }


    /*
     * Method to rotate the robot clockwise a specified number of degrees. For use in auton modes.
     * Parameter is number of degrees to rotate clockwise.
     */
    public int AutonRotateClockwise( double degrees )
    {
        /*
         This method calculates how far to turn in cm to get the desired
         angle of rotation and then uses MoveDistanceWithRamp to
         actually move the robot the required amount.
         */
        double distanceToRotate_cm = degrees * CM_PER_DEGREE;

        int errorTicks = MoveDistanceFullPower( distanceToRotate_cm, DIRECTION.CLOCKWISE );

        // Make sure robot is not moving
        MoveSimple( 0.0, 0.0, 0.0 );

        return errorTicks;
    }


    /*
    * Method to rotate the robot counterclockwise a specified number of degrees. For use in auton modes.
    * Parameter is number of degrees to rotate counterclockwise.
    */
    public int AutonRotateCounterclockwise( double degrees )
    {
        /*
         This method calculates how far to turn in cm to get the desired
         angle of rotation and then uses MoveDistanceWithRamp to
         actually move the robot the required amount.
         */
        double distanceToRotate_cm = degrees * CM_PER_DEGREE;

        int errorTicks = MoveDistanceFullPower( distanceToRotate_cm, DIRECTION.COUNTERCLOCKWISE );

        // Make sure robot is not moving
        MoveSimple( 0.0, 0.0, 0.0 );

        return errorTicks;
    }

    /*
     * Method to move the robot right a specified distance. For use in auton modes.
     * Parameter is distance to move right in centimeters.
     */
    public int AutonRight( double distance_cm )
    {
        /*
         This method just uses MoveDistanceWithRamp to move right the
         distance requested.
         */
        int errorTicks = MoveDistanceFullPower( distance_cm, DIRECTION.RIGHT );

        // Make sure we stop moving
        MoveSimple( 0.0, 0.0, 0.0 );

        return errorTicks;
    }

    /*
     * Method to move the robot left a specified distance. For use in auton modes.
     * Parameter is distance to move left in centimeters.
     */
    public int AutonLeft( double distance_cm )
    {
        /*
         This method just uses MoveDistanceWithRamp to move left the
         distance requested.
         */
        int errorTicks = MoveDistanceFullPower( distance_cm, DIRECTION.LEFT );

        // Make sure we stop moving
        MoveSimple( 0.0, 0.0, 0.0 );

        return errorTicks;
    }

    /*
     * Method to move the robot a specified distance in a specified direction.
     * Parameters are distance to move in centimeters and direction
     */
    private int MoveDistanceWithRamp( double distance_cm, DIRECTION whichWay )
    {
        /*
         This algorithm calculates the total number encoder ticks we need to move for
         the given distance, and then runs the wheels until that distance is met.
         The direction to move is provided by the caller and the power is ramped from
         0 to 0.5 and then held there while still running using RampUpPower.
         */
        int encoderStart = frontLeft.getCurrentPosition();
        int totalTicks = (int) Math.round( distance_cm * TICKS_PER_CM );
        int ticksMoved = 0;

        limitTimer.reset();
        double powerNow = 0.0;
        while ( ( ticksMoved < ( totalTicks - MOMENTUM_BUFFER_TICKS ) ) && ( limitTimer.time() < MAX_SECONDS ) )
        {
            // apply power to wheels, select power based on distance left
            double powerTarget = TARGET_POWER_LONG;
            int ticksLeft = totalTicks - ticksMoved;
            if ( ticksLeft < ( AUTON_SHORT_DISTANCE_CM * TICKS_PER_CM ) )
            {
                powerTarget = TARGET_POWER_SHORT;
            }
            else if ( ticksLeft < ( AUTON_MEDIUM_DISTANCE_CM * TICKS_PER_CM ) )
            {
                powerTarget = TARGET_POWER_MEDIUM;
            }
            // Default else is taken care of by setting initial value to TARGET_POWER_LONG

            powerNow = PowerFilterAndSet( powerNow, powerTarget, whichWay );

            // Delay 20ms to control ramp rate
            Delay_ms( 20.0 );

            // calc how much we have moved
            int encoderNow = frontLeft.getCurrentPosition();
            ticksMoved = Math.abs( encoderNow - encoderStart );
        }

        // Stop moving
        MoveSimple( 0.0, 0.0, 0.0 );

        return ( totalTicks - ticksMoved );
    }

    /*
     * Method to move the robot a specified distance in a specified direction.
     * Parameters are distance to move in centimeters and direction and it always
     * uses full power. Most useful for rotation and sideways movement
     */
    private int MoveDistanceFullPower( double distance_cm, DIRECTION whichWay )
    {
        /*
         This algorithm calculates the total number encoder ticks we need to move for
         the given distance, and then runs the wheels until that distance is met.
         The direction to move is provided by the caller and the power is set using
         PowerFilterAndSet.
         */
        int encoderStart = frontLeft.getCurrentPosition();
        int totalTicks = (int) Math.round( distance_cm * TICKS_PER_CM );
        int ticksMoved = 0;

        limitTimer.reset();
        double powerNow = 0.0;
        while ( ( ticksMoved < totalTicks ) && ( limitTimer.time() < MAX_SECONDS ) )
        {
            // apply power to wheels, select power based on distance left
            int ticksLeft = totalTicks - ticksMoved;
            powerNow = PowerFilterAndSet( powerNow, TARGET_POWER_LONG, whichWay );

            // Delay 20ms to control ramp rate
            Delay_ms( 20.0 );

            // calc how much we have moved
            int encoderNow = frontLeft.getCurrentPosition();
            ticksMoved = Math.abs( encoderNow - encoderStart );
        }

        // Stop moving
        MoveSimple( 0.0, 0.0, 0.0 );

        return ( totalTicks - ticksMoved );
    }

    // Method to ramp up power in a given direction and then hold it
    private double PowerFilterAndSet( double powerNow, double targetPower, DIRECTION whichWay )
    {
        powerNow = JoystickUtilities.LowPassFilter( powerNow, targetPower, INCREASING_FK, DECREASING_FK );

        switch (whichWay)
        {
            case FORWARD:
                MoveSimple( 0.0, -powerNow, 0.0 );
                break;

            case REVERSE:
                MoveSimple( 0.0, powerNow , 0.0 );
                break;

            case RIGHT:
                MoveSimple( powerNow, 0.0, 0.0 );
                break;

            case LEFT:
                MoveSimple( ( -1.0 * powerNow ), 0.0, 0.0 );
                break;

            case CLOCKWISE:
                MoveSimple( 0.0, 0.0, -powerNow );
                break;

            case COUNTERCLOCKWISE:
                MoveSimple( 0.0, 0.0, powerNow  );
                break;
        }

        return powerNow;
    }

    private void Delay_ms( double milliseconds )
    {
        try {
            Thread.sleep( (long) ( milliseconds ) );
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }

}
