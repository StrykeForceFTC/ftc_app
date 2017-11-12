package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by gstaats on 18/09/17.
 */

public class Drive
{
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;
    HardwareMap hwMap = null;

    // Constants for calculating number of ticks per cm to allow calculation of how many ticks
    // to go a given distance.
    final private static double WHEEL_DIAM_IN = 4;
    final private static double IN_2_CM = 2.54;

    // wheel circumference is PI times diameter, but diameter needs to be converted to cm from inches
    final private static double WHEEL_CIRCUM_CM = Math.PI * WHEEL_DIAM_IN * IN_2_CM;
    final private static double TICKS_PER_REV = 1120.0;
    final private static double TICKS_PER_CM = TICKS_PER_REV / WHEEL_CIRCUM_CM;

    // Constants for rotating robot a certain number of degrees. The robot wheels are
    // set at the corners of a 14" square, so this is used to calculate the circumference of
    // the circle that the robot wheels rotatate about. This is used to determine a number of
    // ticks per degree.
    final private static double ROBOT_LENGTH_IN = 14.0;
    final private static double ROBOT_DIAM_CM = Math.sqrt( 2 * ( ROBOT_LENGTH_IN * ROBOT_LENGTH_IN ) ) * IN_2_CM;
    final private static double ROBOT_CIRCUM_CM = ROBOT_DIAM_CM * Math.PI;
    final private static double CM_PER_DEGREE = ROBOT_CIRCUM_CM / 360.0;

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
    }


    /*
     *
     */
    public void MoveSimple( double xLeftRight, double yFwdRev, double rotate )
    {
        // variables for calculating wheel powers
        double frontLeftPower = ( -xLeftRight ) + (   yFwdRev ) + (  rotate );
        double frontRightPower = (  xLeftRight ) + (  yFwdRev ) + (-rotate);
        double rearLeftPower = xLeftRight + yFwdRev + rotate;
        double rearRightPower = (-xLeftRight) + ( yFwdRev) + (-rotate);

        // Powers can be > 1 using above equations, so scale if they are
        double biggestFront = Math.max( Math.abs( frontLeftPower ), Math.abs( frontRightPower ) );
        double biggestRear = Math.max( Math.abs( rearLeftPower ), Math.abs( rearRightPower ) );
        double biggest = Math.max( biggestFront, biggestFront );

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
    public void AutonForward( double distance_cm )
    {
        /*
         This algorithm calculates the total number encoder ticks we need to move for
         the given distance, and then keeps running the wheels in forward and continually
         sums in the ticks moved to get total ticks moved. Once the objective is met, it sets
         wheel power back to 0. When the encoder rolls over (from ~1120 back to near 0), we account
         for this in an if statement that looks for rollover
         */
        int encoderLast = frontLeft.getCurrentPosition();
        int totalTicks = (int) ( distance_cm * TICKS_PER_CM );
        int ticksMoved = 0;

        while ( ticksMoved < totalTicks )
        {
            MoveSimple( 0.0, 0.5, 0.0 );
            int encoderNow = frontLeft.getCurrentPosition();
            if ( encoderNow > encoderLast )
            {
                ticksMoved += encoderNow - encoderLast;
            }
            else
            {
                // encoder must have rolled over, so calculate how far moved before and after
                // rolling over
                ticksMoved += encoderNow + ( TICKS_PER_REV - encoderLast );
            }

            // Update last position for next loop
            encoderLast = encoderNow;
        }

        // Stop moving
        MoveSimple( 0.0, 0.0, 0.0 );
    }

    /*
     * Method to move the robot backwards a specified distance. For use in auton modes.
     * Parameter is distance to move backwards in centimeters.
     */
    public void AutonReverse( double distance_cm )
    {
        /*
         This algorithm calculates the total number encoder ticks we need to move for
         the given distance, and then keeps running the wheels in reverse and continually
         sums in the ticks moved to get total ticks moved. Once the objective is met, it sets
         wheel power back to 0. When the encoder under flows (from 0 to near 1120), we account
         for this in an if statement that looks for underflow
         */
        int encoderLast = frontLeft.getCurrentPosition();
        int totalTicks = (int) ( distance_cm * TICKS_PER_CM );
        int ticksMoved = 0;

        while ( ticksMoved < totalTicks )
        {
            MoveSimple( 0.0, -0.5, 0.0 );
            int encoderNow = frontLeft.getCurrentPosition();
            if ( encoderNow < encoderLast )
            {
                ticksMoved += encoderLast - encoderNow;
            }
            else
            {
                // encoder must have rolled over, so calculate how far moved before and after
                // rolling over
                ticksMoved += encoderLast + ( TICKS_PER_REV - encoderNow );
            }

            // Update last position for next loop
            encoderLast = encoderNow;
        }

        // Stop moving
        MoveSimple( 0.0, 0.0, 0.0 );
    }


    /*
     * Method to rotate the robot clockwise a specified number of degrees. For use in auton modes.
     * Parameter is number of degrees to rotate clockwise.
     */
    public void AutonRotateClockwise( double degrees )
    {
        /*
         This algorithm calculates the number encoder ticks we need to rotate a wheel to
         rotate the requested angle. Then the robot is continually run in the correct rotation
         until the ticks value is met. Once the objective is met, it sets
         wheel power back to 0. When the encoder rolls over (from ~1120 back to near 0), we account
         for this in an if statement that looks for rollover
         */
        int encoderLast = frontLeft.getCurrentPosition();
        int totalTicks = (int) ( degrees * CM_PER_DEGREE * TICKS_PER_CM );
        int ticksMoved = 0;

        while ( ticksMoved < totalTicks )
        {
            MoveSimple( 0.0, 0.0, 0.5 );
            int encoderNow = frontLeft.getCurrentPosition();
            if ( encoderNow > encoderLast )
            {
                ticksMoved += encoderNow - encoderLast;
            }
            else
            {
                // encoder must have rolled over, so calculate how far moved before and after
                // rolling over
                ticksMoved += encoderNow + ( TICKS_PER_REV - encoderLast );
            }

            // Update last position for next loop
            encoderLast = encoderNow;
        }

        // Stop moving
        MoveSimple( 0.0, 0.0, 0.0 );
    }


    /*
    * Method to rotate the robot counterclockwise a specified number of degrees. For use in auton modes.
    * Parameter is number of degrees to rotate counterclockwise.
    */
    public void AutonRotateCounterclockwise( double degrees )
    {
        /*
         This algorithm calculates the number encoder ticks we need to rotate a wheel to
         rotate the requested angle. Then the robot is continually run in the correct rotation
         until the ticks value is met. Once the objective is met, it sets
         wheel power back to 0. When the encoder rolls over (from ~1120 back to near 0), we account
         for this in an if statement that looks for rollover
         */
        int encoderLast = frontRight.getCurrentPosition();
        int totalTicks = (int) ( degrees * CM_PER_DEGREE * TICKS_PER_CM );
        int ticksMoved = 0;

        while ( ticksMoved < totalTicks )
        {
            MoveSimple( 0.0, 0.0, 0.5 );
            int encoderNow = frontRight.getCurrentPosition();
            if ( encoderNow > encoderLast )
            {
                ticksMoved += encoderNow - encoderLast;
            }
            else
            {
                // encoder must have rolled over, so calculate how far moved before and after
                // rolling over
                ticksMoved += encoderNow + ( TICKS_PER_REV - encoderLast );
            }

            // Update last position for next loop
            encoderLast = encoderNow;
        }

        // Stop moving
        MoveSimple( 0.0, 0.0, 0.0 );
    }


}
