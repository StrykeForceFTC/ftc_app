package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by gstaats on 18/09/17.
 */

public class Drive
{
    // Matrix of drive motors
    //                                  LEFT   RIGHT
    private DcMotor[ ][ ] motors = { {  null,  null },    // FRONT_AXLE
                                     {  null,  null } };  // REAR_AXLE
    /*private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;*/

    // Limit timer
    private ElapsedTime limitTimer;
    private ElapsedTime delayTimer;

    // Constants for calculating number of ticks per cm to allow calculation of how many ticks
    // to go a given distance.
    final static double WHEEL_DIAM_IN = 4.0;
    final private static double IN_2_CM = 2.54;

    // wheel circumference is PI times diameter, but diameter needs to be converted to cm from inches
    final private static double WHEEL_CIRCUM_IN = Math.PI * WHEEL_DIAM_IN;
    final private static double TICKS_PER_REV = 1120.0;
    final private static double TICKS_PER_IN = TICKS_PER_REV / WHEEL_CIRCUM_IN;

    // Constants for rotating robot a certain number of degrees. The robot wheels are
    // set at the corners of a 14" square, so this is used to calculate the circumference of
    // the circle that the robot wheels rotatate about. This is used to determine a number of
    // ticks per degree.
    final private static double ROBOT_LENGTH_IN = 21.85;
    final private static double ROBOT_DIAM_IN = Math.sqrt( 2.0 * ( ROBOT_LENGTH_IN * ROBOT_LENGTH_IN ) ) ;
    final private static double ROBOT_CIRCUM_IN = ROBOT_DIAM_IN * Math.PI;
    final private static double IN_PER_DEGREE = ROBOT_CIRCUM_IN / 360.0;
    final private static double TIX_PER_DEGREE = IN_PER_DEGREE * TICKS_PER_IN;
    final private static double MAX_SECONDS = 4.5;
    final private static double AUTON_MEDIUM_DISTANCE_IN = 15.0;
    final private static double AUTON_SHORT_DISTANCE_IN = 8.0;
    final private static double TARGET_POWER_LONG = 0.4;
    final private static double TARGET_POWER_MEDIUM = 0.15;
    final private static double TARGET_POWER_SHORT = 0.075;
    final private static double INCREASING_FK = 0.3;                  // Increasing power filter constant
    final private static double DECREASING_FK = 0.8;                   // Decreasing power filter constant
    final private static int MOMENTUM_BUFFER_TICKS = 80;              // Stop ~1.5cm early to account for momentum
    final private static double AUTON_POWER = 0.5;

    public enum DIRECTION { FORWARD, REVERSE, RIGHT, LEFT }
    public enum ROTATION { CLOCKWISE, COUNTERCLOCKWISE }

    // Constants for indexing into matrices
    final private static int FRONT_AXLE = 0;
    final private static int REAR_AXLE = 1;
    final private static int NUM_AXLES = 2;
    final private static int LEFT = 0;
    final private static int RIGHT = 1;
    final private static int NUM_SIDES = 2;

    // Array for zero power
    final private static double[][] ZERO_POWER = { { 0, 0 }, { 0, 0 } };


    // Matrix for moving forward; to get reverse, multiply by -1.0
    //                                                      LEFT   RIGHT
    final private static double[ ][ ] FORWARD_MATRIX = { {  1.0,   1.0 },    // FRONT_AXLE
                                                         {  1.0,   1.0 } };  // REAR_AXLE

    //                                                      LEFT   RIGHT
    final private static double [ ][ ]BACKWARD_MATRIX = {{ -1.0,   -1.0},
                                                         { -1.0,   -1.0}};

    // Matrix for moving right; to get left, multiply by -1.0
    //                                                    LEFT   RIGHT
    final private static double[ ][ ] RIGHT_MATRIX = { { -1.0,   1.0 },    // FRONT_AXLE
                                                       {  1.0,  -1.0 } };  // REAR_AXLE

    final private static double[ ][ ] LEFT_MATRIX =  { { 1.0,   -1.0 },
                                                        {-1.0,   1.0 } };

    // Matrix for rotating clockwise; to get CCW, multiply by -1.0
    //                                                       LEFT   RIGHT
    final private static double[ ][ ] CLOCKWISE_MATRIX = { {  1.0,  -1.0 },    // FRONT_AXLE
                                                           {  1.0,  -1.0 } };  // REAR_AXLE

    final private static double[ ][ ] COUNTERCLOCKWISE_MATRIX =
                                                         { { -1.0,   1.0 },
                                                           { -1.0,   1.0 } };


    public Drive(DcMotor FL, DcMotor FR, DcMotor RL, DcMotor RR )
    {
        // Define and Initialize Motors
        motors[ FRONT_AXLE ][ RIGHT ] = FR;
        motors[ FRONT_AXLE ][ LEFT ]  = FL;
        motors[ REAR_AXLE ][ RIGHT ]  = RR;
        motors[ REAR_AXLE ][ LEFT ]  = RL;


        // Set direction so positive is always forward with respect to
        // the robot. Right side motors need to be set to reverse, because
        // they spin counter-clockwise to move the robot forward.
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            motors[ axle ][ LEFT ].setDirection( DcMotor.Direction.FORWARD );
            motors[ axle ][ RIGHT ].setDirection( DcMotor.Direction.REVERSE );
        }



        // Set all motors to zero power and reset encoders. Don't want robot moving till
        // we're ready.
        SetWheelPowers( ZERO_POWER );
        SetMotorModes( DcMotor.RunMode.STOP_AND_RESET_ENCODER );


        // Set all motors to run using encoders and not brake.
        SetMotorModes( DcMotor.RunMode.RUN_USING_ENCODER );
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                motors[ axle ][ side ].setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
            }
        }

        // Set up timers
        limitTimer = new ElapsedTime( ElapsedTime.Resolution.SECONDS );
        delayTimer = new ElapsedTime( ElapsedTime.Resolution.MILLISECONDS );
    }


    /*
     * Basic drive control method used by auton routines. Takes 3 input
     * values (FwdRev, LeftRight, rotate) and sets the motor powers
     * to give the desired motion.
     */
    public void MoveSimple( double xLeftRight, double yFwdRev, double rotate )
    {
        // variables for calculating wheel powers
        double[][] wheelPowers = { { 0, 0 }, { 0, 0 } };

        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                wheelPowers[ axle ][ side ] = xLeftRight * RIGHT_MATRIX[ axle ][ side ] +
                                              yFwdRev * FORWARD_MATRIX[ axle ][ side ] +
                                              rotate * CLOCKWISE_MATRIX[ axle ][ side ];
            }
        }

        // Powers can be > 1 using above equations, so scale if they are
        double biggestFront = Math.max( Math.abs( wheelPowers[ FRONT_AXLE ][ LEFT ] ), Math.abs( wheelPowers[ FRONT_AXLE ][ RIGHT ] ) );
        double biggestRear = Math.max( Math.abs( wheelPowers[ REAR_AXLE ][ LEFT ] ), Math.abs( wheelPowers[ REAR_AXLE ][ RIGHT ] ) );
        double biggest = Math.max( biggestFront, biggestRear );

        if ( biggest > 1.0 )
        {

            for ( int axle = 0; axle < NUM_AXLES; axle++ )
            {
                for ( int side = 0; side < NUM_SIDES; side++ )
                {
                    wheelPowers[ axle ][ side ] = wheelPowers[ axle ][ side ] / biggest;
                }
            }

        }

        SetWheelPowers( wheelPowers );

    }

    /*
     *  Method to set wheel powers
     */
    private void SetWheelPowers( double[][] powers )
    {
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                motors[ axle ][ side ].setPower( powers[ axle ][ side ] );
            }
        }
    }




    private void SetEncoderTargets( DIRECTION direction, double distance_in )
    {
        double ticks_2_move = distance_in * TICKS_PER_IN;

        double[][] move_matrix = null;

        // Pick the correct direction matrix to multiply distance by
        switch (direction)
        {
            case FORWARD:
            {
                move_matrix = FORWARD_MATRIX;
                break;
            }

            case REVERSE:
            {
                move_matrix = BACKWARD_MATRIX;
                break;
            }

            case RIGHT:
            {
                move_matrix = RIGHT_MATRIX;
                break;
            }

            default:
            {
                move_matrix = LEFT_MATRIX;
                break;
            }
        }

        // Loop through each wheel and calculate new position and set. Must read current position
        // and
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                int position = motors[ axle ][ side ].getCurrentPosition();
                motors[ axle ][ side ].setTargetPosition( position +
                        (int)( move_matrix[ axle ][ side ] * ticks_2_move ) );
            }
        }
    }

    private void SetEncoderTargetsRotate( ROTATION rotation, double distance_degrees )
    {
        double ticks_2_move = distance_degrees * TIX_PER_DEGREE;

        double[][] move_matrix = null;

        // Pick the correct direction matrix to multiply distance by
        switch (rotation)
        {
            case COUNTERCLOCKWISE:
            {
                move_matrix = COUNTERCLOCKWISE_MATRIX;
                break;
            }


            default:
            {
                move_matrix = COUNTERCLOCKWISE_MATRIX;
                break;
            }
        }

        // Loop through each wheel and calculate new position and set. Must read current position
        // and
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                int position = motors[ axle ][ side ].getCurrentPosition();
                motors[ axle ][ side ].setTargetPosition( position +
                        (int)( move_matrix[ axle ][ side ] * ticks_2_move ) );
            }
        }
    }
    private void SetMotorPowersAuton( DIRECTION direction )
    {
        double[][] move_matrix = null;
        double power = 0.0;

        // Pick the correct direction matrix to multiply distance by and power to use
        switch ( direction )
        {
            case FORWARD:
            {
                move_matrix = FORWARD_MATRIX;
                power = AUTON_POWER;
                break;
            }

            case REVERSE:
            {
                move_matrix = BACKWARD_MATRIX;
                power = AUTON_POWER;
                break;
            }

            case RIGHT:
            {
                move_matrix = RIGHT_MATRIX;
                power = AUTON_POWER;
                break;
            }

            default:
            {
                move_matrix = LEFT_MATRIX;
                power = AUTON_POWER;
                break;
            }
        }

        // Loop through each wheel and calculate power and set.
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                motors[ axle ][ side ].setPower( power * move_matrix[ axle ][ side ] );
            }
        }
    }
    private void SetMotorPowersAutonRotate( ROTATION rotation )
    {
        double[][] move_matrix = null;
        double power = 0.0;

        // Pick the correct direction matrix to multiply distance by and power to use
        switch ( rotation )
        {
            case COUNTERCLOCKWISE:
            {
                move_matrix = COUNTERCLOCKWISE_MATRIX;
                power = AUTON_POWER;
                break;
            }



            default:
            {
                move_matrix = CLOCKWISE_MATRIX;
                power = AUTON_POWER;
                break;
            }
        }

        // Loop through each wheel and calculate power and set.
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                motors[ axle ][ side ].setPower( power * move_matrix[ axle ][ side ] );
            }
        }
    }

    /*
     * Method to move the robot a specified distance in the given direction. For use in auton modes.
     * Parameters are direction and distance to move in inches.

     */

    public void AutonMove( Drive.DIRECTION direction, double distance_in )
    {
        // Set encoder values
        SetEncoderTargets( direction, distance_in );

        // Set mode to run to position
        SetMotorModes( DcMotor.RunMode.RUN_TO_POSITION );

        // Set power to make it move
        // motor powers = desired speed * direction matrix
        SetMotorPowersAuton( direction );

        // Wait for completion
        //! TODO: TO BE COMPLETED BY MAXWELL
        boolean notAtTarget = true;
        limitTimer.reset();

        while ( notAtTarget && ( limitTimer.seconds() < MAX_SECONDS ) )
        {

            //! If any motor finishes, we stop
            notAtTarget = motors[ FRONT_AXLE ][ LEFT ].isBusy() && motors[ FRONT_AXLE ][ RIGHT ].isBusy() &&
                          motors[ REAR_AXLE ][ LEFT ].isBusy() && motors[ REAR_AXLE ][ RIGHT ].isBusy();
        }

        // Done moving, so set power to 0 and mode back to use encoders
        SetWheelPowers( ZERO_POWER );
        SetMotorModes( DcMotor.RunMode.RUN_USING_ENCODER );
    }

    public void AutonMoveRotate( Drive.ROTATION rotation, double distance_degrees )
    {
        // Set encoder values
        SetEncoderTargetsRotate( rotation, distance_degrees );

        // Set mode to run to position
        SetMotorModes( DcMotor.RunMode.RUN_TO_POSITION );

        // Set power to make it move
        // motor powers = desired speed * direction matrix
        SetMotorPowersAutonRotate( rotation );

        // Wait for completion
        //! TODO: TO BE COMPLETED BY MAXWELL
        boolean notAtTarget = true;
        limitTimer.reset();

        while ( notAtTarget && ( limitTimer.seconds() < MAX_SECONDS ) )
        {

            //! If any motor finishes, we stop
            notAtTarget = motors[ FRONT_AXLE ][ LEFT ].isBusy() && motors[ FRONT_AXLE ][ RIGHT ].isBusy() &&
                    motors[ REAR_AXLE ][ LEFT ].isBusy() && motors[ REAR_AXLE ][ RIGHT ].isBusy();
        }

        // Done moving, so set power to 0 and mode back to use encoders
        SetWheelPowers( ZERO_POWER );
        SetMotorModes( DcMotor.RunMode.RUN_USING_ENCODER );
    }





    // Method to set mode of DC motors
    private void SetMotorModes( DcMotor.RunMode mode )
    {
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                motors[ axle ][ side ].setMode( mode );
            }
        }
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

        return 0;
    }

    public int AutonForward( double distance_cm )
    {
        /*
         This method just uses MoveDistanceWithRamp to move forward the
         distance requested.
         */

        return 0;
    }


    /*
     * Method to rotate the robot clockwise a specified number of degrees. For use in auton modes.
     * Parameter is number of degrees to rotate clockwise.
     */
    public int AutonRotateClockwise( double degrees )
    {

        return 0;
    }


    /*
    * Method to rotate the robot counterclockwise a specified number of degrees. For use in auton modes.
    * Parameter is number of degrees to rotate counterclockwise.
    */
    public int AutonRotateCounterclockwise( double degrees )
    {
        return 0;
    }

    /*
     * Method to move the robot right a specified distance. For use in auton modes.
     * Parameter is distance to move right in centimeters.
     */
    public int AutonRight( double distance_cm )
    {
        return 0;
    }


    public int AutonLeft( double distance_cm )
    {
        return 0;
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
