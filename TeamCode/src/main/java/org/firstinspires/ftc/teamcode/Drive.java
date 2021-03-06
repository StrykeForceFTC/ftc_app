package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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


    // Limit timer
    private ElapsedTime limitTimer;

    // Constants for calculating number of ticks per cm to allow calculation of how many ticks
    // to go a given distance.
    final private static double WHEEL_DIAM_IN = 4.0;
    final private static double IN_2_CM = 2.54;

    // wheel circumference is PI times diameter, but diameter needs to be converted to cm from inches
    final private static double WHEEL_CIRCUM_IN = Math.PI * WHEEL_DIAM_IN;
    final private static double TICKS_PER_REV = 1120.0;
    final private static double TICKS_PER_IN = TICKS_PER_REV / WHEEL_CIRCUM_IN;
    final private static double TICKS_LEFT_FOR_SLOWDOWN = TICKS_PER_IN * 4.0;

    final private static double ROBOT_LENGTH = 13.75;
    final private static double ROBOT_WIDTH = 16;

    // Constants for rotating robot a certain number of degrees. The robot wheels are
    // set at the corners of a 14" square, so this is used to calculate the circumference of
    // the circle that the robot wheels rotatate about. This is used to determine a number of
    // ticks per degree.
    final private static double ROBOT_DIAM_IN = Math.sqrt((ROBOT_WIDTH *ROBOT_WIDTH ) + ( ROBOT_LENGTH * ROBOT_LENGTH));
    final private static double ROBOT_CIRCUM_IN = ROBOT_DIAM_IN * Math.PI;
    final private static double IN_PER_DEGREE = ROBOT_CIRCUM_IN / 360.0;
    final private static double TIX_PER_DEGREE = IN_PER_DEGREE * TICKS_PER_IN;
    final private static double TICKS_LEFT_FOR_SLOWDOWN_ROTATE = TIX_PER_DEGREE * 20.0;
    final private static double MAX_SECONDS = 7.0;

    // Constants for power in auton
    final private static double[] AUTON_POWER = {
                                                    0.9,   // Forward
                                                    0.9,   // Reverse
                                                    0.9,   // Right
                                                    0.9    // Left
                                                };

    final private static double[] AUTON_POWER_ROTATE = {
                                                            0.8,   // Clockwise
                                                            0.8,   // Counterclockwise
                                                       };

    final private static double AUTON_POWER_FINAL = 0.4;

    // Time delay while waiting for wheels to get to targets in auton
    final private static double DELAY_DURING_GO_TO_POSITION_ms = 40.0;

    // Short delay after one wheel has gotten to position
    final private static double DELAY_FOR_REST_OF_WHEELS_ms = 100.0;

    //these are enums that are to be given numerical values later so that we don't have to write in the matrices all the time.
    //These enums are also easier to change for debugging (The values assigned to them.)
    public enum DIRECTION { FORWARD, REVERSE, RIGHT, LEFT }
    public enum ROTATION { CLOCKWISE, COUNTERCLOCKWISE }

    // Constants for indexing into matrices
    final private static int FRONT_AXLE = 0;
    final private static int REAR_AXLE = 1;
    final private static int NUM_AXLES = 2;
    final private static int LEFT = 0;
    final private static int RIGHT = 1;
    final private static int NUM_SIDES = 2;

    // adjustment for moving left and right in auton
    final private static double LEFT_RIGHT_ADJUSTMENT = 1.2;
    final private static double ROTATE_ADJUSTMENT = 1.46;

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
    final private static double[ ][ ] RIGHT_MATRIX = { { 1.0,   -1.0 },    // FRONT_AXLE
                                                       {  -1.0,  1.0 } };  // REAR_AXLE

    final private static double[ ][ ] LEFT_MATRIX =  { { -1.0,   1.0 },
                                                        {1.0,   -1.0 } };

    // Matrix for rotating clockwise; to get CCW, multiply by -1.0
    //                                                       LEFT   RIGHT
    final private static double[ ][ ] CLOCKWISE_MATRIX = { {  1.0,  -1.0 },    // FRONT_AXLE
                                                           {  1.0,  -1.0 } };  // REAR_AXLE

    final private static double[ ][ ] COUNTERCLOCKWISE_MATRIX =
                                                         { { -1.0,   1.0 },
                                                           { -1.0,   1.0 } };


    public Drive( HardwareMap ahwMap, boolean autonMode )
    {
        // Define and Initialize Motors
        motors[ FRONT_AXLE ][ RIGHT ] = ahwMap.dcMotor.get("front_right");
        motors[ FRONT_AXLE ][ LEFT ]  = ahwMap.dcMotor.get("front_left");
        motors[ REAR_AXLE ][ RIGHT ]  = ahwMap.dcMotor.get("rear_right");
        motors[ REAR_AXLE ][ LEFT ]   = ahwMap.dcMotor.get("rear_left");


        // Set direction so positive is always forward with respect to
        // the robot. Right side motors need to be set to reverse, because
        // they spin counter-clockwise to move the robot forward.
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            motors[ axle ][ LEFT ].setDirection( DcMotor.Direction.REVERSE );
            motors[ axle ][ RIGHT ].setDirection( DcMotor.Direction.FORWARD );
        }



        // Set all motors to zero power and reset encoders. Don't want robot moving till
        // we're ready.
        SetWheelPowers( ZERO_POWER );
        SetMotorModes( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        // In auton modes, set motor mode to run to position and in teleop modes
        // set to run using encoder
        if ( autonMode )
        {
            SetMotorModes( DcMotor.RunMode.RUN_TO_POSITION );
        }
        else
        {
            SetMotorModes( DcMotor.RunMode.RUN_USING_ENCODER );
        }

        // Set all motors to not brake.
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                motors[ axle ][ side ].setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
            }
        }

        // Set up timers
        limitTimer = new ElapsedTime( ElapsedTime.Resolution.SECONDS );
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

    // These are doubles used for methods that ask for the wheel powers of the motors.
    public double GetWheelPowerFrontLeft( )
    {
        return motors[ FRONT_AXLE ][ LEFT ].getPower();
    }

    public double GetWheelPowerFrontRight( )
    {
        return motors[ FRONT_AXLE ][ RIGHT ].getPower();
    }

    public double GetWheelPowerRearLeft( )
    {
        return motors[ REAR_AXLE ][ LEFT ].getPower();
    }

    public double GetWheelPowerRearRight( )
    {
        return motors[ REAR_AXLE ][ RIGHT ].getPower();
    }

    public int GetEncoderFrontLeft( )
    {
        return motors[ FRONT_AXLE ][ LEFT ].getCurrentPosition();
    }

    public int GetEncoderFrontRight( )
    {
        return motors[ FRONT_AXLE ][ RIGHT ].getCurrentPosition();
    }

    public int GetEncoderRearLeft( )
    {
        return motors[ REAR_AXLE ][ LEFT ].getCurrentPosition();
    }

    public int GetEncoderRearRight( )
    {
        return motors[ REAR_AXLE ][ RIGHT ].getCurrentPosition();
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


    // Method to set encoder targets for auton fwd, rev, left and right
    private int SetEncoderTargets( DIRECTION direction, double distance_in )
    {
        double ticks_2_move = distance_in * TICKS_PER_IN;
        double[][] move_matrix;

        // Adjust distance for left-right
        if ( ( direction == DIRECTION.LEFT ) || ( direction == DIRECTION.RIGHT ) )
        {
            ticks_2_move = LEFT_RIGHT_ADJUSTMENT * ticks_2_move;
        }

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
                        (int)( ( move_matrix[ axle ][ side ] * ticks_2_move ) + 0.5 ) );
            }
        }

        return (int)ticks_2_move;
    }

    // this method calculates the main diameter of the robot and multiply the degrees to turn by
    // based on which direction was chosen when the method was called.
    private int SetEncoderTargetsRotate( ROTATION rotation, double distance_degrees )
    {
        double ticks_2_move = distance_degrees * TIX_PER_DEGREE * ROTATE_ADJUSTMENT;
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
                move_matrix = CLOCKWISE_MATRIX;
                break;
            }
        }

        // Loop through each wheel and calculate new position and set. Must read current position
        // and add the calculated encoder values.
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                int position = motors[ axle ][ side ].getCurrentPosition();
                motors[ axle ][ side ].setTargetPosition( position +
                        (int)( ( move_matrix[ axle ][ side ] * ticks_2_move ) + 0.5 ) );
            }
        }

        return (int)ticks_2_move;
    }

    //This method picks a direction set from calling direction.  then, it calculates power and sets the motors.
    private void SetMotorPowersAuton( DIRECTION direction, double power )
    {
        double[][] move_matrix = null;

        // Pick the correct direction matrix to multiply distance by and power to use
        switch ( direction )
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

        // Loop through each wheel and calculate power and set.
        for ( int axle = 0; axle < NUM_AXLES; axle++ )
        {
            for ( int side = 0; side < NUM_SIDES; side++ )
            {
                motors[ axle ][ side ].setPower( power * move_matrix[ axle ][ side ] );
            }
        }
    }

    //This method does the same, but with rotate to protect from encoder equation mess-ups.
    private void SetMotorPowersAutonRotate( ROTATION rotation, double power )
    {
        double[][] move_matrix = null;

        // Pick the correct direction matrix to multiply distance by and power to use
        switch ( rotation )
        {
            case COUNTERCLOCKWISE:
            {
                move_matrix = COUNTERCLOCKWISE_MATRIX;
                break;
            }



            default:
            {
                move_matrix = CLOCKWISE_MATRIX;
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
        // First set motor powers to 0 prior to setting targets
        SetWheelPowers( ZERO_POWER );

        // Set encoder values, store ticks to move and grab initial value
        // for front right
        int ticks_2_move = SetEncoderTargets( direction, distance_in );
        int front_right_start = motors[ FRONT_AXLE ][ RIGHT ].getCurrentPosition();

        // Set mode to run to position - set in init function and not changed by auton modes
        // SetMotorModes( DcMotor.RunMode.RUN_TO_POSITION );

        // Set power to make it move
        // motor powers = desired speed * direction matrix
        SetMotorPowersAuton( direction, AUTON_POWER[ direction.ordinal() ] );

        boolean notAtTarget = true;
        limitTimer.reset();
        boolean power_reduced = false;
        double ticks_4_slowdown = TICKS_LEFT_FOR_SLOWDOWN;
        if ( distance_in > 12.0 )
        {
            ticks_4_slowdown = 2.0  * ticks_4_slowdown;
        }

        // Loop while waiting for motors to get to target, and set a timeout
        while ( notAtTarget && ( limitTimer.seconds() < MAX_SECONDS ) )
        {
            // Short delay
            Delay_ms( DELAY_DURING_GO_TO_POSITION_ms );

            // Allow other processes to run; same as idle() in linear op mode
            Thread.yield();

            // When we get close to objective, slow down to lower speed
            int delta = Math.abs( motors[ FRONT_AXLE ][ RIGHT ].getCurrentPosition() - front_right_start );
            int ticks_left = Math.abs( ticks_2_move - delta );
            if ( !power_reduced && ( ticks_left <= ticks_4_slowdown ) )
            {
                power_reduced = true;
                SetMotorPowersAuton( direction, AUTON_POWER_FINAL );
            }

            //! If any motor finishes, we stop
            notAtTarget = motors[ FRONT_AXLE ][ LEFT ].isBusy() && motors[ FRONT_AXLE ][ RIGHT ].isBusy() &&
                          motors[ REAR_AXLE ][ LEFT ].isBusy() && motors[ REAR_AXLE ][ RIGHT ].isBusy();
        }

        // Short delay to wait for motors to finish
        Delay_ms( DELAY_FOR_REST_OF_WHEELS_ms );

        // Leave power set to ensure all encoders get to correct final position
        // SetWheelPowers( ZERO_POWER );
        // SetMotorModes( DcMotor.RunMode.RUN_USING_ENCODER );
    }

    //This method calls set encoder targets rotate and then sets motors to run-to-position.
    //Then, it sets powers and goes.
    public void AutonMoveRotate( Drive.ROTATION rotation, double distance_degrees )
    {
        // First set motor powers to 0 prior to setting targets
        SetWheelPowers( ZERO_POWER );

        // Set encoder values
        int ticks_2_rotate = SetEncoderTargetsRotate( rotation, distance_degrees );
        int front_right_start = motors[ FRONT_AXLE ][ RIGHT ].getCurrentPosition();

        // Set mode to run to position - no longer needed, done in init
        // SetMotorModes( DcMotor.RunMode.RUN_TO_POSITION );

        // Set power to make it move
        // motor powers = desired speed * direction matrix
        SetMotorPowersAutonRotate( rotation, AUTON_POWER_ROTATE[ rotation.ordinal() ] );

        // Wait for completion
        boolean notAtTarget = true;
        limitTimer.reset();
        boolean power_reduced = false;

        while ( notAtTarget && ( limitTimer.seconds() < MAX_SECONDS ) )
        {
            // Short delay
            Delay_ms( DELAY_DURING_GO_TO_POSITION_ms );

            // Allow other processes to run; same as idle() in linear op mode
            Thread.yield();

            // When we get close to objective, slow down to lower speed
            int delta = Math.abs( motors[ FRONT_AXLE ][ RIGHT ].getCurrentPosition() - front_right_start );
            int ticks_left = Math.abs( ticks_2_rotate - delta );
            if ( !power_reduced && ( ticks_left < TICKS_LEFT_FOR_SLOWDOWN_ROTATE ) )
            {
                power_reduced = true;
                SetMotorPowersAutonRotate( rotation, AUTON_POWER_FINAL );
            }

            //! If any motor finishes, we stop
            notAtTarget = motors[ FRONT_AXLE ][ LEFT ].isBusy() && motors[ FRONT_AXLE ][ RIGHT ].isBusy() &&
                    motors[ REAR_AXLE ][ LEFT ].isBusy() && motors[ REAR_AXLE ][ RIGHT ].isBusy();
        }

        // Short delay to wait for motors to finish
        Delay_ms( DELAY_FOR_REST_OF_WHEELS_ms );

        // Leave power set and in run to position
        // Done moving, so set power to 0 and mode back to use encoders
        // SetWheelPowers( ZERO_POWER );
        // SetMotorModes( DcMotor.RunMode.RUN_USING_ENCODER );
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
     * Method to delay a specified amount of time; not really
     * needed anymore.
     */
    private void Delay_ms( double milliseconds )
    {
        try {
            Thread.sleep( (long) ( milliseconds ) );
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }

}
