package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 *
 * Auton Program mainly here to test Auton Drive and other Functions. Also
 * configured to be used as a "short" auton on crater side that just samples
 * and then parks on crater in front of robot.
 *
 */
@Autonomous(name = "Auton_Testcode", group = "Linear Opmode")
//@Disabled
public class Auton_Testcode extends AutonLinearBase
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Hardware objects are created in AutonLinearBase
    // Auton steps are enumerated in AutonLinerBase

    // Constants for drive to depot. These are not common with
    // the facing depot auton, so the values are set up here.
    private static double DRIVE_DEPOT_TURN_2_WALL_DEG = 75.0;
    private static double DRIVE_DEPOT_MOVE_2_WALL_IN = 37.0;
    private static double DRIVE_DEPOT_ROT_PARALLEL_2_WALL_DEG = 27.5;
    private static double DRIVE_DEPOT_STRAFE_2_WALL_IN = 11.0;
    private static double DRIVE_DEPOT_FWD_2_DEPOT = 32.5;

    /*
     * There is only runOpMode for linear op modes
     */
    @Override
    public void runOpMode()
    {
        /* Initialize the hardware variables.
         */
        InitHardware();

        // Tune parameters based on robot as needed
        // these values are copied in from facing crater
        switch ( TeamId )
        {
            case team7228:
            {

                break;
            }

            case team8553:
            {
                break;
            }

            case team15106:
            {
                break;
            }
        }

        double zAngle = gyro.GetZAngle();
        telemetry.addData("Mode ", "waiting for start");
        telemetry.addData( " Z: ", zAngle );
        telemetry.update();

        // Wait hit till start button pressed
        waitForStart();
        runtime.reset();

        // Set state you want to start in here
        step = AUTON_STEPS.RELEASE_LANDER;

        // simple states for stepping through release
        double rotateDeg = 5.0;
        double driveIn = 2.0;
        double strafeIn = 2.0;

        // Ensure gold position is unknown for testing
        // release, find gold and move to mineral
        gold = GOLD_POSITIONS.UNKNOWN_POS;

        // Loop until stop or forced to end
        while ( opModeIsActive( ) )
        {
/* Removing gamepad controls from auton test in case it gets used
            if ( gamepad1.dpad_up )
            {
                driveIn += 1.0;

                while ( gamepad1.dpad_up )
                {

                }
            }
            else if ( gamepad1.dpad_down )
            {
                if ( driveIn >= 2.0 )
                {
                    driveIn -= 1.0;
                }
                while ( gamepad1.dpad_down )
                {

                }
            }

            if ( gamepad1.dpad_right )
            {
                if ( rotateDeg <= 215.0 )
                {
                    rotateDeg += 5.0;
                }
                while ( gamepad1.dpad_right )
                {

                }

            }
            else if ( gamepad1.dpad_left )
            {
                if ( rotateDeg >= 10.0 )
                {
                    rotateDeg -= 5.0;
                }
                while ( gamepad1.dpad_left )
                {

                }
            }

            if ( gamepad2.dpad_right )
            {
                strafeIn += 1.0;

                while ( gamepad2.dpad_right )
                {

                }

            }
            else if ( gamepad2.dpad_left )
            {
                if ( strafeIn >= 2.0 )
                {
                    strafeIn -= 1.0;
                }
                while ( gamepad2.dpad_left )
                {

                }
            }

            if ( gamepad1.a )
            {
                go.AutonMoveRotate( Drive.ROTATION.CLOCKWISE, rotateDeg );
            }
            else if ( gamepad1.b )
            {
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, rotateDeg );
            }

            if ( gamepad1.x )
            {
                go.AutonMove( Drive.DIRECTION.FORWARD, driveIn );
            }
            else if ( gamepad1.y )
            {
                go.AutonMove( Drive.DIRECTION.REVERSE, driveIn );
            }

            if ( gamepad2.dpad_up )
            {
                // Lower the bot
                arm.position_lift( Arm.lift_pos.hook_lander, LIFT_SPEED );
                arm.WaitForInPos();
            }
            else if ( gamepad2.dpad_down )
            {
                // Raise the bot
                arm.position_lift( Arm.lift_pos.ZERO, LIFT_SPEED );
                arm.WaitForInPos();
            }

            if ( gamepad2.a )
            {
                go.AutonMove( Drive.DIRECTION.RIGHT, strafeIn );
            }
            else if ( gamepad2.b )
            {
                go.AutonMove( Drive.DIRECTION.LEFT, strafeIn );
            }
*/

            //Auton steps
            switch( step )
            {
                case RELEASE_LANDER:
                {
                    // Run common method from AutonLinearBase and
                    // go to next step
                    ReleaseLander();
                    step = step.Next();
                    break;
                }

                case FIND_GOLD:
                {
                    // Use common method to find gold, commented out until
                    // it can be fully developed / tuned.
                    FindGold();

                    // Move to next step
                    step = step.Next();
                    break;
                }

                case MOVE_TO_MINERAL:
                {
                    // Sample gold mineral
                    GoToGold();
                    step = step.PARK;
                    break;
                }

                case LOAD_GOLD:
                {
                    // Can't load gold at this time
                    //LoadGold();
                    // step = step.Next();
                    break;
                }

                case PARK:
                {
                    // Park the robot on crater
                    ParkTheRobot();
                    step = step.Next();
                    break;
                }

                case STOP:
                {
                    // Auton is complete.
                    StopActions();
                    break;
                }

                default:
                    break;
            }

            zAngle = gyro.GetZAngle();
            telemetry.addData( " Z: ", zAngle );
/* Remove unneeded telemetry to avoid confusion
            telemetry.addData( " Drive inches: ", driveIn )
                     .addData( " Rotate deg: ", rotateDeg )
                     .addData( " Strafe In: ", strafeIn );
            telemetry.addLine( "Gamepad1 Dpad: Up: +1 drive, Down: -1 drive, Right: +5deg, Left: -5deg");
            telemetry.addLine( "Gamepad1 Buttons: A: Rotate CW, B: Rotate CCW, X: Drive FWD, Y: Drive REV" );
            telemetry.addLine( "Gamepad2 Dpad: Up: Lower robot, Down: Raise robot, Right: +1 Strafe, Left: -1 Strafe" );
            telemetry.addLine( "Gamepad2 Buttons: A: Strafe right, B: Strafe Left" );
*/
            AddStdAutonTelemetry( true );

            telemetry.update();

            idle();
        }

    }

    @Override
    protected void DriveToDepot( )
    {
        // To go to the depot, rotate the robot to drive towards the wall, drive
        // to close to the wall, rotate to make robot parallel to the wall, and
        // then strafe to wall and finally go forward to crater.
        go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_TURN_2_WALL_DEG );
        go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_MOVE_2_WALL_IN );
        go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_ROT_PARALLEL_2_WALL_DEG );
        go.AutonMove( Drive.DIRECTION.RIGHT, DRIVE_DEPOT_STRAFE_2_WALL_IN );
        go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_2_DEPOT );

    }

    // Method to get to park position - hard coded to attempt to park on crater in front of robot
    // after sampling crater side
    @Override
    protected void ParkTheRobot( )
    {
        // At end of going to gold, wrist starts moving to unload, so use gold position
        // to determine how to move to crater
        switch ( gold )
        {
            case LEFT_POS:
            {
                // For left, wait for wrist to finish and then drive forward
                arm.WaitForInPos();
                go.AutonMove( Drive.DIRECTION.FORWARD, 10.0 );
                break;
            }

            case MID_POS:
            case RIGHT_POS:
            {
                // rotate CCW 15deg and then move fwd 10" to park, remember, wrist is
                // moving to load during this. Right & mid are same.
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, 15.0 );
                go.AutonMove( Drive.DIRECTION.FORWARD, 10.0 );
                break;
            }

        }

        // Position wrist out front to give best chance of being over crater
        arm.position_wrist( Arm.WRIST_POS.MOVE, WRIST_SPEED );
        arm.WaitForInPos();
    }



}


