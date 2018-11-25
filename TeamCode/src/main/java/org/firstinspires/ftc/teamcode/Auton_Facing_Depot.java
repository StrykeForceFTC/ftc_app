package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 *
 * Auton Program mainly here to test Auton Drive and other Functions in
 *
 */
@Autonomous( name = "Facing Depot", group = "Linear Opmode" )
//@Disabled
public class Auton_Facing_Depot extends AutonLinearBase
{
    // Hardware objects are created in AutonLinearBase
    // Enumeration for auton steps is in AutonLinearBase

    // Constants for drive to depot. These are not common with
    // the facing depot auton, so the values are set up here.
    private double DRIVE_DEPOT_TURN_2_WALL_DEG = 65.0;
    private double DRIVE_DEPOT_MOVE_2_WALL_IN = 40.0;
    private double DRIVE_DEPOT_ROT_PARALLEL_2_WALL_DEG = 60;
    private double DRIVE_DEPOT_STRAFE_2_WALL_IN = 13.0;
    private double DRIVE_DEPOT_FWD_2_DEPOT = 26;    // 33
    private double ROTATE_AFTER_DROP = 6;

    /*
     * There is only runOpMode for linear op modes
     */
    @Override
    public void runOpMode()
    {
        /*
         * Initialize the hardware variables.
         */
        InitHardware();

        // Override values for various auton movements - if you want
        // to change from values in AutonLinearBase, then delete the "//"
        // and set to the desired value.
        PARK_DISTANCE_IN = 49;

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



        // Wait hit till start button pressed
        waitForStart();
        runtime.reset();

        // Start at move to mineral, for now
        //step = AUTON_STEPS.MOVE_TO_MINERAL;

        // Loop until stop or forced to end
        while ( opModeIsActive() )
        {
            // Auton steps
            switch( step )
            {
                case RELEASE_LANDER:
                {
                    // Run common method from AutonLinearBase and
                    // go to next step
                    ReleaseLander();
                    step = AUTON_STEPS.MOVE_TO_MINERAL;
                    break;
                }

                case FIND_GOLD:
                {
                    // Use common method to find gold, commented out until
                    // it can be fully developed / tuned.
                    // FindGold();

                    // Display mineral position on phone
                    telemetry.addLine().addData( "GP: ", gold.toString() );

                    // Move to next step
                    step = step.Next();
                    break;
                }

                case MOVE_TO_MINERAL:
                {
                    // Move in front of gold mineral
                    GoToGold();
                    step = step.Next();
                    break;
                }

                case LOAD_GOLD:
                {
                    LoadGold();
                    step = step.Next();
                    break;
                }

                case DRIVE_DEPOT:
                {
                    DriveToDepot( );
                    step = step.Next();
                    break;
                }

                case UNLOAD:
                {
                    // Drop marker and gold sample
                    UnloadGoldAndMarker();
                    step = step.Next();
                    break;
                }

                case PARK:
                {
                    ParkTheRobot();
                    step = step.Next();
                    break;
                }

                case STOP:
                {
                    StopActions();
                    break;
                }

            }

            telemetry.addData("IsAligned", GoldAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos", GoldXPosition());    // Gold X pos.

            AddStdAutonTelemetry(true);
            telemetry.update();

            idle();
        }

    }

    @Override
    protected void DriveToDepot( )
    {
        // To go to the depot, rotate the robot to drive towards the wall, drive
        // close to the wall, rotate to make robot parallel to the wall,
        // then strafe to wall and finally go forward to depot.
        go.AutonMoveRotate( Drive.ROTATION.CLOCKWISE, DRIVE_DEPOT_TURN_2_WALL_DEG );
        go.AutonMove( Drive.DIRECTION.REVERSE, DRIVE_DEPOT_MOVE_2_WALL_IN );
        go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_ROT_PARALLEL_2_WALL_DEG );
        arm.position_wrist( Arm.WRIST_POS.MOVE, WRIST_SPEED );
        go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_2_DEPOT );

    }

    @Override
    protected void UnloadGoldAndMarker( )
    {
        // arm should already be in correct position

        // Unload gold sample
        loader.AutonUnload();
        go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, ROTATE_AFTER_DROP );
    }


}


