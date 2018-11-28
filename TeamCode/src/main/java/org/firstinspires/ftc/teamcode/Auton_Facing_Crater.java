package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 *
 * Auton Program mainly here to test Auton Drive and other Functions in
 *
 */
@Autonomous( name = "Facing Crater", group = "Linear Opmode" )
//@Disabled
public class Auton_Facing_Crater extends AutonLinearBase
{
    // Hardware objects are created in AutonLinearBase
    // Enumeration for auton steps is in AutonLinearBase

    // Constants for drive to depot. These are not common with
    // the facing depot auton, so the values are set up here.
    private double DRIVE_DEPOT_TURN_2_WALL_LEFT_DEG = 111.0;
    private double DRIVE_DEPOT_TURN_2_WALL_MID_DEG = 111.0;
    private double DRIVE_DEPOT_TURN_2_WALL_RIGHT_DEG = 111.0;

    private double DRIVE_DEPOT_MOVE_2_WALL_LEFT_IN = 43.5;
    private double DRIVE_DEPOT_MOVE_2_WALL_MID_IN = 43.5;
    private double DRIVE_DEPOT_MOVE_2_WALL_RIGHT_IN = 43.5;

    private double DRIVE_DEPOT_ROT_PARALLEL_2_WALL_LEFT_DEG = 43;
    private double DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG = 43;
    private double DRIVE_DEPOT_ROT_PARALLEL_2_WALL_RIGHT_DEG = 43;

    private double DRIVE_DEPOT_FWD_2_DEPOT_LEFT_IN = 23;
    private double DRIVE_DEPOT_FWD_2_DEPOT_MID_IN = 23;
    private double DRIVE_DEPOT_FWD_2_DEPOT_RIGHT_IN = 23;

    private double DRIVE_DEPOT_STRAFE_2_WALL_IN = 12.0;

    private double ROTATE_AFTER_DROP_DEG = 8;

    /*
     * There is only runOpMode for linear op modes
     */
    @Override
    public void runOpMode()
    {
        // This line to disables gold mineral position detection - assuming middle
        gold = GOLD_POSITIONS.UNKNOWN_POS;

        /*
         * Initialize the hardware variables.
         */
        InitHardware();

        // Override values for various auton movements - if you want
        // to change from values in AutonLinearBase, then delete the "//"
        // and set to the desired value.
        // PARK_DISTANCE_IN = 75.0;

        // Robot-specific overrides
        switch ( TeamId )
        {
            case team7228:
            {
                DRIVE_DEPOT_TURN_2_WALL_MID_DEG = 90;
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG = 20;
                ROTATE_AFTER_DROP_DEG = 7;
                DRIVE_DEPOT_MOVE_2_WALL_MID_IN = 45;
                DRIVE_DEPOT_FWD_2_DEPOT_MID_IN = 26;
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
                    // Release from lander and rotate to face minerals with arm positioned to sample
                    ReleaseLander();
                    step = step.Next();
                    break;
                }

                case FIND_GOLD:
                {
                    // Use common method to find gold
                    // if gold position is already set, skip detection
                    if ( gold == GOLD_POSITIONS.UNKNOWN_POS )
                    {
                        FindGold( );
                    }

                    // Display mineral position on phone
                    telemetry.addLine().addData( "GP: ", gold.toString() );

                    // Move to next step
                    step = step.Next();
                    break;
                }

                case MOVE_TO_MINERAL:
                {
                    // Sample gold mineral
                    GoToGold();
                    step = step.Next();
                    break;
                }

                case LOAD_GOLD:
                {
                    // Can't load gold at this time
                    //LoadGold();
                    step = step.Next();
                    break;
                }

                case DRIVE_DEPOT:
                {
                    // Move from completion of sampling to Depot
                    DriveToDepot( );
//                    step = step.Next();
                    step = AUTON_STEPS.STOP;
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
                    // Back-up from the depot into the crater.
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

            }

            telemetry.addData("Is Found", GoldIsFound());   // Is there a gold block in view
            telemetry.addData("Y Pos", GoldYPosition());    // Gold Y pos.

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

        switch(gold)
        {   // Not defined yet
            case LEFT_POS:
                // Turn away from minerals to face the wall between the alliance crater and depot
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_TURN_2_WALL_LEFT_DEG);

                break;


            case MID_POS:
                // Turn away from minerals to face the wall between the alliance crater and depot
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_TURN_2_WALL_MID_DEG);
/*
                // Drive to near the wall, then rotate to face the depot
                go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_MOVE_2_WALL_MID_IN );
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG );

                // start positioning the arm for dropping the team marker and drive forward to
                // the depot, ready to unload the marker.
                arm.position_wrist( Arm.WRIST_POS.MOVE, WRIST_SPEED );
                go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_2_DEPOT_MID_IN );
*/                break;


            // Not defined yet
            case RIGHT_POS:
                // Turn away from minerals to face the wall between the alliance crater and depot
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_TURN_2_WALL_RIGHT_DEG);

                break;


            // Shouldn't be used
            case UNKNOWN_POS:
                break;
        }

    }

    @Override
    protected void UnloadGoldAndMarker( )
    {
        // arm should already be in correct position

        // Unload gold sample and adjust robot rotation for backing into crater.
        loader.AutonUnload();
        go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, ROTATE_AFTER_DROP_DEG );
    }

}


