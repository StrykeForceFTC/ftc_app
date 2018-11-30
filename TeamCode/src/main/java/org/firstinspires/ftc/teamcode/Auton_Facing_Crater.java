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
    private double ROTATE_BEFORE_DROP_DEG = 0;

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
                DRIVE_DEPOT_TURN_2_WALL_LEFT_DEG = 55.0;
                DRIVE_DEPOT_TURN_2_WALL_RIGHT_DEG = 116.0;
                DRIVE_DEPOT_TURN_2_WALL_MID_DEG = 82;

                DRIVE_DEPOT_MOVE_2_WALL_MID_IN = 42;
                DRIVE_DEPOT_MOVE_2_WALL_LEFT_IN = 36.5;
                DRIVE_DEPOT_MOVE_2_WALL_RIGHT_IN = 44;

                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_LEFT_DEG = 19.5;
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG = 26;
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_RIGHT_DEG = 33;

                DRIVE_DEPOT_FWD_2_DEPOT_LEFT_IN = 20;
                DRIVE_DEPOT_FWD_2_DEPOT_MID_IN = 19;
                DRIVE_DEPOT_FWD_2_DEPOT_RIGHT_IN = 20;

                ROTATE_AFTER_DROP_DEG = 7;
                break;
            }

            case team8553:
            {
                //no break, use 15106 tunings (untested)
            }

            case team15106:
            {
                DRIVE_DEPOT_TURN_2_WALL_LEFT_DEG = 66.0; //66
                DRIVE_DEPOT_MOVE_2_WALL_LEFT_IN = 41.3; //38.8
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_LEFT_DEG = 44;  // 56
                DRIVE_DEPOT_FWD_2_DEPOT_LEFT_IN = 24; //30

                DRIVE_DEPOT_TURN_2_WALL_MID_DEG = 86.0; //86
                DRIVE_DEPOT_MOVE_2_WALL_MID_IN = 41.5;  //41.5
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG = 57; //64
                DRIVE_DEPOT_FWD_2_DEPOT_MID_IN = 32; //35

                DRIVE_DEPOT_TURN_2_WALL_RIGHT_DEG = 131.0; //131
                DRIVE_DEPOT_MOVE_2_WALL_RIGHT_IN = 46.0; //49.5
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_RIGHT_DEG = 50; //50
                DRIVE_DEPOT_FWD_2_DEPOT_RIGHT_IN = 30; //20

                ROTATE_BEFORE_DROP_DEG = 7;
                ROTATE_AFTER_DROP_DEG = 0;

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

                // Drive to near the wall, then rotate to face the depot
                go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_MOVE_2_WALL_LEFT_IN );
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_ROT_PARALLEL_2_WALL_LEFT_DEG );

                // start positioning the arm for dropping the team marker and drive forward to
                // the depot, ready to unload the marker.
                arm.position_wrist( Arm.WRIST_POS.MOVE, WRIST_SPEED );
                go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_2_DEPOT_LEFT_IN );
                break;


            case UNKNOWN_POS:
            case MID_POS:
                // Turn away from minerals to face the wall between the alliance crater and depot
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_TURN_2_WALL_MID_DEG);

                // Drive to near the wall, then rotate to face the depot
                go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_MOVE_2_WALL_MID_IN );
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG );

                // start positioning the arm for dropping the team marker and drive forward to
                // the depot, ready to unload the marker.
                arm.position_wrist( Arm.WRIST_POS.MOVE, WRIST_SPEED );
                go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_2_DEPOT_MID_IN );

                break;


            // Not defined yet
            case RIGHT_POS:
                // Turn away from minerals to face the wall between the alliance crater and depot
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_TURN_2_WALL_RIGHT_DEG);

                // Drive to near the wall, then rotate to face the depot
                go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_MOVE_2_WALL_RIGHT_IN );
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_ROT_PARALLEL_2_WALL_RIGHT_DEG );

                // start positioning the arm for dropping the team marker and drive forward to
                // the depot, ready to unload the marker.
                arm.position_wrist( Arm.WRIST_POS.MOVE, WRIST_SPEED );
                go.AutonMove( Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_2_DEPOT_RIGHT_IN );

                break;

        }

    }

    @Override
    protected void UnloadGoldAndMarker( )
    {
        // arm should already be in correct position

        // Unload gold sample and adjust robot rotation for backing into crater.
        if (ROTATE_BEFORE_DROP_DEG != 0)
            go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, ROTATE_BEFORE_DROP_DEG );
        loader.AutonUnload();
        if (ROTATE_AFTER_DROP_DEG != 0)
            go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, ROTATE_AFTER_DROP_DEG );
    }

}


