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
    private double DRIVE_DEPOT_TURN_2_WALL_LEFT_DEG = 100.0;
    private double DRIVE_DEPOT_TURN_2_WALL_MID_DEG = 65.0;
    private double DRIVE_DEPOT_TURN_2_WALL_RIGHT_DEG = 55.0;

    private double DRIVE_DEPOT_MOVE_2_WALL_LEFT_IN = 30.0;
    private double DRIVE_DEPOT_MOVE_2_WALL_MID_IN = 40.0;
    private double DRIVE_DEPOT_MOVE_2_WALL_RIGHT_IN = 53.0;

    private double DRIVE_DEPOT_ROT_PARALLEL_2_WALL_LEFT_DEG = 75;
    private double DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG = 60;
    private double DRIVE_DEPOT_ROT_PARALLEL_2_WALL_RIGHT_DEG = 55;

    private double DRIVE_DEPOT_FWD_NEAR_DEPOT_LEFT_IN = 0;
    private double DRIVE_DEPOT_FWD_NEAR_DEPOT_MID_IN = 0;
    private double DRIVE_DEPOT_FWD_NEAR_DEPOT_RIGHT_IN = 0;

    private double DRIVE_DEPOT_STRAFE_2_WALL_LEFT_IN = 0;
    private double DRIVE_DEPOT_STRAFE_2_WALL_MID_IN = 0;
    private double DRIVE_DEPOT_STRAFE_2_WALL_RIGHT_IN = 0;

    private double DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_LEFT_IN = 0;
    private double DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_MID_IN = 0;
    private double DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_RIGHT_IN = 0;

    private double DRIVE_DEPOT_FWD_2_DEPOT_LEFT_IN = 26;
    private double DRIVE_DEPOT_FWD_2_DEPOT_MID_IN = 26;    // 33
    private double DRIVE_DEPOT_FWD_2_DEPOT_RIGHT_IN = 26;

    //private double DRIVE_DEPOT_STRAFE_2_WALL_IN = 13.0;


    private double ROTATE_BEFORE_DROP_DEG = 0;
    private double ROTATE_AFTER_DROP_DEG = 6;

    /*
     * There is only runOpMode for linear op modes
     */
    @Override
    public void runOpMode()
    {
        // If gold is set to a non unknown position, disable gold mineral position detection and assume the position is as set
//        gold = GOLD_POSITIONS.UNKNOWN_POS;
//        gold = GOLD_POSITIONS.LEFT_POS;
//        gold = GOLD_POSITIONS.MID_POS;
        gold = GOLD_POSITIONS.UNKNOWN_POS;

        /*
         * Initialize the hardware variables.
         */
        InitHardware();

        // Override values for various auton movements - if you want
        // to change from values in AutonLinearBase, then delete the "//"
        // and set to the desired value.
        PARK_DISTANCE_IN = 49;

        // Robot-specific overrides
        switch ( TeamId )
        {
            case team7228:
            {
                DRIVE_DEPOT_TURN_2_WALL_LEFT_DEG = 82.0;
                DRIVE_DEPOT_TURN_2_WALL_MID_DEG = 55;
                DRIVE_DEPOT_TURN_2_WALL_RIGHT_DEG = 25.0;

                DRIVE_DEPOT_MOVE_2_WALL_LEFT_IN = 29.25;
                DRIVE_DEPOT_MOVE_2_WALL_MID_IN = 37.5;
                DRIVE_DEPOT_MOVE_2_WALL_RIGHT_IN = 41.0;

                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_LEFT_DEG = 51;
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG = 50;
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_RIGHT_DEG = 52;

                DRIVE_DEPOT_FWD_2_DEPOT_LEFT_IN = 31.5;
                DRIVE_DEPOT_FWD_2_DEPOT_MID_IN = 31.5;
                DRIVE_DEPOT_FWD_2_DEPOT_RIGHT_IN = 31.5;

                ROTATE_BEFORE_DROP_DEG = 0;
                ROTATE_AFTER_DROP_DEG = 7;

                PARK_DISTANCE_IN = 49;
                
                break;
            }

            case team8553:
            {
                GO_TO_GOLD_FWD_RIGHT_IN = 10.0;
                GO_TO_GOLD_FWD_MID_IN = 4.5;

                DRIVE_DEPOT_TURN_2_WALL_LEFT_DEG = 105.0;
                DRIVE_DEPOT_TURN_2_WALL_MID_DEG = 88.0;
                DRIVE_DEPOT_TURN_2_WALL_RIGHT_DEG = 30.0;

                DRIVE_DEPOT_MOVE_2_WALL_LEFT_IN = 37.0;
                DRIVE_DEPOT_MOVE_2_WALL_RIGHT_IN = 45.0;

                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG = 65.0;
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_RIGHT_DEG = 52.0;
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_LEFT_DEG = 64.0;

                DRIVE_DEPOT_FWD_2_DEPOT_RIGHT_IN = 35.0;
                DRIVE_DEPOT_FWD_2_DEPOT_MID_IN = 30.0;
                DRIVE_DEPOT_FWD_2_DEPOT_LEFT_IN = 33.0;
                break;
            }

            case team15106:
            {
                DRIVE_DEPOT_TURN_2_WALL_LEFT_DEG = 113.0;
                DRIVE_DEPOT_MOVE_2_WALL_LEFT_IN = 34.0; // 37
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_LEFT_DEG = 67.0; // 62
                DRIVE_DEPOT_FWD_NEAR_DEPOT_LEFT_IN = 0;
                DRIVE_DEPOT_STRAFE_2_WALL_LEFT_IN = 0;
                DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_LEFT_IN = 0;
                DRIVE_DEPOT_FWD_2_DEPOT_LEFT_IN = 24.0; // 33

                DRIVE_DEPOT_TURN_2_WALL_MID_DEG = 88.0;
                DRIVE_DEPOT_MOVE_2_WALL_MID_IN = 39.0; // 40
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG = 75.0; // 65
                DRIVE_DEPOT_FWD_NEAR_DEPOT_MID_IN = 0;
                DRIVE_DEPOT_STRAFE_2_WALL_MID_IN = 0;
                DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_MID_IN = 0;
                DRIVE_DEPOT_FWD_2_DEPOT_MID_IN = 24.0; // 30

                DRIVE_DEPOT_TURN_2_WALL_RIGHT_DEG = 35.0;
                DRIVE_DEPOT_MOVE_2_WALL_RIGHT_IN = 47.0; // 49
                DRIVE_DEPOT_ROT_PARALLEL_2_WALL_RIGHT_DEG = 57.0; // 42
                DRIVE_DEPOT_FWD_NEAR_DEPOT_RIGHT_IN = 0;
                DRIVE_DEPOT_STRAFE_2_WALL_RIGHT_IN = 0;
                DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_RIGHT_IN = 0;
                DRIVE_DEPOT_FWD_2_DEPOT_RIGHT_IN = 32.0; // 50

                ROTATE_BEFORE_DROP_DEG = 0;
                if (gold == GOLD_POSITIONS.MID_POS)
                    ROTATE_BEFORE_DROP_DEG = 7;

                ROTATE_AFTER_DROP_DEG = 0;

                break;
            }
        }

        double zAngle = gyro.GetZAngle();
        telemetry.addData( " Z: ", zAngle );
        telemetry.update();

        // Wait hit till start button pressed
        waitForStart();
        runtime.reset();

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

            zAngle = gyro.GetZAngle();
            telemetry.addData( " Z Angle: ", zAngle );
            telemetry.addData("Is Found", GoldIsFound() );  // Is there a gold in view
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

        switch (gold)
        {   // Not defined yet
            case LEFT_POS:
                // Turn away from minerals to face the wall between the opposing alliance crater and depot
                go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, DRIVE_DEPOT_TURN_2_WALL_LEFT_DEG);

                // Drive to near the wall, then rotate to face the depot
                go.AutonMove(Drive.DIRECTION.REVERSE, DRIVE_DEPOT_MOVE_2_WALL_LEFT_IN);
                go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_ROT_PARALLEL_2_WALL_LEFT_DEG);

                // start positioning the arm for dropping the team marker and drive forward to
                // the depot, ready to unload the marker.
                arm.position_wrist(Arm.WRIST_POS.MOVE, WRIST_SPEED);
                if (DRIVE_DEPOT_FWD_NEAR_DEPOT_LEFT_IN != 0)
                    go.AutonMove(Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_NEAR_DEPOT_LEFT_IN);
                if (DRIVE_DEPOT_STRAFE_2_WALL_LEFT_IN != 0)
                    go.AutonMove(Drive.DIRECTION.LEFT, DRIVE_DEPOT_STRAFE_2_WALL_LEFT_IN);
                if (DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_LEFT_IN != 0)
                    go.AutonMove(Drive.DIRECTION.RIGHT, DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_LEFT_IN);
                go.AutonMove(Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_2_DEPOT_LEFT_IN);
                break;


            case MID_POS:
            case UNKNOWN_POS:   // for unknown gold mineral position, assume middle
                // Turn away from minerals to face the wall between the opposing alliance crater and depot
                go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, DRIVE_DEPOT_TURN_2_WALL_MID_DEG);

                // Drive to near the wall, then rotate to face the depot
                go.AutonMove(Drive.DIRECTION.REVERSE, DRIVE_DEPOT_MOVE_2_WALL_MID_IN);
                go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_ROT_PARALLEL_2_WALL_MID_DEG);

                // start positioning the arm for dropping the team marker and drive forward to
                // the depot, ready to unload the marker.
                arm.position_wrist(Arm.WRIST_POS.MOVE, WRIST_SPEED);
                if (DRIVE_DEPOT_FWD_NEAR_DEPOT_MID_IN != 0)
                    go.AutonMove(Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_NEAR_DEPOT_MID_IN);
                if (DRIVE_DEPOT_STRAFE_2_WALL_MID_IN != 0)
                    go.AutonMove(Drive.DIRECTION.LEFT, DRIVE_DEPOT_STRAFE_2_WALL_MID_IN);
                if (DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_MID_IN != 0)
                    go.AutonMove(Drive.DIRECTION.RIGHT, DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_MID_IN);
                go.AutonMove(Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_2_DEPOT_MID_IN);
                break;


            // Not defined yet
            case RIGHT_POS:
                // Turn away from minerals to face the wall between the opposing alliance crater and depot
                go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, DRIVE_DEPOT_TURN_2_WALL_RIGHT_DEG);

                // Drive to near the wall, then rotate to face the depot
                go.AutonMove(Drive.DIRECTION.REVERSE, DRIVE_DEPOT_MOVE_2_WALL_RIGHT_IN);
                go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, DRIVE_DEPOT_ROT_PARALLEL_2_WALL_RIGHT_DEG);

                // start positioning the arm for dropping the team marker and drive forward to
                // the depot, ready to unload the marker.
                arm.position_wrist(Arm.WRIST_POS.MOVE, WRIST_SPEED);
                if (DRIVE_DEPOT_FWD_NEAR_DEPOT_RIGHT_IN != 0)
                    go.AutonMove(Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_NEAR_DEPOT_RIGHT_IN);
                if (DRIVE_DEPOT_STRAFE_2_WALL_RIGHT_IN != 0)
                    go.AutonMove(Drive.DIRECTION.LEFT, DRIVE_DEPOT_STRAFE_2_WALL_RIGHT_IN);
                if (DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_RIGHT_IN != 0)
                    go.AutonMove(Drive.DIRECTION.LEFT, DRIVE_DEPOT_STRAFE_AWAY_FROM_WALL_RIGHT_IN);
                go.AutonMove(Drive.DIRECTION.FORWARD, DRIVE_DEPOT_FWD_2_DEPOT_RIGHT_IN);
                break;
        }
    }

    @Override
    protected void UnloadGoldAndMarker( )
    {
        // arm should already be in correct position

        // Unload gold sample and adjust robot rotation for backing into crater.
        if (ROTATE_BEFORE_DROP_DEG != 0)
            go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, ROTATE_BEFORE_DROP_DEG );
        loader.AutonUnload();
        if (ROTATE_AFTER_DROP_DEG != 0)
            go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, ROTATE_AFTER_DROP_DEG );
    }


}


