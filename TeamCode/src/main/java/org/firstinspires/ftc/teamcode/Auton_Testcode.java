package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 *
 * Auton Program mainly here to test Auton Drive and other Functions in
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

        // Wait hit till start button pressed
        waitForStart();
        runtime.reset();

        // Set state you want to start in here
        step = AUTON_STEPS.RELEASE_LANDER;

        // Ensure gold position is unknown for testing
        // release, find gold and move to mineral
        gold = GOLD_POSITIONS.UNKNOWN_POS;

        // Loop until stop or forced to end
        while ( opModeIsActive( ) )
        {

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
                    //step = step.Next();
                    step = step.Next();
                    break;
                }

                case MOVE_TO_MINERAL:
                {
                    // Sample gold mineral
                    GoToGold();
                    step = step.STOP;
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

            telemetry.addData("Is Found", GoldIsFound());   // Is the bot aligned with the gold mineral
            telemetry.addData("Y Pos", GoldYPosition());    // Gold Y pos.
            telemetry.addLine( gold.toString() );

            telemetry.addLine("Encoders ")
                    .addData("FL ", go.GetEncoderFrontLeft())
                    .addData("FR ", go.GetEncoderFrontRight())
                    .addData("RL ", go.GetEncoderRearLeft())
                    .addData("RR ", go.GetEncoderRearRight());
            telemetry.addLine("TeamId")
                    .addData("Team", TeamId.name());

            telemetry.update();

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

}


