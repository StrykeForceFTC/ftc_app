package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private final static double DRIVE_DEPOT_TURN_2_WALL_DEG = 75.0;
    private final static double DRIVE_DEPOT_MOVE_2_WALL_IN = 37.0;
    private final static double DRIVE_DEPOT_ROT_PARALLEL_2_WALL_DEG = 27.5;
    private final static double DRIVE_DEPOT_STRAFE_2_WALL_IN = 11.0;
    private final static double DRIVE_DEPOT_FWD_2_DEPOT = 32.5;

    /*
     * There is only runOpMode for linear op modes
     */
    @Override
    public void runOpMode()
    {
        /* Initialize the hardware variables.
         */
        InitHardware();

        // Wait hit till start button pressed
        waitForStart();
        runtime.reset();

        // Set state you want to start in here
        step = AUTON_STEPS.RELEASE_LANDER;

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
                    step = AUTON_STEPS.STOP;
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
                    step = AUTON_STEPS.PARK;
                    break;
                }

                case PARK:
                {
                    ParkTheRobot();
                    step = AUTON_STEPS.STOP;
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


