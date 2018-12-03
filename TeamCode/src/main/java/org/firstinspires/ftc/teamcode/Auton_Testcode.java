package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


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

    // IMU members
    BNO055IMU    imu;
    Orientation  lastAngles = new Orientation();

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

        // Set up for IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode           = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize( parameters );

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

        telemetry.addData("Mode", "calibrating gyro...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        lastAngles = imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
        telemetry.addData("Mode ", "waiting for start");
        telemetry.addData("imu calib status ", imu.getCalibrationStatus().toString());
        telemetry.addData( " Z: ", lastAngles.firstAngle )
                 .addData( " Y: ", lastAngles.secondAngle )
                 .addData( " X: ", lastAngles.thirdAngle );
        telemetry.update();

        // Wait hit till start button pressed
        waitForStart();
        runtime.reset();

        // Set state you want to start in here
        step = AUTON_STEPS.RELEASE_LANDER;

        // simple states for stepping through release
        int releaseStep = 0;

        // Ensure gold position is unknown for testing
        // release, find gold and move to mineral
        gold = GOLD_POSITIONS.UNKNOWN_POS;

        // Loop until stop or forced to end
        while ( opModeIsActive( ) )
        {

            if ( gamepad1.dpad_up )
            {
                switch ( releaseStep )
                {
                    case 0:
                    {
                        arm.position_lift( Arm.lift_pos.hook_lander, LIFT_SPEED );
                        arm.WaitForInPos();

                        releaseStep++;
                        break;
                    }

                    case 1:
                    {
                        go.AutonMove( Drive.DIRECTION.RIGHT, RELEASE_STRAFE_IN );

                        releaseStep++;
                        break;
                    }

                    case 2:
                    {
                        go.AutonMove( Drive.DIRECTION.REVERSE, RELEASE_MOVE_AWAY_IN );

                        releaseStep++;
                        break;
                    }

                    case 3:
                    {
                        arm.position_lift( Arm.lift_pos.sampling, LIFT_SPEED );
                        arm.position_wrist( Arm.WRIST_POS.UNLOAD, WRIST_SPEED );
                        go.AutonMove( Drive.DIRECTION.LEFT, RELEASE_STRAFE_IN );

                        releaseStep++;
                        break;
                    }

                    case 4:
                    {
                        go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, RELEASE_ROTATE_DEG );

                        releaseStep++;
                        break;
                    }

                    default:
                    {
                        // do nuttin
                        break;
                    }
                }
            }
            else if ( gamepad1.dpad_down )
            {
                arm.position_lift( Arm.lift_pos.fulldown, LIFT_SPEED );
                arm.WaitForInPos();
            }

            if ( gamepad1.dpad_right )
            {
                go.AutonMove( Drive.DIRECTION.RIGHT, RELEASE_STRAFE_IN );
            }
            else if ( gamepad1.dpad_left )
            {
                go.AutonMove( Drive.DIRECTION.LEFT, RELEASE_STRAFE_IN );
            }

            if ( gamepad1.a )
            {
                go.AutonMoveRotate( Drive.ROTATION.CLOCKWISE, 180.0 );
            }
            else if ( gamepad1.b )
            {
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, 180.0 );
            }

            if ( gamepad1.x )
            {
                go.AutonMove( Drive.DIRECTION.FORWARD, 24.0 );
            }
            else if ( gamepad1.y )
            {
                go.AutonMove( Drive.DIRECTION.REVERSE, 24.0 );
            }

            /*    REMOVE STANDARD STATE MACHINE WHILE TRYING NEW THINGS
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
            */

            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData( " Z: ", lastAngles.firstAngle )
                     .addData( " Y: ", lastAngles.secondAngle )
                     .addData( " X: ", lastAngles.thirdAngle );
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

}


