package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by jscott on 11/10/17.
 *
 * Auton Program mainly here to test Auton Drive and other Functions in
 *
 */
@Autonomous(name = "Auton_Testcode", group = "Linear Opmode")
//@Disabled
public class Auton_Testcode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    HardwareMap robotMap = hardwareMap;
    private Drive go = null;

    // Enumeration for auton steps
    private enum AUTON_STEPS { START, PICK_UP_GLYPH, KNOCK_OFF_JEWEL,
        MOVE_IN_FRONT_OF_BOX, ROTATE_TO_FACE_BOX, MOVE_FORWARD_TO_BOX,
        DROP_GYLPH, BACK_UP, STOP }

    private AUTON_STEPS step = AUTON_STEPS.START;

    // Constants for controlling / tuning auton, long distances reduced by 1"
    // to make up for error we have seen. All values in cm or degrees
    private static final double DISTANCE_FOR_LEFT_COLUMN = ( 72.07 - 2.54 );     //
    private static final double DISTANCE_FOR_CENTER_COLUMN = ( 91.44 - 2.54 );   //
    private static final double DISTANCE_FOR_RIGHT_COLUMN = ( 110.81 - 2.54 );   //
    private static final double DEGREES_2_ROTATE = 90.0;              // Must rotate CCW
    private static final double DISTANCE_FORWARD_2_DROP = 20.32;      // Leave long, will just run into wall
    private static final double DISTANCE_FOR_JEWEL = 9.21;            // Distance to move to knock off a jewel

    // Used to compensate for movement to knock off jewel
    private double yDistanceFromStart = 0.0;

    /*
     * There is only runOpMode for linear op modes
     */
    @Override
    public void runOpMode()
    {
        /*
        **  Initialization code
         */
        telemetry.addData("Status", "Initializing");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        go = new Drive( hardwareMap );

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait hit till start button pressed
        waitForStart();
        runtime.reset();

        /*
        // Loop until stop or forced to end
        while ( opModeIsActive() )
        {
            int errorTicks = 0;
            go.AutonMove(Drive.DIRECTION.FORWARD, 48);


            telemetry.addData( "Movement Error ", errorTicks );

            telemetry.addLine("Encoders ")
                    .addData("FL ", frontLeft.getCurrentPosition() )
                    .addData("FR ", frontRight.getCurrentPosition() );

            telemetry.update();

            idle();
        }
        */
        go.AutonMove( Drive.DIRECTION.FORWARD, 12.0 );
        telemetry.addLine("Encoders ")
                .addData("FL ", go.GetEncoderFrontLeft() )
                .addData("FR ", go.GetEncoderFrontRight() )
                .addData("RL ", go.GetEncoderRearLeft() )
                .addData("RR ", go.GetEncoderRearRight() );
    }


}


