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
 * auton program based on the rear position for the blue team that uses
 * encoders for longer distances and rotation. For 7228
 *
 */
@Autonomous(name = "Blue Rear", group = "Linear Opmode")
//@Disabled
public class Auton_Blue_Rear extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;

    HardwareMap robotMap = hardwareMap;
    private Drive go = null;
    private Pole wep = null;
    private Claw claw = null;
    private Lift lift = null;
    private JewelKnocker jewelKnocker = null;

    // Jewel Knocker hardware
    private Servo knockerServo = null;
    private ColorSensor colorSensor = null;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    // Enumeration for auton steps
    private enum AUTON_STEPS { START, PICK_UP_GLYPH, KNOCK_OFF_JEWEL,
        MOVE_T0WARDS_BOX, MOVE_TO_COLUMN, ADJUST_ANGLE, MOVE_FORWARD_TO_BOX,
        DROP_GYLPH, BACK_UP, STOP }

    private AUTON_STEPS step = AUTON_STEPS.START;

    // Constants for controlling / tuning auton, long distances reduced by 1"
    // to make up for error we have seen. All values in cm or degrees
    private static final double DISTANCE_TOWARDS_BOX = ( 60.96 - 2.54 );         // Distance from starting point to being in front of crypto box (also need to move sideways)
    private static final double DISTANCE_FOR_LEFT_COLUMN = 11.11;     //
    private static final double DISTANCE_FOR_CENTER_COLUMN = 30.48;   //
    private static final double DISTANCE_FOR_RIGHT_COLUMN = 49.85;    //
    private static final double DEGREES_2_ROTATE = 5.0;               // Small rotation to angle inwards to box
    private static final double DISTANCE_FORWARD_2_DROP = 20.32;      // Leave long; will just run into wall
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
        frontLeft = hardwareMap.dcMotor.get("front_left");
        frontRight = hardwareMap.dcMotor.get("front_right");
        rearLeft = hardwareMap.dcMotor.get("rear_left");
        rearRight = hardwareMap.dcMotor.get("rear_right");

        go = new Drive(frontLeft, frontRight, rearLeft, rearRight);

        // Set up Claw
        claw = new Claw( hardwareMap );

        // Set up pole
        wep = new Pole( hardwareMap );

        // Set up lift
        lift = new Lift( hardwareMap );

        knockerServo = hardwareMap.servo.get( "knocker_servo" );
        colorSensor = hardwareMap.colorSensor.get( "color" );
        jewelKnocker = new JewelKnocker( knockerServo, colorSensor );

        /*
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
         * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
         * purposes here, we choose the back camera; for a competition robot, the front camera might
         * prove to be more convenient.
         *
         * Note that in addition to indicating which camera is in use, we also need to tell the system
         * the location of the phone on the robot; see phoneLocationOnRobot below.
         *
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * Vuforia will not load without a valid license being provided. Vuforia 'Development' license
         * keys, which is what is needed here, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Valid Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string form of the key from the Vuforia web site
         * and paste it in to your code as the value of the 'vuforiaLicenseKey' field of the
         * {@link Parameters} instance with which you initialize Vuforia.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AcQJECX/////AAAAGf7VwDG0xk/wixT8Uym8/slG+89jXtXE+3T0C7q/hKEFDzsXaxquypbEhjwA+Zs8dA0ZMf1G8uxQxyDr4pTAp3y0w/S3evObZXNu8ghKY3ZgODj+s9BrWuylH7WvYDpeC2EA0g2EP4X58as1NdAqegUuZzT+pepCzqLIhNVeYmYysc8E6Y9ADPitfFy4jWkmIKOEYenlLlACqWQFZp9eriWtZbIXyAjT3do/pg34l+Zpc5MKLuGOwVkWprByhmjvYnyVn5CZaRwjBNFTho3GQuMvKIanA40rht6L93FzIg/rTbiKnyk/aQ+EQIG2BUtYRFEAXxgmvBN+zm1vUnjOpKWoE6QInksknLqrzGO1mht3";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /*
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
         * example "StonesAndChips", datasets can be found in in this project in the
         * documentation directory.
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Open the claw to get ready to pick up glyph
        claw.claw_Outward();

        // Look for vumark 5x during init unless it is found
        for ( int loop = 0; loop < 5; loop++ )
        {
            if (vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }
        }

        // Wait hit till start button pressed
        waitForStart();
        runtime.reset();

        // Create rotate adjustment outside of while loop so it doesn't get
        // reset each pass of the loop
        double rotateAngle = 0.0;

        // Loop until stop or forced to end
        while ( opModeIsActive() )
        {
            int errorTicks = 0;

            // Keep looking for vumark until found (or we just don't care anymore)
            if ( vuMark == RelicRecoveryVuMark.UNKNOWN )
            {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
            }

            /*
            if ( gamepad1.x )
            {
                errorTicks = go.AutonForward( 3.5*2.54 );
            }

            if ( gamepad1.y )
            {
                errorTicks = go.AutonReverse( 3.5*2.54 );
            }

            if ( gamepad1.a )
            {
                errorTicks = go.AutonRight( 6.0*2.54 );
            }

            if ( gamepad1.b )
            {
                errorTicks = go.AutonLeft( 6.0*2.54 );
            }
            */

            switch ( step )
            {
                case START:
                    // Move pole out of way of lift
                    wep.autonLift();
                    step = AUTON_STEPS.PICK_UP_GLYPH;
                    break;

                case PICK_UP_GLYPH:
                    // To pick up the glyph, close claw and raise lift a bit.
                    claw.claw_Inward();

                    // short delay to let claw close
                    sleep( 250 );

                    lift.AutonRaise();
                    step = AUTON_STEPS.KNOCK_OFF_JEWEL;
                    break;

                case KNOCK_OFF_JEWEL:
                    // Lower jewel knocker and delay to give time to move
                    jewelKnocker.LowerKnocker();
                    sleep( 500 );

                    // read color sensor
                    JewelKnocker.COLORS color = jewelKnocker.GetColor();

                    // Move forward or reverse based on color sensor, knock off red, because
                    // we are blue. The color sensor points forward.
                    if ( color == JewelKnocker.COLORS.RED )
                    {
                        // Jewel to front is red, so move forward to knock off
                        go.MoveSimple( 0.0, -0.23, 0.0 );
                        sleep( 300 );
                        go.MoveSimple( 0.0, 0.0, 0.0 );
                        yDistanceFromStart = DISTANCE_FOR_JEWEL;
                    }
                    else
                    {
                        // Need to move backwards to knock off the red jewel; note that this
                        // means distance is negative.
                        go.MoveSimple( 0.0, 0.23, 0.0 );
                        sleep( 300 );
                        go.MoveSimple( 0.0, 0.0, 0.0 );
                        yDistanceFromStart = -DISTANCE_FOR_JEWEL;
                    }

                    // Raise the knocker and give it time to move
                    jewelKnocker.RaiseKnocker();
                    sleep( 500 );

                    // Go to next step
                    step = AUTON_STEPS.MOVE_T0WARDS_BOX;
                    break;

                case MOVE_T0WARDS_BOX:

                    // Move towards box and slight delay to make sure we have stopped
                    errorTicks = go.AutonForward( DISTANCE_TOWARDS_BOX - yDistanceFromStart );
                    sleep( 250 );

                    // go to next step
                    step = AUTON_STEPS.MOVE_TO_COLUMN;
                    break;

                case MOVE_TO_COLUMN:
                {
                    switch (vuMark)
                    {
                        case LEFT:
                            errorTicks = go.AutonRight( DISTANCE_FOR_LEFT_COLUMN );
                            rotateAngle = DEGREES_2_ROTATE;
                            break;

                        case RIGHT:
                            errorTicks = go.AutonRight( DISTANCE_FOR_RIGHT_COLUMN );
                            rotateAngle = -DEGREES_2_ROTATE;
                            break;

                        default:  // Default is for unknown or center
                        {
                            errorTicks = go.AutonRight( DISTANCE_FOR_CENTER_COLUMN );
                            rotateAngle = 0.0;
                        }
                        break;
                    }

                    step = AUTON_STEPS.ADJUST_ANGLE;
                }
                    break;

                case ADJUST_ANGLE:
                    if (rotateAngle < 0.0)
                    {
                        errorTicks = go.AutonRotateCounterclockwise( Math.abs(rotateAngle) );
                    }
                    else
                    {
                        errorTicks = go.AutonRotateClockwise(rotateAngle);
                    }


                    step = AUTON_STEPS.MOVE_FORWARD_TO_BOX;
                    break;

                case MOVE_FORWARD_TO_BOX:
                    errorTicks = go.AutonForward( DISTANCE_FORWARD_2_DROP );
                    step = AUTON_STEPS.DROP_GYLPH;
                    break;

                case DROP_GYLPH:
                    claw.claw_Outward();
                    sleep( 250 );
                    lift.AutonLower();
                    step = AUTON_STEPS.BACK_UP;
                    break;

                case BACK_UP:
                    go.MoveSimple( 0.0, 0.35, 0.0 );
                    sleep( 150 );
                    go.MoveSimple( 0, 0, 0 );
                    step = AUTON_STEPS.STOP;
                    break;

                case STOP:
                    // In stop, just turn all motors off for safety
                    go.MoveSimple( 0.0, 0.0, 0.0 );
                    lift.Raise( 0.0 );
                    claw.claw_Outward();
                    wep.stay();
                    wep.lift( 0.0 );
                    jewelKnocker.RaiseKnocker( );

                    // Force to stop mode
                    requestOpModeStop();

                    break;

                default:  // Should never get here, so just go to stop
                    step = AUTON_STEPS.STOP;
                    break;
            }


            telemetry.addData("VuMark", "%s visible", vuMark);
            telemetry.addData( "Movement Error ", errorTicks );

            telemetry.addLine("Encoders ")
                    .addData("FL ", frontLeft.getCurrentPosition() )
                    .addData("FR ", frontRight.getCurrentPosition() );

            telemetry.update();

            idle();
        }
    }

}


