package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by jscott on 11/10/17.
 *
 * Example auton program using time for movement based on red back position.
 *
 */
@Autonomous(name = "Auton Time Based Example", group = "Iterative Opmode")
public class Auton_Time_Based_Example extends OpMode {

    public static final String TAG = "Vuforia Navigation Sample";

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime delayTimer = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;

    HardwareMap robotMap = hardwareMap;
    private Drive go = null;
    private Pole wep = null;
    private Claw claw = null;
    private Lift lift = null;

    // determine new target
    int newLeftTarget;
    int newRightTarget;
    int targetX, targetY, targetSpin;

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    OpenGLMatrix lastLocation = null;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    // Enumeration for auton steps
    private enum AUTON_STEPS { START, KNOCK_OFF_JEWEL, MOVE_TO_CRYPTO_BOX, STOP }

    private AUTON_STEPS step = AUTON_STEPS.START;

    // Constants for controlling / tuning auton
    private static double TIME_2_KNOCK_JEWEL = 3.0;         // TODO: VALUE NEEDS TO BE CHOSEN
    private static double TIME_2_MOVE_2_CRYPTO = 10.0;      // TODO: VALUE NEEDS TO BE CHOSEN

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
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

        /**
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

        /**
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
        // Keep looking for vumark until found
        if ( vuMark == RelicRecoveryVuMark.UNKNOWN )
        {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        // Keep looking for vumark until found
        if ( vuMark == RelicRecoveryVuMark.UNKNOWN )
        {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        // Open the claw to get ready to pick up glyph
        claw.claw_Outward();

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        // Keep looking for vumark until found (or we just don't care anymore)
        if ( vuMark == RelicRecoveryVuMark.UNKNOWN )
        {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }

        switch (step)
        {
            case START:
                step = AUTON_STEPS.KNOCK_OFF_JEWEL;
                break;

            case KNOCK_OFF_JEWEL:
                // Lower jewel knocker
                // read color sensor
                // move forward or reverse based on color sensor

                step = AUTON_STEPS.MOVE_TO_CRYPTO_BOX;
                break;

            case MOVE_TO_CRYPTO_BOX:
            {
                // Move backwards for red side
                go.MoveSimple( 0.0, -1.0, 0.0 );
                Delay_s( TIME_2_MOVE_2_CRYPTO );

                // Stop moving
                go.MoveSimple( 0.0, 0.0, 0.0 );

                step = AUTON_STEPS.STOP;
            }
                break;

            case STOP:
                // In stop, just turn all motors off for safety
                go.MoveSimple( 0.0, 0.0, 0.0 );
                claw.claw_Outward();
                wep.stay();
                wep.lift(0.0);
                break;

            default:  // Should never get here, so just go to stop
                step = AUTON_STEPS.STOP;
                break;
        }

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    // Delay time method
    private void Delay_s( double seconds )
    {
        delayTimer.reset();
        while ( delayTimer.time() < seconds )
        {
            telemetry.addLine( "Encoder " )
                    .addData( "Value", frontLeft.getCurrentPosition() );
            telemetry.update();
        }
    }


}


