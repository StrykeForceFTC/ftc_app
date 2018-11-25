package org.firstinspires.ftc.teamcode;

import java.util.Date;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 * Base linear op mode for auton use - declares all of the robot HW and
 * DogeCV or Vuforia items along with implementing init and other code
 * to work with those.
 *
 */
public abstract class AutonLinearBase extends LinearOpMode
{
    // Run time tracker
    public ElapsedTime runtime = new ElapsedTime();

    // Declare HW objects
    public Drive go = null;
    public Loader loader = null;
    public Auto_Robot_Detect robotDetector = null;
    public Arm arm = null;

    // Detectors
    private GoldAlignDetector detector;

    // Team ID
    public Auto_Robot_Detect.teamId TeamId = Auto_Robot_Detect.teamId.teamUnknown;

    // TeamCode BuildID
    public String swBuildID = "";

    // Auton steps
    // Enumeration for auton steps allowing move to next step
    // using Next
    protected enum AUTON_STEPS
    {
        RELEASE_LANDER,
        FIND_GOLD,
        MOVE_TO_MINERAL,
        LOAD_GOLD,
        DRIVE_DEPOT,
        UNLOAD,
        PARK,
        STOP;

        private static AUTON_STEPS[] vals = values();
        public AUTON_STEPS Next( )
        {
            if ( this != STOP )
            {
                return vals[ ( this.ordinal() + 1 ) ];
            }

            return STOP;
        }

    }

    // Enumeration for possible positions of the gold mineral
    protected enum GOLD_POSITIONS {
        LEFT_POS,
        MID_POS,
        RIGHT_POS,
        UNKNOWN_POS
    }

    // Variables for step we are on and position of gold mineral
    protected AUTON_STEPS step = AUTON_STEPS.RELEASE_LANDER;
    protected GOLD_POSITIONS gold = GOLD_POSITIONS.UNKNOWN_POS;

    // Values for distances to move, degrees to rotate, etc.
    protected double RELEASE_STRAFE_IN = 4.0;
    protected double RELEASE_MOVE_AWAY_IN = 7.0;
    protected double RELEASE_ROTATE_DEG = 172.0;
    protected double FIND_GOLD_INITIAL_CW_ROT_DEG = 135.0;
    protected double FIND_GOLD_ROTATE_4_SAMPLE_IN = 45.0;
    protected double GO_TO_GOLD_FWD_IN = 4.25;
    protected double GO_TO_GOLD_SIDEWAYS_IN = 8.0;
    protected double LOAD_GOLD_FWD_IN = 4.0;
    protected double PARK_DISTANCE_IN = 44.0;
    protected int    LIFT_SPEED = 10;
    protected int    WRIST_SPEED = 10;


    // Method to initialize any connected hardware
    public void InitHardware( )
    {
        // Find what robot you are running and set up hardware
        robotDetector = new Auto_Robot_Detect( hardwareMap );
        TeamId = robotDetector.TeamId;
        go = new Drive( hardwareMap );
        loader = new Loader( hardwareMap );
        arm = new Arm( hardwareMap, true );

        /*
         ** Rest of this method is about starting up a Gold detector from
         ** DogeCV.
         */
        // Initialize
        detector = new GoldAlignDetector();
        detector.init( hardwareMap.appContext, CameraViewDisplay.getInstance( ) );
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        switch (TeamId)
        {
            case team7228: {
                //RELEASE_MOVE_AWAY_IN = 9.0;
                GO_TO_GOLD_FWD_IN = 6.0;
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

        Date buildDate = BuildConfig.BUILD_TIME;
        swBuildID = hardwareMap.appContext.getString(R.string.gitBranch) + " @ " + buildDate.toString();

        AddStdAutonTelemetry(false);
        telemetry.update();
    }

    protected void AddStdAutonTelemetry(boolean tfShowEncoderValues)
    {
        if (tfShowEncoderValues) {
            telemetry.addLine("Drive Encoders: ")
                    .addData("FL ", go.GetEncoderFrontLeft())
                    .addData("FR ", go.GetEncoderFrontRight())
                    .addData("RL ", go.GetEncoderRearLeft())
                    .addData("RR ", go.GetEncoderRearRight());

            telemetry.addLine("Arm Pos: ")
                    .addData("Lift ", arm.LiftEncoderValue())
                    .addData("Wrist ", arm.WristEncoderValue());
        }

        telemetry.addLine("Robot Id: " + TeamId.name());

        telemetry.addLine("TeamCode Build ID: ");
        telemetry.addLine("   " + swBuildID);
    }

    // Method to determined if aligned on gold block
    public boolean GoldAligned( )
    {
        return detector.getAligned();
    }


    // Method to determine X offset of gold block
    public double GoldXPosition( )
    {
        return detector.getXPosition( );
    }

    // Method to run for stop step of auton - ensures all HW left in
    // known and safe state
    protected void StopActions( )
    {
        // Ensure loader motor is off
        loader.teleopstop();

        // Setting all inputs to move simple to 0 ensures all
        // drive motors are set to 0 power and stopped.
        go.MoveSimple( 0, 0, 0 );

        // Stop the gold align detector
        detector.disable( );

        // Next line indicates to robot core that we are requesting
        // to stop the op mode (like hitting stop button on driver station)
        requestOpModeStop( );
    }

    /*
    ** Methods common to all auton routines.
    */
    protected void ReleaseLander( )
    {
        arm.position_lift( Arm.lift_pos.hook_lander, LIFT_SPEED );

        telemetry.addLine("LOWERING");
        AddStdAutonTelemetry(true);
        telemetry.update();

        arm.WaitForInPos();

        telemetry.addLine("DONE");
        AddStdAutonTelemetry(true);
        telemetry.update();

        go.AutonMove( Drive.DIRECTION.RIGHT, RELEASE_STRAFE_IN );
        go.AutonMove( Drive.DIRECTION.REVERSE, RELEASE_MOVE_AWAY_IN );
        arm.position_lift( Arm.lift_pos.sampling, LIFT_SPEED );
        arm.position_wrist( Arm.WRIST_POS.UNLOAD, WRIST_SPEED );
        go.AutonMove( Drive.DIRECTION.LEFT, RELEASE_STRAFE_IN );
        go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, RELEASE_ROTATE_DEG );
        arm.position_wrist( Arm.WRIST_POS.LOAD, WRIST_SPEED );
        arm.WaitForInPos();
    }

    // Method to find position of gold mineral.
    protected void FindGold( )
    {
        // Initial rotation to align with sample on left when facing crater or depot
        go.AutonMoveRotate( Drive.ROTATION.CLOCKWISE, FIND_GOLD_INITIAL_CW_ROT_DEG );

        if ( GoldAligned() )
        {
            gold = GOLD_POSITIONS.LEFT_POS;
            go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, FIND_GOLD_ROTATE_4_SAMPLE_IN );

        }
        else
        {
            go.AutonMoveRotate( Drive.ROTATION.CLOCKWISE, FIND_GOLD_ROTATE_4_SAMPLE_IN );
            if ( GoldAligned() )
            {
                gold = GOLD_POSITIONS.MID_POS;

            }
            else
            {
                go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, FIND_GOLD_ROTATE_4_SAMPLE_IN );
                if ( GoldAligned() )
                {
                    gold = GOLD_POSITIONS.RIGHT_POS;
                    go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, FIND_GOLD_ROTATE_4_SAMPLE_IN );
                }
                else
                {
                    gold = GOLD_POSITIONS.UNKNOWN_POS;
                    go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, FIND_GOLD_ROTATE_4_SAMPLE_IN );

                }
            }

        }
    }

    // Method to move to gold mineral position
    protected void GoToGold( )
    {
        /*switch ( gold )
        {
            case LEFT_POS:
                go.AutonMove( Drive.DIRECTION.LEFT, GO_TO_GOLD_SIDEWAYS_IN );
                break;

            case MID_POS:
                break;

            case RIGHT_POS:
                go.AutonMove( Drive.DIRECTION.RIGHT, GO_TO_GOLD_SIDEWAYS_IN );
                break;

            case UNKNOWN_POS:
                break;
        }
        */

        go.AutonMove( Drive.DIRECTION.FORWARD, GO_TO_GOLD_FWD_IN );
        go.AutonMoveRotate( Drive.ROTATION.CLOCKWISE, 30 );
        arm.position_wrist( Arm.WRIST_POS.UNLOAD, WRIST_SPEED );
    }


    // Method to load gold mineral
    protected void LoadGold( )
    {
        // Only load if we found the gold mineral
        if ( gold != GOLD_POSITIONS.UNKNOWN_POS )
        {
            go.AutonMove(Drive.DIRECTION.FORWARD, LOAD_GOLD_FWD_IN);

            //! @todo Need arm method to put to load position
            // arm.LoadSamplesPositionSet();
            loader.AutonLoad();

            //put down arm and turn on loader
            go.AutonMove( Drive.DIRECTION.REVERSE, LOAD_GOLD_FWD_IN );

            switch ( gold )
            {
                case LEFT_POS:
                    go.AutonMove( Drive.DIRECTION.RIGHT, GO_TO_GOLD_SIDEWAYS_IN );
                    break;

                case MID_POS:
                    break;

                case RIGHT_POS:
                    go.AutonMove( Drive.DIRECTION.LEFT, GO_TO_GOLD_SIDEWAYS_IN );
                    break;
            }
        }
    }

    // The drive to depot method is expected to be different for each
    // position, so each auton routine must override it.
    abstract protected void DriveToDepot( );

    // Common method to unload marker and gold sample
Ma    abstract protected void UnloadGoldAndMarker( );

    // Method to get to park position
    protected void ParkTheRobot( )
    {
        // Raise arm to move position?
        go.AutonMove( Drive.DIRECTION.REVERSE, PARK_DISTANCE_IN );

    }

}