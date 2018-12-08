package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetectorVertical;
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
    protected Gyro gyro = null;

    // Detectors
    private GoldAlignDetectorVertical detector;

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
    protected double RELEASE_ROTATE_DEG = 142.0;                // Reduced by 30 for gold detection
    protected double FIND_GOLD_INITIAL_CW_ROT_DEG = 135.0;
    protected double FIND_GOLD_ROTATE_4_SAMPLE_IN = 45.0;

    // FindGold constants
    protected double UPPER_Y_LIMIT_FOR_RIGHT = 200;             // When Y value is <= this, assume gold is in right position


    // GoToGold() Default Positioning Values
    protected double GO_TO_GOLD_ROTATE_TO_RIGHT_DEG = 25; // 15          // Clockwise! to get to right
    protected double GO_TO_GOLD_ROTATE_TO_MID_DEG = 30;             // Counter clockwise
    protected double GO_TO_GOLD_ROTATE_TO_LEFT_DEG = 60;            // Mid rotate plus extra to get to left, CCW
    protected double GO_TO_GOLD_FWD_LEFT_IN = 6.25;
    protected double GO_TO_GOLD_FWD_MID_IN = 4.25;
    protected double GO_TO_GOLD_FWD_RIGHT_IN = 6.25;
    protected double GO_TO_GOLD_SIDEWAYS_LEFT_IN = 14.0;
    protected double GO_TO_GOLD_SIDEWAYS_RIGHT_IN = 14.0;

    protected double LOAD_GOLD_FWD_IN = 4.0;
    protected double PARK_DISTANCE_IN = 44.0;
    protected int    LIFT_SPEED = 10;
    protected int    WRIST_SPEED = 10;

    protected double ROTATE_TO_KNOCK_OFF_SAMPLE_DEG = 30;

    protected double startingZAngle = 0;

    // Method to initialize any connected hardware
    public void InitHardware( )
    {
        // Find what robot you are running and set up hardware
        robotDetector = new Auto_Robot_Detect( hardwareMap );
        TeamId = robotDetector.TeamId;
        go = new Drive( hardwareMap, true );
        loader = new Loader( hardwareMap );
        arm = new Arm( hardwareMap, true, TeamId );
        gyro = new Gyro( hardwareMap );

        /*
         ** Rest of this method is about starting up a Gold detector from
         ** DogeCV.
         */
        // Initialize
        detector = new GoldAlignDetectorVertical();
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

        // Robot-specific overrides
        switch (TeamId)
        {
            case team7228:
            {
                RELEASE_ROTATE_DEG = 155;
                RELEASE_MOVE_AWAY_IN = 9.0;
                RELEASE_STRAFE_IN = 5.0;

                UPPER_Y_LIMIT_FOR_RIGHT = 250;

                GO_TO_GOLD_FWD_MID_IN = 3;
                GO_TO_GOLD_ROTATE_TO_MID_DEG = 15;              // Counter clockwise

                GO_TO_GOLD_FWD_LEFT_IN = 6.5;
                GO_TO_GOLD_ROTATE_TO_LEFT_DEG = 38;            // Mid rotate plus extra to get to left, CCW

                GO_TO_GOLD_FWD_RIGHT_IN = 7.5 ;
                GO_TO_GOLD_ROTATE_TO_RIGHT_DEG = 18 ;           // Clockwise! to get to right

                ROTATE_TO_KNOCK_OFF_SAMPLE_DEG = 30;

                PARK_DISTANCE_IN = 44.0;
                break;
            }

            case team8553:
            {
                RELEASE_ROTATE_DEG = 172.0;
                GO_TO_GOLD_ROTATE_TO_LEFT_DEG = 40;
                GO_TO_GOLD_FWD_LEFT_IN = 8.25;
                GO_TO_GOLD_ROTATE_TO_MID_DEG = 15;
                GO_TO_GOLD_FWD_RIGHT_IN = 8.25;
                GO_TO_GOLD_ROTATE_TO_RIGHT_DEG = 20;
                break;
            }

            case team15106:
            {
                UPPER_Y_LIMIT_FOR_RIGHT = 200;

                LIFT_SPEED = 10;
                WRIST_SPEED = 10;

                RELEASE_STRAFE_IN = 4.0;
                RELEASE_MOVE_AWAY_IN = 7.0;
                RELEASE_ROTATE_DEG = 172.0; // 172

                LOAD_GOLD_FWD_IN = 4.0;

                GO_TO_GOLD_ROTATE_TO_LEFT_DEG = 40; // 40
                GO_TO_GOLD_FWD_LEFT_IN = 8.25;  // 8.25

                GO_TO_GOLD_ROTATE_TO_MID_DEG = 15;  // 15
                GO_TO_GOLD_FWD_MID_IN = 4.75;   // 4.25

                GO_TO_GOLD_ROTATE_TO_RIGHT_DEG = 22; // 25
                GO_TO_GOLD_FWD_RIGHT_IN = 9.25; // 8.25

                ROTATE_TO_KNOCK_OFF_SAMPLE_DEG = 35; // 30

                PARK_DISTANCE_IN = 44.0; // 44

                break;
            }
        }

        // Get the TeamCode build ID from the resources
        swBuildID = hardwareMap.appContext.getString(R.string.TeamCode_BuildID);

        // Send initial telemetry data
        AddStdAutonTelemetry(false);
        telemetry.update();
    }

    protected void AddStdAutonTelemetry(boolean tfShowEncoderValues)
    {
        telemetry.addData( "Gold Position ", gold.toString() );
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

        telemetry.addLine("TeamCode Build ID: " + swBuildID);
    }

    // Method to determined if gold block is found
    public boolean GoldIsFound( )
    {
        return detector.isFound();
    }


    // Method to determine Y offset of gold block
    public double GoldYPosition( )
    {
        return detector.getYPosition( );
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
        // Record starting angle
        startingZAngle = gyro.GetZAngle();

        // Raise lift to drop the robot to the ground & update telemetry
        arm.position_lift( Arm.lift_pos.hook_lander, LIFT_SPEED );
        telemetry.addLine("LOWERING");
        AddStdAutonTelemetry(true);
        telemetry.update();

        // Wait for the lift to be raised to position then update telemetry
        arm.WaitForInPos();
        telemetry.addLine("DONE");
        AddStdAutonTelemetry(true);
        telemetry.update();

        // Strafe robot hook off of lander & back away from lander
        go.AutonMove( Drive.DIRECTION.RIGHT, RELEASE_STRAFE_IN );
        GyroCorrect();
        go.AutonMove( Drive.DIRECTION.REVERSE, RELEASE_MOVE_AWAY_IN );

        // Start raising the arm to a near vertical position and strafe back to near center on the lander side
        arm.position_lift( Arm.lift_pos.sampling, LIFT_SPEED );
        arm.position_wrist( Arm.WRIST_POS.UNLOAD, WRIST_SPEED );
        go.AutonMove( Drive.DIRECTION.LEFT, RELEASE_STRAFE_IN );

        // Rotate to face the minerals
        RELEASE_ROTATE_DEG = RELEASE_ROTATE_DEG - ( gyro.GetZAngle() - startingZAngle );
        go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, RELEASE_ROTATE_DEG );

        // Wrist is now left in unload position while finding gold position
    }

    // Method to find position of gold mineral.
    protected void FindGold( )
    {
        // Should already be aligned so pointing at middle and right position
        int attempts = 0;
        int position_counter[] = new int[ GOLD_POSITIONS.values().length ];
        for ( int index = 0; index < position_counter.length; index++ )
        {
            position_counter[ index ] = 0;
        }

        gold = GOLD_POSITIONS.UNKNOWN_POS;

        while ( attempts < 4 )
        {
            if ( GoldIsFound() )
            {
                // When gold is found, it indicates the gold mineral is in the right
                // or middle position. Pick right or middle based on Y value.
                if ( GoldYPosition() <= UPPER_Y_LIMIT_FOR_RIGHT )
                {
                    position_counter[ GOLD_POSITIONS.RIGHT_POS.ordinal() ]++;
                    if ( position_counter[ GOLD_POSITIONS.RIGHT_POS.ordinal() ] >= 2 )
                    {
                        gold = GOLD_POSITIONS.RIGHT_POS;
                        break;
                    }
                }
                else
                {
                    position_counter[ GOLD_POSITIONS.MID_POS.ordinal() ]++;
                    if ( position_counter[ GOLD_POSITIONS.MID_POS.ordinal() ] >= 2 )
                    {
                        gold = GOLD_POSITIONS.MID_POS;
                        break;
                    }
                }
            }
            else
            {
                // Gold was not found, so assume Left position (not in phone's field of view)
                position_counter[ GOLD_POSITIONS.LEFT_POS.ordinal() ]++;
                if ( position_counter[ GOLD_POSITIONS.LEFT_POS.ordinal() ] >= 2 )
                {
                    gold = GOLD_POSITIONS.LEFT_POS;
                    break;
                }
            }

            sleep( 100 );
            attempts++;
        }
    }




    // Method to move to gold mineral position
    protected void GoToGold( )
    {
        // Position the arm to prepare for pushing the gold mineral
        arm.position_wrist( Arm.WRIST_POS.LOAD, WRIST_SPEED );

        // If using strafe, then uncomment and face straight to mid here
        // go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, GO_TO_GOLD_ROTATE_TO_MID );

        // Movements specific to mineral sampling position
        switch ( gold )
        {
            case LEFT_POS:
                // First rotate to face left
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, GO_TO_GOLD_ROTATE_TO_LEFT_DEG );
                arm.WaitForInPos();

                // Alternative to use strafe, if angles are an issue
                // go.AutonMove( Drive.DIRECTION.LEFT, GO_TO_GOLD_SIDEWAYS_LEFT_IN );

                // Move into mineral
                go.AutonMove( Drive.DIRECTION.FORWARD, GO_TO_GOLD_FWD_LEFT_IN );

                break;

            case MID_POS:
            case UNKNOWN_POS:   // for unknown gold mineral position, assume middle
                // First rotate to face middle
                go.AutonMoveRotate( Drive.ROTATION.COUNTERCLOCKWISE, GO_TO_GOLD_ROTATE_TO_MID_DEG );
                arm.WaitForInPos();

                // Alternative to use strafe, if angles are an issue
                // Don't need to strafe for middle

                // Move into mineral
                go.AutonMove( Drive.DIRECTION.FORWARD, GO_TO_GOLD_FWD_MID_IN );

                break;

            case RIGHT_POS:
                // First rotate to face left
                go.AutonMoveRotate( Drive.ROTATION.CLOCKWISE, GO_TO_GOLD_ROTATE_TO_RIGHT_DEG );
                arm.WaitForInPos();

                // Alternative to use strafe, if angles are an issue
                // go.AutonMove( Drive.DIRECTION.RIGHT, GO_TO_GOLD_SIDEWAYS_RIGHT_IN );

                // Move into mineral
                go.AutonMove( Drive.DIRECTION.FORWARD, GO_TO_GOLD_FWD_RIGHT_IN );

                break;
        }

        // Movements common to all sampling positions

        // Swipe to the right to sweep mineral out of position
        go.AutonMoveRotate( Drive.ROTATION.CLOCKWISE, ROTATE_TO_KNOCK_OFF_SAMPLE_DEG );

        // Raise the arm to be clear of minerals
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
                    //go.AutonMove( Drive.DIRECTION.RIGHT, GO_TO_GOLD_SIDEWAYS_IN );
                    break;

                case MID_POS:
                    break;

                case RIGHT_POS:
                    //go.AutonMove( Drive.DIRECTION.LEFT, GO_TO_GOLD_SIDEWAYS_IN );
                    break;
            }
        }
    }


    // The drive to depot method is expected to be different for each
    // position, so each auton routine must override it.
    abstract protected void DriveToDepot( );


    // Common method to unload marker and gold sample
    protected void UnloadGoldAndMarker( )
    {
        // arm should already be in correct position

        // Unload gold sample and adjust robot rotation for backing into crater.
        loader.AutonUnload();
        go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, 8);
    }


    // Method to get to park position
    protected void ParkTheRobot( )
    {
        // Raise arm to move position?
        go.AutonMove( Drive.DIRECTION.REVERSE, PARK_DISTANCE_IN );

    }

    public void GyroCorrect ( )
    {
        double angle = gyro.GetZAngle() - startingZAngle;

        if ( Math.abs( angle ) > 4.0 )
        {
            if ( angle > 0.0 )
            {
                go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, angle);
            }
            else
            {
                go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, -angle);
            }
        }
    }

}