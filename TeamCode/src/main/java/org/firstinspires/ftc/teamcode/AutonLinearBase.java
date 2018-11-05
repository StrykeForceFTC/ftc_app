package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

    public ElapsedTime runtime = new ElapsedTime();

    // Declare HW objects
    public Drive go = null;
    public Loader loader = null;
    public Auto_Robot_Detect robotDetector = null;

    // Detectors
    private GoldAlignDetector detector;

    public Auto_Robot_Detect.teamId TeamId = Auto_Robot_Detect.teamId.teamUnknown;

    // Method to initialize any connected hardware
    public void InitHardware( )
    {
        // Find what robot you are running and set up hardware
        robotDetector = new Auto_Robot_Detect( hardwareMap );
        go = new Drive( hardwareMap );
        loader = new Loader( hardwareMap );

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

}