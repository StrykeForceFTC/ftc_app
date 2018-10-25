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

    // Detectors
    private GoldAlignDetector detector;

    //This is where we put in the name of he team.  (At least the enum for it.)
    public enum teamId
    {
        teamUnknown, team8553, team7228, team15106;
    }

    public AutonLinearBase.teamId TeamId = AutonLinearBase.teamId.teamUnknown;

    // Method to initialize any connected hardware
    public void InitHardware( )
    {
        // TODO Determine correct robot and initialize HW modules for it
        //Status: Not tested yet (For Auton)
        //Find what robot you are running
        WhoAmI();
        go = new Drive( hardwareMap );

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

    //
    private boolean AmI(String team )
    //This looks for the name in the team so, if team is 8553, the name it looks for is 8553.
    //Then it will throw a exception if it was false, which is caught by the catch exception and is made as false in the boolean.
    {
        try
        {
            hardwareMap.get(team);
            return true;
        }
        catch ( Exception e)
        {
            return false;
        }

    }

    //This takes the above method and runs it until it returns true.
    //If it returns true, the teamId will change to be given team
    //Ex.  If the 8553 is true, the teamId is set to team8553.
    //If all return false, the team id will stay at default, (team unknown) so that nothing mechanical will break.
    private void WhoAmI()
    {
        if (AmI("8553"))
        {
            TeamId = teamId.team8553;
        }
        else if (AmI("7228"))
        {
            TeamId = teamId.team7228;
        }
        else if (AmI("15106"))
        {
            TeamId = teamId.team15106;
        }

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


}


