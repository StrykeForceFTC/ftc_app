package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 *
 * Auton Program mainly here to test Auton Drive and other Functions in
 *
 */
@Autonomous(name = "Facing Crater", group = "Linear Opmode")
// @Disabled
public class Auton_Facing_Crater extends AutonLinearBase
{
    // Hardware objects are created in AutonLinearBase

    // Enumeration for auton steps allowing move to next step
    // using Next
    private enum AUTON_STEPS
    {
        LOWER_ROBOT,
        DISCONNECT_FROM_LANDER,
        // Etc.
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

    private AUTON_STEPS step = AUTON_STEPS.LOWER_ROBOT;

    /*
     * There is only runOpMode for linear op modes
     */
    @Override
    public void runOpMode()
    {
        /*
         * Initialize the hardware variables.
         */
        InitHardware();

        // Wait hit till start button pressed
        waitForStart();
        runtime.reset();

        // Loop until stop or forced to end
        while ( opModeIsActive() )
        {
            // Do steps here (big switch case)
            switch ( step )
            {
                case LOWER_ROBOT:
                {
                    LowerRobot();
                    break;
                }

                case DISCONNECT_FROM_LANDER:
                {
                    DisconnectFromLander();
                    break;
                }

                default:  // Stop case
                {
                    StopActions();
                }
            }

            step = step.Next();

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

            idle();
        }

    }

    protected void LowerRobot( )
    {
        // do whatever to lower robot
    }

    protected void DisconnectFromLander( )
    {
        // do whatever to disconnect from lander
    }

}


