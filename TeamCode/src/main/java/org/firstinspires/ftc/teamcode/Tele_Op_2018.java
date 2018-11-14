
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an iterative (Non-Linear) "OpMode".
 * This op mode was created to allow testing of the robot hardware and
 * software controls. It is easier to debug here than having to
 * comment out code in the standard teleop
 *
 */

@TeleOp(name="Tele Op 2018", group="Iterative Opmode")
// @Disabled
public class Tele_Op_2018 extends Tele_Op_Base
{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        // Use base class to init HW
        HwInit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        telemetry.addData("Status ", "Running: " + runtime.toString());

        // Move robot based on joystick inputs from gamepad 1 / driver 1
        // shape joystick inputs
        ProcessGamepad1Joysticks();
        ProcessLoadingInput();
        ProcessRaiseArm();
        go.MoveSimple( robotLeftRight, robotForwardBack, robotRotate );

        //Processes joystick and button values to raise or lower the arm.
        ProcessRaiseArm();

        //! @todo Add Arm controls in

        telemetry.addLine("Encoders ")
                .addData("FL ", go.GetEncoderFrontLeft() )
                .addData("FR ", go.GetEncoderFrontRight() )
                .addData("RL ", go.GetEncoderRearLeft() )
                .addData("RR ", go.GetEncoderRearRight() );
        telemetry.addLine("Team Id"  )
                .addData("team", TeamId.name());

        telemetry.update();
    }

}
