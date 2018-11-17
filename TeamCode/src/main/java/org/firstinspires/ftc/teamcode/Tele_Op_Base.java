/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an iterative (Non-Linear) "OpMode".
 * This op mode was created to allow testing of the robot hardware and
 * software controls. It is easier to debug here than having to
 * comment out code in the standard teleop
 *
 */

// @TeleOp(name="Tele Op Test", group="Iterative Opmode")
// @Disabled
public abstract class Tele_Op_Base extends OpMode
{
    // Declare HW objects
    public Drive go = null;
    public Loader loader = null;
    public Auto_Robot_Detect robotDetector = null;
    public Arm arm = null;

    // Joystick input values
    public double robotLeftRight = 0.0;
    public double robotForwardBack = 0.0;
    public double robotRotate = 0.0;

    // Constants for joystick shaping
    private static final double ROBOT_LEFT_RIGHT_WEIGHTING = 0.5;
    private static final double ROBOT_FWD_BACK_WEIGHTING = 0.5;
    private static final double ROBOT_ROTATE_WEIGHTING = 0.5;

    public Auto_Robot_Detect.teamId TeamId = Auto_Robot_Detect.teamId.teamUnknown;
    /*
     * HW initialization code
     */
    public void HwInit()
    {
        /*
        ** Initialize the hardware variables and determine team / robot.
        */
        robotDetector = new Auto_Robot_Detect( hardwareMap );
        TeamId = robotDetector.TeamId;
        go = new Drive( hardwareMap );
        loader = new Loader( hardwareMap );
        arm = new Arm( hardwareMap );
    }


    /*
     * Code to process gamepad1 joysticks into fwd/back, left/right and rotate
     */
    public void ProcessGamepad1Joysticks()
    {
        // Shape joystick inputs
        robotForwardBack = JoystickUtilities.ShapeCubePlusInputWeighted( -gamepad1.left_stick_y, ROBOT_FWD_BACK_WEIGHTING );
        robotLeftRight = JoystickUtilities.ShapeCubePlusInputWeighted( gamepad1.left_stick_x, ROBOT_LEFT_RIGHT_WEIGHTING );
        robotRotate = JoystickUtilities.ShapeCubePlusInputWeighted( gamepad1.right_stick_x, ROBOT_ROTATE_WEIGHTING );

       if( ( Math.abs(robotForwardBack) > 0.05 ) || ( Math.abs(robotLeftRight) > 0.05 ))
       {
           robotRotate = robotRotate * 0.5;
       }
    }

    public void ProcessLoadingInput()
    {
        if ((gamepad2.left_trigger > 0.1) || (gamepad2.left_bumper ))
        {

            loader.TeleopUnloadSamples();
        }
        else if (( gamepad2.right_bumper ) || (gamepad2.right_trigger > 0.1))
        {
            loader.TeleopLoad();
        }
        else
        {
            loader.teleopstop();
        }
    }

    public void ProcessRaiseArm()
    {
        /* DISABLE DPAD BASED CONTROLS
        if ( gamepad2.dpad_up )    // Gamepad2, DPAD Up is for unload position
        {
            arm.position_lift( Arm.lift_pos.hook_lander, 9 );
            arm.position_wrist( Arm.WRIST_POS.UNLOAD, 9 );
        }
        else if ( gamepad2.dpad_down )
        {
            // Gamepad2, DPAD down is for load position
            arm.position_lift( Arm.lift_pos.fulldown, 9 );
            arm.position_wrist( Arm.WRIST_POS.LOAD, 9 );
        }
        else if ( gamepad2.dpad_right )
        {
            // Gamepad2, DPAD right is for travel position
            arm.position_lift( Arm.lift_pos.fulldown, 9 );
            arm.position_wrist( Arm.WRIST_POS.MOVE, 9 );
        }
        else if ( gamepad2.dpad_left )
        {
            // Gamepad2, DPAD down is for load position
            arm.position_lift( Arm.lift_pos.hook_lander, 9 );
            arm.position_wrist( Arm.WRIST_POS.START, 9 );
        }
        */

        if (gamepad2.left_stick_y <= -0.1)
        {
            arm.adjust_lift( Arm.lift_dir.up, gamepad2.left_stick_y * -1.0, 6 );
        }
        else if (gamepad2.left_stick_y >= 0.1)
        {
            arm.adjust_lift( Arm.lift_dir.down, gamepad2.left_stick_y * 1, 6 );
        }

        if ( gamepad2.right_stick_x >= 0.1 )
        {
            arm.adjust_wrist( Arm.WRIST_DIR.FORWARD, gamepad2.right_stick_x, 9 );
        }
        else if (gamepad2.right_stick_x <= -0.1)
        {
            arm.adjust_wrist( Arm.WRIST_DIR.BACKWARD, -gamepad2.right_stick_x, 9 );
        }

    }


    /*
     * The stop method is created in the base class to ensure it is consistent
     * across all teleop modes. Want to ensure hardware is "off" and in a safe
     * state when stopping robot.
     */
    @Override
    public void stop()
    {
        // Stop the arm
        arm.Stop();

        // Turn loader off
        loader.teleopstop();

        // Make sure Robot stops
        go.MoveSimple( 0.0, 0.0, 0.0 );
    }


}
