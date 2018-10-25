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


    // Joystick input values
    public double robotLeftRight = 0.0;
    public double robotForwardBack = 0.0;
    public double robotRotate = 0.0;

    // Constants for joystick shaping
    private static final double ROBOT_LEFT_RIGHT_WEIGHTING = 0.5;
    private static final double ROBOT_FWD_BACK_WEIGHTING = 0.5;
    private static final double ROBOT_ROTATE_WEIGHTING = 0.5;

    //enum for detection
    public enum teamId
    {
       teamUnknown, team8553, team7228, team15106;
    }

    public teamId TeamId = teamId.teamUnknown;
    /*
     * HW initialization code
     */
    public void HwInit()
    {
        /* Initialize the hardware variables.
         * TODO: Detect robot here and select appropriate HW
         */
        //
        WhoAmI();
        go = new Drive( hardwareMap );



        // TODO: use telemetry to put team on phone
    }

    //This looks for the name in the team so, if team is 8553, the name it looks for is 8553.
    //Then it will throw a exception if it was false, which is caught by the catch exception and is made as false in the boolean.
    private boolean AmI(String team )
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







    /*
     * Code to process gamepad1 joysticks into fwd/back, left/right and rotate
     */
    public void ProcessGamepad1Joysticks()
    {
        // Shape joystick inputs
        robotForwardBack = JoystickUtilities.ShapeCubePlusInputWeighted( -gamepad1.left_stick_y, ROBOT_FWD_BACK_WEIGHTING );
        robotLeftRight = JoystickUtilities.ShapeCubePlusInputWeighted( gamepad1.left_stick_x, ROBOT_LEFT_RIGHT_WEIGHTING );
        robotRotate = JoystickUtilities.ShapeCubePlusInputWeighted( gamepad1.right_stick_x, ROBOT_ROTATE_WEIGHTING );
    }

}
