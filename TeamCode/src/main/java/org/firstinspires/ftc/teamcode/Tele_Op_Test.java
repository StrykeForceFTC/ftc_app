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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file contains an iterative (Non-Linear) "OpMode".
 * This op mode was created to allow testing of the robot hardware and
 * software controls. It is easier to debug here than having to
 * comment out code in the standard teleop
 *
 */

@TeleOp(name="Tele Op Test", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
// @Disabled
public class Tele_Op_Test extends OpMode
{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Drive hardware
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;

    // Jewel Knocker hardware
    private Servo knockerServo = null;
    private ColorSensor colorSensor = null;

    HardwareMap robotMap = hardwareMap;
    private Drive go = null;
    private Pole wep = null;
    private Claw claw = null;
    private Lift lift = null;
    private JewelKnocker jewelKnocker = null;

    // Joystick input values
    private double robotLeftRight = 0.0;
    private double robotForwardBack = 0.0;
    private double robotRotate = 0.0;

    // Constants for joystick shaping
    private static final double ROBOT_LEFT_RIGHT_WEIGHTING = 0.5;
    private static final double ROBOT_FWD_BACK_WEIGHTING = 0.5;
    private static final double ROBOT_ROTATE_WEIGHTING = 0.5;

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
        frontLeft  = hardwareMap.dcMotor.get("front_left");
        frontRight  = hardwareMap.dcMotor.get("front_right");
        rearLeft  = hardwareMap.dcMotor.get("rear_left");
        rearRight  = hardwareMap.dcMotor.get("rear_right");

        go = new Drive(frontLeft, frontRight, rearLeft, rearRight);

        knockerServo = hardwareMap.servo.get("knocker_servo");
        colorSensor = hardwareMap.colorSensor.get("color");
        jewelKnocker = new JewelKnocker( knockerServo, colorSensor );

        // Set up Claw
        claw = new Claw( hardwareMap );

        // Set up pole
        wep = new Pole( hardwareMap );

        // Set up lift
        lift = new Lift( hardwareMap );

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        jewelKnocker.RaiseKnocker();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        double leftClawPosition = claw.GetLeftPosition();
        double rightClawPosition = claw.GetRightPosition();

        // Show joystick information as some other illustrative data
        telemetry.addLine("left joystick | ")
                .addData("x", gamepad1.left_stick_x)
                .addData("y", gamepad1.left_stick_y);
/*
        telemetry.addLine("right joystick | ")
                .addData("x", gamepad1.right_stick_x)
                .addData("y", gamepad1.right_stick_y);

        telemetry.addLine("Knocker ")
                .addData( "Position", knockerServo.getPosition() );

        telemetry.addLine("Color Values | ")
                .addData("Red", jewelKnocker.RedValue())
                .addData("Blue", jewelKnocker.BlueValue());

        telemetry.addLine("Claw Positions | ")
                .addData( "Right", rightClawPosition )
                .addData( "Left", leftClawPosition );
        telemetry.addLine("WEP ")
                .addData("Position", wep.GetPosition() );
        telemetry.addLine("Lift" )
                .addData( "Position ", lift.GetPosition() );
        telemetry.addLine("relic ")
                .addData("Position ", wep.clawPosition());
*/

        // ************* Test code for pole **************
        // Use gamepad Y & A raise and lower the arm
/*
        wep.lift( gamepad2.right_stick_y );

        // Use gamepad2 D padd to extend and retract the arm
        if (gamepad2.dpad_up) {
            wep.extend();
        }
        else if (gamepad2.dpad_down) {
            wep.retract();
        }
        else
        {
            wep.stay();
        }
*/

        // Move robot based on joystick inputs from gamepad 1 / driver 1
        // shape joystick inputs
        robotForwardBack = JoystickUtilities.ShapeCubePlusInputWeighted( gamepad1.left_stick_y, ROBOT_FWD_BACK_WEIGHTING );
        robotLeftRight = JoystickUtilities.ShapeCubePlusInputWeighted( gamepad1.left_stick_x, ROBOT_LEFT_RIGHT_WEIGHTING );
        robotRotate = JoystickUtilities.ShapeCubePlusInputWeighted( gamepad1.right_stick_x, ROBOT_ROTATE_WEIGHTING );
        go.MoveSimple( robotLeftRight, robotForwardBack, robotRotate );

        // ************* Test code for auton Drive methods **************
        // Gamepad1.x moves forward 20cm
        if ( gamepad1.x )
        {
            go.AutonForward( 20.0 );
            while ( gamepad1.x )
            {

            }
        }

        // Gamepad1.y moves backwards 20cm
        if ( gamepad1.y )
        {
            go.AutonReverse( 20.0 );
            while ( gamepad1.y )
            {

            }
        }

        // Gamepad1.a moves forward 10cm
        if ( gamepad1.a )
        {
            go.AutonForward( 10.0 );
            while ( gamepad1.a )
            {

            }
        }

        // Gamepad1.y moves backwards 10cm
        if ( gamepad1.b )
        {
            go.AutonReverse( 10.0 );
            while ( gamepad1.b )
            {

            }
        }


        // ************* Test code for lift **************
        // lift.Raise( gamepad2.left_stick_y );


        //testcode for relic
/*
        if ( gamepad2.left_bumper)
        {
            wep.closeClaw();
        }

        if (gamepad2.right_bumper)
        {
         wep.openClaw();
        }
*/

        telemetry.addLine("Encoders ")
                .addData("FL ", frontLeft.getCurrentPosition() )
                .addData("FR ", frontRight.getCurrentPosition() );
/*
        telemetry.addLine("Lift")
                .addData("enc ", lift.GetPosition());
*/

        telemetry.update();
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        jewelKnocker.RaiseKnocker();
    }

}
