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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 */

@TeleOp(name="Tele Op 8553 ", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
// @Disabled
public class Tele_Op_8553 extends OpMode
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

    //Variables for drive
    double drive_x = 0;
    double drive_y = 0;
    double drive_turn = 0;

    //Variables for wep.lift
    double poleRotate = 0;

    //Constants for joystick shaping, gamepad2
    private static final double ROBOT_POLE_ROTATE_WEIGHTING = 0.5;

    //Constants for joystick shaping, gamepad1
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


        knockerServo = hardwareMap.servo.get("knocker_servo");
        colorSensor = hardwareMap.colorSensor.get("color");


        go = new Drive(frontLeft, frontRight, rearLeft, rearRight);

        jewelKnocker = new JewelKnocker( knockerServo, colorSensor );


        // Set up Claw
        claw = new Claw( hardwareMap );

        // Set up Pole
        wep = new Pole( hardwareMap );

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
        jewelKnocker.RaiseKnocker8553();
        runtime.reset();
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
        telemetry.addLine("right joystick | ")
                .addData("x", gamepad1.right_stick_x)
                .addData("y", gamepad1.right_stick_y);

        // Show joystick information as some other illustrative data
        telemetry.addLine("left joystick2 | ")
                .addData("x", gamepad2.left_stick_x)
                .addData("y", gamepad2.left_stick_y);
        telemetry.addLine("right joystick2 | ")
                .addData("x", gamepad2.right_stick_x)
                .addData("y", gamepad2.right_stick_y);
       /*
        telemetry.addLine("Knocker Positon | ")
                .addData( "Pos", jewelKnocker.KnockerPositionGet() );
        telemetry.addLine("Color Values | ")
                .addData("Red", jewelKnocker.RedValue())
                .addData("Blue", jewelKnocker.BlueValue());
        */
        telemetry.addLine("Claw Positions | ")
                .addData( "Right", rightClawPosition )
                .addData( "Left",leftClawPosition );

        poleRotate = (Math.pow(gamepad2.right_stick_x, 3.0)*ROBOT_POLE_ROTATE_WEIGHTING)+(gamepad2.right_stick_y*(1-ROBOT_POLE_ROTATE_WEIGHTING));
        wep.lift(poleRotate);
        // Use gamepad Y & A raise and lower the arm
        //wep.lift( gamepad2.right_stick_x );


        lift.Raise(gamepad2.left_stick_y);

        // Use gamepad X & B to extend and retract the arm
        if (gamepad2.dpad_up) {
            wep.extend();
        }
        else if (gamepad2.dpad_down) {
            wep.retract();
        }
        else {
            wep.stay();
        }
        // Move robot based on joystick inputs from gamepad 1 / driver 1
        drive_x = (Math.pow(gamepad1.left_stick_x, 3.0)*ROBOT_LEFT_RIGHT_WEIGHTING)+(gamepad1.left_stick_x*(1-ROBOT_LEFT_RIGHT_WEIGHTING));
        drive_y = (Math.pow(gamepad1.left_stick_y, 3.0)*ROBOT_FWD_BACK_WEIGHTING)+(gamepad1.left_stick_y*(1-ROBOT_FWD_BACK_WEIGHTING));
        drive_turn = (Math.pow(gamepad1.right_stick_x, 3.0)*ROBOT_ROTATE_WEIGHTING)+(gamepad1.right_stick_x*(1-ROBOT_ROTATE_WEIGHTING));
        go.MoveSimple( drive_x, drive_y, drive_turn );


        /*Drive code without weighting
        go.MoveSimple( gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x );
        */


        // ******* Test code for JewelKnocker ***********
        /*
        // Gamepad1.x used to increase jewel knocker position
        if ( gamepad1.x )
        {
            double position = jewelKnocker.KnockerPositionGet();
            if ( position <= 0.9 ) {
                jewelKnocker.KnockerPositionSet(position + 0.1);
            }
        }

        // Gamepad1.y to decrease jewel knocker position
        if ( gamepad1.y )
        {
            double position = jewelKnocker.KnockerPositionGet();
            if ( position >= 0.1 ) {
                jewelKnocker.KnockerPositionSet(position - 0.1);
            }
        }
        */

        // ************* Test code for drive auto methods **************
        /*
        if ( gamepad1.a )
        {
            go.AutonForward( 30.0 );
        }

        if ( gamepad1.b )
        {
            go.AutonReverse( 30.0 );
        }

        if ( gamepad1.x )
        {
            go.AutonRotateClockwise( 90.0 );
        }

        if ( gamepad1.y )
        {
            go.AutonRotateCounterclockwise( 90.0 );
        }
        */
       if (gamepad2.right_bumper) { wep.openClaw(); }
       if (gamepad2.left_bumper) { wep.closeClaw(); }
        // ************* Test code for Claw methods **************

        if ( gamepad1.x )
        {
            claw.claw_Inward8553();
        }

        if ( gamepad1.y )
        {
            claw.claw_Outward8553();
        }


        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}