package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by gstaats on 18/09/17.
 */

public class Drive {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;

    /* Constructor */
    public Drive(){

            // Define and Initialize Motors
            frontLeft = HardwareMap(DcMotor.class, "front_left");
            frontRight = HardwareMap(DcMotor.class, "front_right");
            rearLeft = HardwareMap(DcMotor.class, "rear_left");
            rearRight = HardwareMap(DcMotor.class, "rear_right");

            frontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            frontRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            rearLeft.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            rearRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


        // Set all motors to zero power
            frontLeft.setPower(0);
           frontRight.setPower(0);
            rearRight.setPower(0);
            rearLeft.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Define and initialize ALL installed servos.
            leftClaw = hwMap.get(Servo.class, "left_hand");
            rightClaw = hwMap.get(Servo.class, "right_hand");
            leftClaw.setPosition(MID_SERVO);
        };

    }

    // function to move forward
    public void forward( double power, double negativepower )
    {
        frontLeft.setPower(negativepower);
        rearLeft.setPower(power);
        frontRight.setPower(negativepower);
        rearRight.setPower(power);
    }

    // function to move backward
    public void backward(double negativepower, double power)
    {
        frontLeft.setPower(negativepower);
        frontRight.setPower(power);
        rearRight.setPower(power);
        rearLeft.setPower(negativepower);

    }

    // function to move left (not turn!)
    public void left(double power, double negativepower )
    {
      frontLeft.setPower(negativepower);
      rearLeft.setPower(power);
      rearRight.setPower(power);
      frontRight.setPower(negativepower);

    }

    // function to move right (not turn!)
    public void right(double power, double negativepower)
    {
      frontLeft.setPower(power);
        rearLeft.setPower(negativepower);
        rearRight.setPower(negativepower);
        frontRight.setPower(power);
    }

    // function to move diagonal
    public void diagonal(double power1, double power2)
    {
        frontLeft.setPower(power1);
        rearRight.setPower(power1);
        rearRight.setPower(-power2);
        rearLeft.setPower(-power2);
    }

    // function to turn
    public void turn( double signedSpeed )
    {
        if ( signedSpeed <= 0.0 )
        {
            // when speed is < 0, rotate CCW. To rotate CCW
            // set ..... <this is not correct, yet>
            double power = -1.0 * signedSpeed;

            frontLeft.setPower(signedSpeed);
            rearLeft.setPower(power);
            frontRight.setPower(power);
            rearRight.setPower(signedSpeed);
        }
    }
//assuming that we are turning in one direction
    //this turns  counterclockwise
}

