package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by gstaats on 18/09/17.
 */

public class Drive
{
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;
    HardwareMap hwMap = null;

    public Drive(DcMotor FL, DcMotor FR, DcMotor RL, DcMotor RR )
    {
        // Define and Initialize Motors

        frontRight = FR;
        frontLeft  = FL;
        rearRight  = RR;
        rearLeft   = RL;

        // Set direction so positive is always forward with respect to
        // the robot. Right side motors need to be set to reverse, because
        // they spin counter-clockwise to move the robot forward.
        frontRight.setDirection( DcMotor.Direction.FORWARD );
        rearRight.setDirection( DcMotor.Direction.FORWARD );
        frontLeft.setDirection( DcMotor.Direction.REVERSE );
        rearLeft.setDirection( DcMotor.Direction.REVERSE );

        // Set all motors to zero power. Don't want robot moving till
        // we're ready.
        frontLeft.setPower( 0 );
        frontRight.setPower( 0 );
        rearLeft.setPower( 0 );
        rearRight.setPower( 0 );

        // Set all motors to run with encoders.

        frontLeft.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        frontRight.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rearLeft.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        rearRight.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        // Set motors to not brake
        frontLeft.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
        frontRight.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
        rearLeft.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
        rearRight.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );

    }
    public Drive( HardwareMap ahwMap )
    {
        hwMap = ahwMap;

        // Define and Initialize Motors

        frontRight = hwMap.dcMotor.get("front_right");
        frontLeft  = hwMap.dcMotor.get("front_left");
        rearRight  = hwMap.dcMotor.get("rear_right");
        rearLeft   = hwMap.dcMotor.get("rear_left");

        // Set direction so positive is always forward with respect to
        // the robot. Right side motors need to be set to reverse, because
        // they spin counter-clockwise to move the robot forward.
        frontRight.setDirection( DcMotor.Direction.FORWARD );
        rearRight.setDirection( DcMotor.Direction.FORWARD );
        frontRight.setDirection( DcMotor.Direction.REVERSE );
        rearRight.setDirection( DcMotor.Direction.REVERSE );

        // Set all motors to zero power. Don't want robot moving till
        // we're ready.
        frontLeft.setPower( 0 );
        frontRight.setPower( 0 );
        rearLeft.setPower( 0 );
        rearRight.setPower( 0 );

        // Set all motors to run with encoders.

        frontLeft.setMode( DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        rearLeft.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        rearRight.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
    }
/*
    public void vector(double driveY, double driveX)
    {
        if (driveY > 0)
        {

        } else if (driveY < 0)
        {

        }

        if (driveX > 0)
        {

        } else if (driveX < 0)
        {

        }
    }
 */

    // function to move forward
    public void forward( double power )
    {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
    }

    // function to move forward
    public void forward( )
    {
        frontLeft.setPower(1);
        frontRight.setPower(1);
        rearLeft.setPower(1);
        rearRight.setPower(1);
    }


    // function to move backward
    public void backward( double power)
    {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
        rearLeft.setPower(power);

    }
    // function to move backward
    public void backward()
    {
        frontLeft.setPower(-1);
        frontRight.setPower(-1);
        rearLeft.setPower(-1);
        rearRight.setPower(-1);
    }


    // function to move left (not turn!)
    public void turn_left()
    {
        frontLeft.setPower(-1);
        frontRight.setPower(1);
        rearLeft.setPower(-1);
        rearRight.setPower(1);
    }

    // function to move right (not turn!)
    public void turn_right()
    {
        frontLeft.setPower(1);
        frontRight.setPower(-1);
        rearLeft.setPower(1);
        rearRight.setPower(-1);
    }

    // function to move diagonal 45 degrees from the front of the robot
    public void diagonal45()
    {
        frontRight.setPower(1);
        rearLeft.setPower(1);

    }
    //function to move 225 degrees
    public void diagonal225()
    {
        frontRight.setPower(-1);
        rearLeft.setPower(1);
    }

    public void diagonal135()
    {
        frontLeft.setPower(1);
        rearRight.setPower(1);

    }
    public void diagonal315()
    {frontLeft.setPower(1);

        rearRight.setPower(-1);
    }

    // function to move left (not turn!)
    public void left(double power  )
    {
      frontLeft.setPower(power);
      rearLeft.setPower(-power);
      rearRight.setPower(-power);
      frontRight.setPower(power);

    }

    // function to go left
    public void left()
    { frontLeft.setPower(1);
        frontRight.setPower(-1);
        rearLeft.setPower(-1);
        rearRight.setPower(1);
    }

    // function to move right (not turn!)
    public void right(double power)
    {
      frontLeft.setPower(-power);
        rearLeft.setPower(power);
        rearRight.setPower(power);
        frontRight.setPower(-power);
    }

    //function to go left
    public void right() {
        frontLeft.setPower(-1);
        frontRight.setPower(1);
        rearLeft.setPower(1);
        rearRight.setPower(-1);
    }

    // function to move diagonal
    public void diagonal( double power )
    {
        frontLeft.setPower( power );
        rearRight.setPower( power );
        rearRight.setPower( -power );
        rearLeft.setPower( -power );
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
}

