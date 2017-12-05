package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.JoystickUtilities.Delay_ms;

/**
 * Created by gstaats on 21/09/17.
 */

// Pole Class
public class Pole {

    // extends\retracts the pole
    private DcMotorSimple far;
    private Servo relicClaw;
    // lifts\lowers the pole
    private DcMotor high;
    //extends the pole
    private double farpower = 0.0;

    private static final double incr_filter = 0.4;
    private static final double decr_filter = 0.7;

    public Pole( HardwareMap ahwMap )
    {
        // Hardware Map
        far = ahwMap.dcMotor.get( "pole_extend" );
        high = ahwMap.dcMotor.get( "pole_lift" );
        relicClaw = ahwMap.servo.get( "relicClaw");

        // Motor setup
        far.setDirection( DcMotorSimple.Direction.FORWARD );
        high.setDirection( DcMotorSimple.Direction.FORWARD );
        high.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        high.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

        // Claw setup
        relicClaw.setDirection(Servo.Direction.FORWARD);
    }


    public void extend( )
    {
        farpower = JoystickUtilities.LowPassFilter( farpower, 0.5, incr_filter, decr_filter );
        far.setPower(farpower);
    }

    //retracts the pole
    public void retract()
    {
        farpower = JoystickUtilities.LowPassFilter(farpower, -0.5, incr_filter, decr_filter);
        far.setPower(farpower);
    }

    // Leaves the pole in place
    public void stay()
    {
        farpower = JoystickUtilities.LowPassFilter(farpower, 0.0, incr_filter, decr_filter);
        if(Math.abs(farpower)<0.03)
        {
            farpower = 0;
        }
        far.setPower(0);
    }

    public void retractFast()
    {
        farpower = JoystickUtilities.LowPassFilter(farpower, -0.75, incr_filter, decr_filter);
        far.setPower(farpower);
    }

    public void extendFast( )
    {
        farpower = JoystickUtilities.LowPassFilter( farpower, 0.75, incr_filter, decr_filter );
        far.setPower(farpower);
    }

    //lifts s the pole
    public void lift( double upDown )
    {
        high.setPower(0.35*upDown);
    }

    public void autonLift( )
    {
        // Lift relic pole out of the way
        high.setPower( 0.35 );
        Delay_ms( 250.0 );
        high.setPower( 0.0 );
    }

    //lowers the pole
    public void lower()
    {
        high.setPower(-1);
    }

    public void liftstop()
    {
        high.setPower(0);
    }

    public int GetPosition( )
    {
        return high.getCurrentPosition();
    }

    public void openClaw( )
    {
        relicClaw.setPosition(1);
    }

    public void closeClaw ( )
    {
      relicClaw.setPosition(0);
    }
    public double clawPosition ()
    {
        return relicClaw.getPosition();
    }
}
