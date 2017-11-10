package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by gstaats on 21/09/17.
 */

// Pole Class
public class Pole {

    // extends\retracts the pole
    private DcMotorSimple far;

    // lifts\lowers the pole
    private DcMotor high;
    //extends the pole

    public Pole( HardwareMap ahwMap )
    {
        // Hardware Map
        far = ahwMap.dcMotor.get( "pole_extend" );
        high = ahwMap.dcMotor.get( "pole_lift" );

        // Motor setup
        far.setDirection( DcMotorSimple.Direction.FORWARD );
        high.setDirection( DcMotorSimple.Direction.FORWARD );
        high.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        high.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
    }

    public void extend( )
    {
        far.setPower(1);
    }

    //retracts the pole
    public void retract()
    {
        far.setPower(-1);
    }

    // Leaves the pole in place
    public void stay()
    {
        far.setPower(0.0);
    }
    //lifts s the pole
    public void lift( double upDown )
    {
        high.setPower(upDown);
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
}
