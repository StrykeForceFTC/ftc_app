package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by jscott on 11/10/17.
 */

// Lift Class
public class Lift {

    // Raises/lowers the scissor lift
    private DcMotor liftMotor;

    // Timer for limiting amount of time raising/lowering in Auton
    ElapsedTime limitTimer = new ElapsedTime();

    // Constants for raising / lowering in auton
    final private static int TICKS_TO_RAISE = 100;
    final private static int TICKS_PER_REV = 1120;

    public Lift(HardwareMap ahwMap )
    {
        // Hardware Map
        liftMotor = ahwMap.dcMotor.get( "claw_lift" );

        // Set direction and off operation for each motor
        liftMotor.setDirection( DcMotor.Direction.FORWARD );
        liftMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        // Set up motors. Far is just a simple 12V DC motor and
        // high is a NeverRest 40
        liftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
    }

    public void Raise( double raiseLower )
    {
        liftMotor.setPower(raiseLower);
    }

    // Method to raise the lift a set amount in auton mode
    public void AutonRaise( )
    {
        int encoderLast = liftMotor.getCurrentPosition();
        int ticksMoved = 0;

        Raise(1.0);
        limitTimer.reset();
        while ( ( ticksMoved < TICKS_TO_RAISE ) && ( limitTimer.time() < 3 ) )
        {
            int encoderNow = liftMotor.getCurrentPosition();
            if ( encoderNow >= encoderLast )
            {
                ticksMoved += encoderNow - encoderLast;
            }
            else
            {
                // encoder must have rolled over, so calculate how far moved before and after
                // rolling over
                ticksMoved += encoderNow + ( TICKS_PER_REV - encoderLast );
            }

            // Update last position for next loop
            encoderLast = encoderNow;
        }

        // Stop moving
        Raise( 0.0 );
    }

    // Method to lower the lift a set amount in auton mode
    public void AutonLower( )
    {
        int encoderLast = liftMotor.getCurrentPosition();
        int ticksMoved = 0;

        // Lowering 40 fewer ticks than raising just makes sure we don't
        // crash the lift and also limit time to less
        Raise(-1.0);
        limitTimer.reset();
        while ( ( ticksMoved < ( TICKS_TO_RAISE - 40 ) ) && ( limitTimer.time() < 2.8 ) )
        {
            int encoderNow = liftMotor.getCurrentPosition();
            if ( encoderNow <= encoderLast )
            {
                ticksMoved += encoderLast - encoderNow;
            }
            else
            {
                // encoder must have underflowed, so calculate how far moved before and after
                // underflowing
                ticksMoved += encoderLast + ( TICKS_PER_REV - encoderNow );
            }

            // Update last position for next loop
            encoderLast = encoderNow;
        }

        // Stop moving
        Raise( 0.0 );
    }

    public int GetPosition( )
    {
        return liftMotor.getCurrentPosition();
    }
}
