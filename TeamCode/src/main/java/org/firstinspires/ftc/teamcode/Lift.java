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

    // Constants for lift input filtering. Make start up softer
    // than stop / reducing power.
    final private static double INCREASING_FILTER_K = 0.3;
    final private static double DECREASING_FILTER_K = 0.6;

    // Constants for raising / lowering in auton. Lower for less time than
    // we raise to avoid bottoming out the motor. Delay is actually done in
    // shorter segments to allow ramping of motor power.
    final private static double AUTON_LIFT_POWER = 0.5;
    final private static double AUTON_RAISE_TIME_MS = 400.0;
    final private static double AUTON_LOWER_TIME_MS = AUTON_RAISE_TIME_MS - 80.0;
    final private static double AUTON_TIME_PER_PASS_MS = 20.0;
    final private static int AUTON_LOOPS_4_RAISE = (int) ( AUTON_RAISE_TIME_MS / AUTON_TIME_PER_PASS_MS );
    final private static int AUTON_LOOPS_4_LOWER = (int) ( AUTON_LOWER_TIME_MS / AUTON_TIME_PER_PASS_MS );

    // Lift motor power - must be a class member to allow filtering
    private double liftMotorPower = 0.0;

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
        liftMotorPower = JoystickUtilities.LowPassFilter( liftMotorPower, raiseLower, INCREASING_FILTER_K, DECREASING_FILTER_K );
        RaiseUnfiltered( liftMotorPower );
    }

    private void RaiseUnfiltered( double liftPower )
    {
        liftMotor.setPower( liftPower );
    }

    // Method to raise the lift a set amount in auton mode
    public void AutonRaise( )
    {
        double liftPower = 0.0;
        for (int loop = 0; loop < AUTON_LOOPS_4_RAISE; loop++)
        {
            // Ramp power up
            liftPower = RampPower( AUTON_LIFT_POWER, liftPower );

            // To raise, power is negative
            RaiseUnfiltered( -liftPower );

            // Delay is used to give time for motor to go
            // to new power and to control over all raise time
            JoystickUtilities.Delay_ms( AUTON_TIME_PER_PASS_MS );
        }

        RaiseUnfiltered( 0.0 );
    }

    // Method to lower the lift a set amount in auton mode
    public void AutonLower( )
    {
        double liftPower = 0.0;
        for (int loop = 0; loop < AUTON_LOOPS_4_LOWER; loop++)
        {
            // Ramp power up
            liftPower = RampPower( AUTON_LIFT_POWER, liftPower );

            // To lower, power is positive
            RaiseUnfiltered( liftPower );

            // Delay is used to give time for motor to go
            // to new power and to control over all raise time
            JoystickUtilities.Delay_ms( AUTON_TIME_PER_PASS_MS );
        }

        RaiseUnfiltered( 0.0 );
    }

    // Method to ramp power. Adds in 20% each pass
    private double RampPower( double target, double now )
    {
        double updated = now + 0.2 * target;
        if ( updated > target )
        {
            updated = target;
        }

        return updated;
    }

    public int GetPosition( )
    {
        return liftMotor.getCurrentPosition();
    }
}
