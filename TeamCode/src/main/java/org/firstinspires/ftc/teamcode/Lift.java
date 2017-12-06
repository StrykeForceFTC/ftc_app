package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    final private static double AUTON_RAISE_TIME_MS = 250.0;
    final private static double AUTON_LOWER_TIME_MS = AUTON_RAISE_TIME_MS - 50.0;

    // Lower limit for lift power
    final private static double LIFT_POWER_LOWER_LIMIT = 0.02;

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

        // If input near 0, then force value to 0 if mag < LIFT_POWER_LOWER_LIMIT
        double liftMagnitude = Math.abs( liftMotorPower );
        if ( ( liftMagnitude <= LIFT_POWER_LOWER_LIMIT ) && ( Math.abs( raiseLower ) <= 0.01 ) )
        {
            liftMotorPower = 0.0;
        }

        RaiseUnfiltered( liftMotorPower );
    }

    private void RaiseUnfiltered( double liftPower )
    {
        liftMotor.setPower( liftPower );
    }

    // Method to raise the lift a set amount in auton mode
    public void AutonRaise( )
    {
        // To raise, power is negative
        RaiseUnfiltered( -AUTON_LIFT_POWER );

        // Give time to raise
        JoystickUtilities.Delay_ms( AUTON_RAISE_TIME_MS );

        RaiseUnfiltered( 0.0 );
    }

    // Method to lower the lift a set amount in auton mode
    public void AutonLower( )
    {
        // To lower, power is positive
        RaiseUnfiltered( AUTON_LIFT_POWER );

        // Give time to raise
        JoystickUtilities.Delay_ms( AUTON_LOWER_TIME_MS );

        RaiseUnfiltered( 0.0 );
    }

    public int GetPosition( )
    {
        return liftMotor.getCurrentPosition();
    }
}
