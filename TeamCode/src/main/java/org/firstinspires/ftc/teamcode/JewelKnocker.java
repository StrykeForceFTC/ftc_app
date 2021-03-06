package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by jscott on 11/04/17.
 */

public class JewelKnocker
{
    // Enumeration for ball color
    public enum COLORS { RED, BLUE }

    private Servo knockerServo = null;
    private ColorSensor colorSensor = null;

    // Constants for detecting color
    final private static int RED_LOWER_LIMIT = 55;

    // Position values for servo
    final private static double KNOCKER_DOWN_POSITION = 0.65;
    final private static double KNOCKER_UP_POSITION = 0.3;
    final private static double KNOCKER_DOWN_POSITION_8553 = 0.625;
    final private static double KNOCKER_UP_POSITION_8553 = 0.1;

    public JewelKnocker( Servo servo, ColorSensor color )
    {
        // Store hardware references
        knockerServo = servo;
        colorSensor  = color;

        // Turn on color sensor LED light to help make it easier to see jewels
        colorSensor.enableLed( true );

        // No setup needed for servo
    }

    // Method used to determine whether color of jewel is blue or red
    public COLORS GetColor( )
    {
        int timesItWasRed = 0;
        int timesItWasBlue = 0;
        int tries = 0;

        while (tries < 7 )
        {
            if ( colorSensor.red( ) > colorSensor.blue() )
            {
                timesItWasRed++;
                tries++;
            }

            else  if ( colorSensor.red() < colorSensor.blue() ){

                timesItWasBlue++;
                tries++;

            }
            else{

            }

            // Put short delay between checks
            Delay_ms( 20.0 );

        }

        if ( timesItWasRed >= 4 )
        {
            return COLORS.RED;
        }
        else
        {
            return COLORS.BLUE;
        }
 
    }

    // Method to lower the jewel knocker
    public void LowerKnocker( )
    {
        knockerServo.setPosition( KNOCKER_DOWN_POSITION );
    }

    public void LowerKnocker8553( )
    {
        knockerServo.setPosition( KNOCKER_DOWN_POSITION_8553 );
    }

    // Method to lower the jewel knocker
    public void RaiseKnocker( )
    {
        knockerServo.setPosition( KNOCKER_UP_POSITION );
    }

    public void RaiseKnocker8553( )
    {
        knockerServo.setPosition( KNOCKER_UP_POSITION_8553 );
    }

     // Method used to read red value to help with debug
    public int RedValue( )
    {
        return colorSensor.red( );
    }

    // Method used to read blue value to help with debug
    public int BlueValue( )
    {
        return colorSensor.blue( );
    }

    // Method used to read knocker servo position for debug
    public double KnockerPositionGet( )
    {
        return knockerServo.getPosition( );
    }

    // Method to set knocker position for debug
    public void KnockerPositionSet( double position )
    {
        if ( ( position >= 0.0 ) && ( position <= 1.0 ) )
        {
            knockerServo.setPosition( position );
        }
    }

    private void Delay_ms( double delay )
    {
        ElapsedTime delayTimer = new ElapsedTime( ElapsedTime.Resolution.MILLISECONDS );
        delayTimer.reset();
        int timeWaster = 0;
        while ( delayTimer.time() < delay )
        {
            timeWaster++;
        }
    }

}
