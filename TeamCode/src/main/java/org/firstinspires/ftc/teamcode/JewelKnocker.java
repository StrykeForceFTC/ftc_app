package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

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
    final private static int RED_LOWER_LIMIT = 50;

    // Position values for servo
    final private static double KNOCKER_DOWN_POSITION = 0.6;
    final private static double KNOCKER_UP_POSITION = 0.1;

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
        if ( colorSensor.red( ) > RED_LOWER_LIMIT )
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

    // Method to lower the jewel knocker
    public void RaiseKnocker( )
    {
        knockerServo.setPosition( KNOCKER_UP_POSITION );
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

}
