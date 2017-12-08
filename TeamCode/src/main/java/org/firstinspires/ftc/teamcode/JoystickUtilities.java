package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Utility methods for shaping and filtering joystick inputs.
 */

public final class JoystickUtilities
{
    // Shape joystick input using the cube of the joystick input value times a weighting factor
    // plus the input times one minus the weighting factor. Basic equation is:
    // Y = ( ( X^3 )* W ) + ( X * ( 1 - W ) )
    // Where Y = shaped joystick output, X = joystick input and W = weighting
    // W must be in the range of 0 to 1, and if it isn't, it will be forced to 0.5.
    public static double ShapeCubePlusInputWeighted( double joystick, double weighting )
    {
        // Protect for weighting not being in allowed range of 0-1
        if ( ( weighting > 1.0 ) || ( weighting < 0.0 ) )
        {
            // Use default value of 0.5, which just gives X^3*0.5 + 0.5*X
            weighting = 0.5;
        }

        double joystickCubed = Math.pow( joystick, 3.0 );
        double shapedJoystick = ( joystickCubed * weighting ) + ( ( 1.0 - weighting ) * joystick );

        return shapedJoystick;
    }

    // Shape joystick input using the square root of the joystick input value times a weighting factor
    // plus the input times one minus the weighting factor. Basic equation is:
    // Y = ( SQRT( X ) * W ) + ( X * ( 1 - W ) )
    // Where Y = shaped joystick output, X = joystick input and W = weighting
    // W must be in the range of 0 to 1, and if it isn't, it will be forced to 0.5.
    public static double ShapeSqrtPlusInputWeighted( double joystick, double weighting )
    {
        // Protect for weighting not being in allowed range of 0-1
        if ( ( weighting > 1.0 ) || ( weighting < 0.0 ) )
        {
            // Use default value of 0.5, which just gives X^3*0.5 + 0.5*X
            weighting = 0.5;
        }

        double joystickSqrt = Math.sqrt( Math.abs( joystick ) );

        if ( joystick < 0.0 )
        {
            joystickSqrt = -joystickSqrt;
        }

        double shapedJoystick = ( joystickSqrt * weighting ) + ( ( 1.0 - weighting ) * joystick );

        return shapedJoystick;
    }

    // This method creates a low pass filtered that has different filter constants for increasing vs
    // decreasing magnitude inputs. The basic equation is:
    // Y[n] = X[n] * Fk + Y[n-1] * ( 1 - Fk )
    // Where Fk is the filter constant, must be from 0-1, and is selected based on whether the ABS of
    // the input is >= or < the filteredValue. Y is the output and X is the input.
    public static double LowPassFilter( double filteredLast, double input, double increasingFk, double decreasingFk )
    {
        double filterK = 1.0;        // Default value set to give no filtering if input value is out of bounds

        if ( Math.abs( input ) >= Math.abs( filteredLast ) )
        {
            // When magnitude is increasing, using increasingFk, if valid
            if ( ( increasingFk < 1.0 ) && ( increasingFk >= 0.0 ) )
            {
                filterK = increasingFk;
            }
            // else use default of 1.0
        }
        else
        {
            // Magnitude is decreasing so use decreasingFk
            if ( ( decreasingFk < 1.0 ) && ( decreasingFk >= 0.0 ) )
            {
                filterK = decreasingFk;
            }
            // else use default of 1.0
        }

        // Run filter
        double filtered = ( input * filterK ) + ( filteredLast * ( 1.0 - filterK ) );

        return filtered;
    }

    // Delay for a certain amount of time in seconds
    public static void Delay_s( double seconds )
    {
        try {
            Thread.sleep( (long) ( seconds * 1000.0 ) );
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // Delay for a certain amount of time in milliseconds
    public static void Delay_ms( double milliseconds )
    {
        try {
            Thread.sleep( (long) ( milliseconds ) );
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}
