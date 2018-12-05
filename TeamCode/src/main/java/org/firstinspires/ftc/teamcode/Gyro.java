package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 *
 * Class for encapsulating the gyro sensor built into the Rev Hub
 *
 */
public class Gyro
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Hardware objects are created in AutonLinearBase
    // Auton steps are enumerated in AutonLinerBase

    // IMU members
    private BNO055IMU    imu;
    private Orientation  lastAngles = new Orientation();

    // Constructor
    public Gyro( HardwareMap hardwareMap )
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode           = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit      = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit      = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get( BNO055IMU.class, "imu" );
        imu.initialize( parameters );

        runtime.reset();
        while ( ( runtime.milliseconds() < 1000 ) && !imu.isGyroCalibrated() )
        {
            Delay_ms(50 );
        }
    }

    public double GetZAngle( )
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return lastAngles.firstAngle;
    }

    /*
     *  Method to delay a specified amount of time
     */
    private void Delay_ms( double milliseconds )
    {
        try {
            Thread.sleep( (long) ( milliseconds ) );
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }


}


