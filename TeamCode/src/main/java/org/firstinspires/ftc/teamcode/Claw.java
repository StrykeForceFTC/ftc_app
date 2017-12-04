package org.firstinspires.ftc.teamcode;

/**
 * Created by gstaats on 21/09/17.
 */
//Hello!
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Declaration of Servos
public class Claw {

        private Servo leftClawMotor = null;
        private Servo rightClawMotor = null;

        // Values for servos when claw is closed, 7228 robot
        private static final double LEFT_CLAW_IN_POSITION_7228 = 0.9;
        private static final double RIGHT_CLAW_IN_POSITION_7228 = 0.85;

        // Values for servos when claw is open, 7228 robot
        private static final double LEFT_CLAW_OUT_POSITION_7228 = 0.55;
        private static final double RIGHT_CLAW_OUT_POSITION_7228 = 0.55;

        // Values for servos when claw is closed, 8553 robot
        private static final double LEFT_CLAW_IN_POSITION_8553 = 0.7;
        private static final double RIGHT_CLAW_IN_POSITION_8553 = 0.75;

        // Values for servos when claw is open, 8553 robot
        private static final double LEFT_CLAW_OUT_POSITION_8553 = 0.45;
        private static final double RIGHT_CLAW_OUT_POSITION_8553 = 0.45;
        public Claw( HardwareMap ahwMap )
        {
                //Hardware Map
                rightClawMotor = ahwMap.servo.get( "rightClawMotor" );
                leftClawMotor = ahwMap.servo.get( "leftClawMotor" );

                //Setting direction of Servos
                rightClawMotor.setDirection( Servo.Direction.FORWARD );
                leftClawMotor.setDirection( Servo.Direction.REVERSE );
        }

        //
        // Claw inward motion for 7228 robot
        public void claw_Inward()
        {
                leftClawMotor.setPosition( LEFT_CLAW_IN_POSITION_7228 );
                rightClawMotor.setPosition( RIGHT_CLAW_IN_POSITION_7228 );
        }

        // Claw inward motion for 8553 robot
        public void claw_Inward8553()
        {
                leftClawMotor.setPosition( LEFT_CLAW_IN_POSITION_8553 );
                rightClawMotor.setPosition( RIGHT_CLAW_IN_POSITION_8553 );
        }

        //Claw outward motion
        public void claw_Outward()
        {
                leftClawMotor.setPosition( LEFT_CLAW_OUT_POSITION_7228 );
                rightClawMotor.setPosition( RIGHT_CLAW_OUT_POSITION_7228 );
        }

        //Claw outward motion for 8553 robot
        public void claw_Outward8553()
        {
                leftClawMotor.setPosition( LEFT_CLAW_OUT_POSITION_8553 );
                rightClawMotor.setPosition( RIGHT_CLAW_OUT_POSITION_8553 );
        }

        public void StepClosed( )
        {
                leftClawMotor.setPosition( leftClawMotor.getPosition() + 0.05 );
                rightClawMotor.setPosition( rightClawMotor.getPosition() + 0.05 );
        }

        public void StepOpen( )
        {
                leftClawMotor.setPosition( leftClawMotor.getPosition() - 0.05 );
                rightClawMotor.setPosition( rightClawMotor.getPosition() - 0.05 );
        }

        public void StepLeftUp( )
        {
                leftClawMotor.setPosition( leftClawMotor.getPosition() + 0.05 );
        }

        public void StepRightUp( )
        {
                rightClawMotor.setPosition( rightClawMotor.getPosition() + 0.05 );
        }

        public void StepLeftDown( )
        {
                leftClawMotor.setPosition( leftClawMotor.getPosition() - 0.05 );
        }

        public void StepRightDown( )
        {
                rightClawMotor.setPosition( rightClawMotor.getPosition() - 0.05 );
        }

        public double GetLeftPosition( ) {
            return leftClawMotor.getPosition();
        }

        public double GetRightPosition( )
        {
            return rightClawMotor.getPosition();
        }

}
