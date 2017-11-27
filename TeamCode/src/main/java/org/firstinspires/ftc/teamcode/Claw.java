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
        //Claw inward motion
        public void claw_Inward(){


                leftClawMotor.setPosition( 1 );
                rightClawMotor.setPosition( 1 );
        }

        //Claw outward motion
        public void claw_Outward(){


                leftClawMotor.setPosition( -1 );
                rightClawMotor.setPosition( -1 );
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

        public double GetLeftPosition( ) {
            return leftClawMotor.getPosition();
        }

        public double GetRightPosition( )
        {
            return rightClawMotor.getPosition();
        }

}
