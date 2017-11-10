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
        private double leftClawPosition = 0.0;
        private double rightClawPosition = 0.0;

        public Claw( HardwareMap ahwMap )
        {
                //Hardware Map
                rightClawMotor = ahwMap.servo.get( "rightClawMotor" );
                leftClawMotor = ahwMap.servo.get( "leftClawMotor" );

                //Setting direction of Servos
                rightClawMotor.setDirection( Servo.Direction.FORWARD );
                leftClawMotor.setDirection( Servo.Direction.REVERSE );

                // Get initial position
                leftClawPosition = leftClawMotor.getPosition();
                rightClawPosition = rightClawMotor.getPosition();
        }

        //
        //Claw inward motion
        public void claw_Inward(){


                leftClawMotor.setPosition( 2 );
                rightClawMotor.setPosition( 2 );
        }

        //Claw outward motion
        public void claw_Outward(){


                leftClawMotor.setPosition( 1.5 );
                rightClawMotor.setPosition( 1.5 );
        }

        public double GetLeftPosition( ) {
                return leftClawPosition;
        }

        public double GetRightPosition( )
        {
                return rightClawPosition;
        }

}
