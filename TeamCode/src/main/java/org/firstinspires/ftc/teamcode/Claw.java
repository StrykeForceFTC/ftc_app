package org.firstinspires.ftc.teamcode;

/**
 * Created by gstaats on 21/09/17.
 */
//Hello!
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
                leftClawMotor.setDirection( Servo.Direction.FORWARD );

                //Setting power to 0
                rightClawMotor.setPower( 0 );
                leftClawMotor.setPower( 0 );
        }

        //
//Claw inward motion
        public void claw_Inward(){
                leftClawMotor.setPower(0.1);
                rightClawMotor.setPower(-0.1);
        }
        //Claw outward motion
        public void claw_Outward(){
                leftClawMotor.setPower(-0.1);
                rightClawMotor.setPower(0.1);

        }

}