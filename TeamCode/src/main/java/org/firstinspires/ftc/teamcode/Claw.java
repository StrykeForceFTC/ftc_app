package org.firstinspires.ftc.teamcode;

/**
 * Created by gstaats on 21/09/17.
 */
//Hello!
import com.qualcomm.robotcore.hardware.Servo;

//Declaration of Servos
public class claw {
private Servo leftClawMotor;
private Servo rightClawMotor;

//
//Claw inward motion
public void claw_Inward(){
        leftClawMotor.setPower(1);
        rightClawMotor.setPower(1);
        }
//Claw outward motion
public void claw_Outward(){
        leftClawMotor.setPower(-1);
        rightClawMotor.setPower(-1);

        }

        }