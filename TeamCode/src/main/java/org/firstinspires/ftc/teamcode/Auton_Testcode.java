package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 *
 * Auton Program mainly here to test Auton Drive and other Functions in
 *
 */
@Autonomous(name = "Auton_Testcode", group = "Linear Opmode")
//@Disabled
public class Auton_Testcode extends AutonLinearBase
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // Hardware objects are created in AutonLinearBase

    // Enumeration for auton steps
    private enum AUTON_STEPS {RELEASE_LANDER, FIND_GOLD, MOVE_TO_MINERAL, LOAD_GOLD, DRIVE_DEPOT,
        UNLOAD, PARK, STOP }

    private enum GOLD_POSITIONS { LEFT_POS, MID_POS, RIGHT_POS, TERMINATE}
    private AUTON_STEPS step = AUTON_STEPS.MOVE_TO_MINERAL;
    private GOLD_POSITIONS gold=GOLD_POSITIONS.TERMINATE;
    /*
     * There is only runOpMode for linear op modes
     */
    @Override
    public void runOpMode()
    {
        /* Initialize the hardware variables.
         */
        InitHardware();

        // Wait hit till start button pressed
        waitForStart();
        runtime.reset();

        // Loop until stop or forced to end
        while (opModeIsActive())
        {

            telemetry.addData("IsAligned", GoldAligned()); // Is the bot aligned with the gold mineral
            telemetry.addData("X Pos", GoldXPosition());    // Gold X pos.

            telemetry.addLine("Encoders ")
                    .addData("FL ", go.GetEncoderFrontLeft())
                    .addData("FR ", go.GetEncoderFrontRight())
                    .addData("RL ", go.GetEncoderRearLeft())
                    .addData("RR ", go.GetEncoderRearRight());
            telemetry.addLine("TeamId")
                    .addData("Team", TeamId.name());

            telemetry.update();

            //Auton steps
            switch(step){
                case RELEASE_LANDER:
                    step= AUTON_STEPS.FIND_GOLD;
                    break;
                case FIND_GOLD:
                    go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, 135);
                    if(GoldAligned()){
                        gold=GOLD_POSITIONS.LEFT_POS;
                        go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE,45);

                    }
                    else {
                        go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, 45);
                        if(GoldAligned()){
                            gold=GOLD_POSITIONS.MID_POS;

                        }
                        else {
                            go.AutonMoveRotate(Drive.ROTATION.CLOCKWISE, 45);
                            if(GoldAligned()){
                                gold=GOLD_POSITIONS.RIGHT_POS;
                                go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, 45);
                            }
                            else{
                                gold=GOLD_POSITIONS.TERMINATE;
                                go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, 45);

                            }
                        }

                    }
                    telemetry.addData("GP", gold);

                    step=AUTON_STEPS.MOVE_TO_MINERAL;
                    break;
                case MOVE_TO_MINERAL:
                    go.AutonMove(Drive.DIRECTION.FORWARD, 8);
                    switch(gold){
                        case LEFT_POS:
                            go.AutonMove(Drive.DIRECTION.LEFT, 8);
                            break;
                        case MID_POS:
                            break;
                        case RIGHT_POS:
                            go.AutonMove(Drive.DIRECTION.RIGHT, 8);
                            break;
                        case TERMINATE:
                            break;
                    }
                    step=AUTON_STEPS.LOAD_GOLD;
                    break;
                case LOAD_GOLD:
                    if(gold!=GOLD_POSITIONS.TERMINATE){
                        go.AutonMove(Drive.DIRECTION.FORWARD, 4);
                        //put down arm and turn on loader
                        go.AutonMove(Drive.DIRECTION.REVERSE, 4);

                        switch (gold){
                            case LEFT_POS:
                                go.AutonMove(Drive.DIRECTION.RIGHT, 8);
                                break;
                            case MID_POS:
                                break;
                            case RIGHT_POS:
                                go.AutonMove(Drive.DIRECTION.LEFT, 8);
                                break;
                        }
                    }
                    step=AUTON_STEPS.DRIVE_DEPOT;
                    break;

                case DRIVE_DEPOT:
                    go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE, 75);
                   go.AutonMove(Drive.DIRECTION.FORWARD,37);
                    go.AutonMoveRotate(Drive.ROTATION.COUNTERCLOCKWISE,27.5);
                    go.AutonMove(Drive.DIRECTION.RIGHT,11);
                    go.AutonMove(Drive.DIRECTION.FORWARD, 32.5);
                    step=AUTON_STEPS.UNLOAD;
                    break;
                case UNLOAD:
                    // attach unloading thingy
                    step=AUTON_STEPS.PARK;
                    break;
                case PARK:
                    go.AutonMove(Drive.DIRECTION.REVERSE, 54);
                    step=AUTON_STEPS.STOP;
                    break;
                case STOP:
                    break;


            }


        }

    }
}


