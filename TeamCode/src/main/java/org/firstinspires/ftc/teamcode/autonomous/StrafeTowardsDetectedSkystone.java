package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.ETC2019OpModes.NavigationWebcamVuforia;
import org.firstinspires.ftc.teamcode.common.Robot;
@Autonomous(name = "StrafeSkystone", group = "auto")
public class StrafeTowardsDetectedSkystone extends LinearOpMode {

    // Global variables go before runOpMode()
    Robot robot = new Robot();

    NavigationWebcamVuforia vuforia = new NavigationWebcamVuforia();

    int state = 1;

    // Capital letters and underscores for constants
    static final int STRAFE_TO_VIEWING_POSITION = 1;
    static final int DETERMINE_SKYSTONE = 2;
    static final int STRAFE_TO_SKYSTONE = 3;
    static final int STATE_END = 5;

    double strafePower = 0.75;
    double drivePower = 0.5;
    double viewingPosition1 = 23.0; //strafe
    double viewingPosition2 = 7.25; //forward
    double viewingPosition3 = 12.5; //forward
    double grabbingPosition = 20; //strafe

    int skyStonePosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // init()
        robot.initForRunToPosition(hardwareMap);

        waitForStart(); // MUST add this yourself

        while(opModeIsActive()) {  // MUST add this yourself
            // loop()
            telemetry.addData("Current State", state);

            switch(state) {
                case STRAFE_TO_VIEWING_POSITION:
                    robot.strafe(strafePower, viewingPosition1);

                    if(robot.strafe(strafePower, viewingPosition1)); {
                        goToNextState();
                    }
                    break;

                case DETERMINE_SKYSTONE:
                    //determine if there is a skystone and return boolean vuforia.skyStoneSeen()
                    if(!vuforia.skyStoneSeen(hardwareMap)){
                        skyStonePosition += 1;
                        robot.drive(drivePower, viewingPosition2);

                        if (robot.drive(drivePower, viewingPosition2)){
                            //determine if there is a skystone and update boolean vuforia.skyStoneSeen()
                        }
                    }

                    else if(!vuforia.skyStoneSeen(hardwareMap)){
                        skyStonePosition += 1;
                        robot.drive(drivePower, viewingPosition3);

                        if (robot.drive(drivePower, viewingPosition2)){
                            //determine if there is a skystone and update boolean vuforia.skyStoneSeen()
                        }
                    }

                    if (vuforia.skyStoneSeen(hardwareMap)){
                        goToNextState();
                        break;
                    }



                case STRAFE_TO_SKYSTONE:

                    robot.strafe(strafePower,grabbingPosition);

                    break;

                //For this purpose, the default state means that we are in the final state (or the tasks are finished)
                default:
                    state = STATE_END;
                    break;
            }

        }
    }



    /*
     * For each of these methods, you can add more things you want the robot to do each time it goes to a new state
     */
    // Increment the state variable to go to the next state
    public void goToNextState(){
        state++;
    }

    // Go to a specific state
    public void goToState(int newState) {
        state = newState;
    }
}


