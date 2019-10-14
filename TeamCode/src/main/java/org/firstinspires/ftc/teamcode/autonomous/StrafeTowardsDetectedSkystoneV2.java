package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.ETC2019OpModes.NavigationWebcamVuforia;
import org.firstinspires.ftc.teamcode.common.Robot;
@Autonomous(name = "StrafeSkystoneV2", group = "auto")
public class StrafeTowardsDetectedSkystoneV2 extends LinearOpMode {

    // Global variables go before runOpMode()
    Robot robot = new Robot();




    int state = 1;

    // Capital letters and underscores for constants
    static final int STRAFE_TO_VIEWING_POSITION = 1;
    static final int DETERMINE_SKYSTONE = 2;
    static final int STRAFE_TO_SKYSTONE = 3;
    static final int STATE_END = 5;

    int skyStonePosition = 1;
    double strafePower = 0.75;
    double drivePower = 0.5;
    double viewingPosition = 15.1; //strafe



    @Override
    public void runOpMode() throws InterruptedException {
        // init()
        robot.initForRunToPosition(hardwareMap);
        SkystoneVuforiaData vision = new SkystoneVuforiaData(hardwareMap,robot);
        waitForStart(); // MUST add this yourself

        vision.targetsSkyStone.activate();
        while(opModeIsActive()) {  // MUST add this yourself
            // loop()
            telemetry.addData("Current State", state);

            switch(state) {
                case STRAFE_TO_VIEWING_POSITION:
                    robot.strafe(strafePower, viewingPosition);

                    if(robot.strafe(strafePower, viewingPosition)); {
                        goToNextState();
                        break;
                    }


                case DETERMINE_SKYSTONE:
                    // run vuforia to determine vuforia.skyStoneConfiguration()
                    break;


                case STRAFE_TO_SKYSTONE:
                    telemetry.addData("Determined SkyStone Position:", skyStonePosition);
                    if (skyStonePosition == 1){

                        robot.strafe(strafePower,15);

                        if(robot.strafe(strafePower,15)) {

                            robot.drive(drivePower,-7.25);
                        }


                    }


                    if (skyStonePosition == 2){

                        robot.strafe(strafePower,15);

                        if(robot.strafe(strafePower,15)) {

                            robot.drive(drivePower, 1.0);
                        }
                    }

                    if(skyStonePosition == 3){

                        robot.strafe(strafePower,15);

                        if(robot.strafe(strafePower,15)) {

                            robot.drive(drivePower, 9.25);
                        }
                    }

                    break;

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



