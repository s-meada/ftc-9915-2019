package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name="OnlyParking", group="Autonomous")
public class PathTwoOnlyParkingAutonomous extends LinearOpMode {
    Robot robot = new Robot();

    int state = 1;

    //cases
    static final int DRIVES_FORWARD = 1;
    static final int PARKS = 2;
    static final int END_STATE = 3;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.initForRunToPosition(hardwareMap);
        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("currentState", state);
            telemetry.update();
            switch (state) {

                case DRIVES_FORWARD:
                    if (robot.drive(1, -17)) {
                        goToNextState();
                        }
                    break;

                case PARKS:
                    robot.stop();
                    goToNextState();
                    break;

                default:
                    state = END_STATE;
                    break;
            }


        }
    }

    public void goToNextState() {state++;}
    public void goToState (int newState) {
        state = newState;
    }

}
