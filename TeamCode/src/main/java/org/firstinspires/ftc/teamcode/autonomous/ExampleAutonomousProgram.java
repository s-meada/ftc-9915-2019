package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Example Autonomous Program", group = "test")
public class ExampleAutonomousProgram extends LinearOpMode {
    // Global variables go before runOpMode()
    Robot robot = new Robot();


    int state = 1;

    // Capital letters and underscores for constants
    static final int START_DRIVING  = 1;
    static final int STOP_DRIVING   = 2;
    static final int STATE_END      = 3;

    double drivePower = 0.5;
    double driveDistance = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        // init()
        robot.initForRunToPosition(hardwareMap);

        waitForStart(); // MUST add this yourself

        while(opModeIsActive()) {  // MUST add this yourself
            // loop()
            telemetry.addData("Current State", state);

            switch(state) {
                case START_DRIVING:
                    // Code for this state

                    // If-statement to ensure the robot is done driving before we go to the next state
                    if(robot.drive(drivePower, driveDistance)) {
                        goToNextState();
                    }
                    break;

                case STOP_DRIVING:
                    // Code for this state
                    robot.stop();
                    goToNextState();
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
    public void goToNextState() {
        state++;
    }

    // Go to a specific state
    public void goToState(int newState) {
        state = newState;
    }
}
