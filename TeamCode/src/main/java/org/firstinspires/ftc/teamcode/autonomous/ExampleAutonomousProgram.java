package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Example Autonomous Program", group = "test")
public class ExampleAutonomousProgram extends LinearOpMode {
    // Global variables go before runOpMode()

    int state = 1;

    // Capital letters and underscores for constants
    static final int START_DRIVING  = 1;
    static final int STOP_DRIVING   = 2;
    static final int STATE_END      = 3;

    @Override
    public void runOpMode() throws InterruptedException {
        // init()

        waitForStart(); // MUST add this yourself

        while(opModeIsActive()) {  // MUST add this yourself
            // loop()
            switch(state) {
                case START_DRIVING:
                    // Code for state 1


                    goToNextState();
                    break;

                case STOP_DRIVING:
                    // Code for state 2


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
