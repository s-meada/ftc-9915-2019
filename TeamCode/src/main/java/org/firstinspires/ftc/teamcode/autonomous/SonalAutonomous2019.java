package org.firstinspires.ftc.teamcode.autonomous;

/* The position of the arm extension motor changes so that the arm is extended.
The position of the arm angle motor changes so that the arm is lowered.
The position of the claw servo changes so that the skystone is released onto the foundation.
The position of the arm extension motor changes so that the arm is retracted.
The position of the arm angle motor changes so that the arm is raised to its original position before the process of releasing the skystone.
The robot runs to a position (backwards) in which the front of the robot is behind the horizontal position of the back of the foundation.
The robot strafes to a position to the right until it is a bit to the left of the vertical centerline of the building zone, out of the vertical trajectory of the alliance partner robot.
The robot runs to a position (backwards) until the color sensor senses red.
The robot stops.*/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

import static org.firstinspires.ftc.teamcode.autonomous.AlignAndPickUpSkystone.STATE_END;
import static org.firstinspires.ftc.teamcode.common.Robot.ANGLE_MOTOR_DOWN_LIMIT;
import static org.firstinspires.ftc.teamcode.common.Robot.EXTENSION_MOTOR_EXTENDED_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.EXTENSION_MOTOR_RETRACTED_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_OPEN_POSITION;

public class SonalAutonomous2019 extends LinearOpMode {
    
    Robot robot = new Robot();
    int state;
    //variables
    double drivePower = -0.5;
    double strafePower = -0.75;
    double behindFoundationPosition = 32;
    double towardsCenterPosition = 12;
    double towardsRedLinePosition = 25;



    //cases
    static final int ROBOT_EXTENDS_ARM = 1;
    static final int ROBOT_RAISES_ARM = 2;
    static final int ROBOT_RELEASES_SKYSTONE = 3;
    static final int ROBOT_RETRACTS_ARM = 4;
    static final int ROBOT_LOWERS_ARM = 5;
    static final int ROBOT_MOVES_BEHIND_FOUNDATION = 6;
    static final int ROBOT_STRAFES_CLOSER_TO_CENTER = 7;
    static final int ROBOT_MOVES_BACKWARDS = 8;
    static final int ROBOT_STOPS = 9;
    static final int END_STATE = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        // init()
        robot.initForRunToPosition(hardwareMap);

        waitForStart(); // MUST add this yourself

        while (opModeIsActive()) {  // MUST add this yourself
            // loop()
            telemetry.addData("Current State", state);
            switch (state) {
                case ROBOT_EXTENDS_ARM:
                    robot.extensionMotor.setTargetPosition(EXTENSION_MOTOR_EXTENDED_POSITION);
                    goToNextState();
                    break;

                case ROBOT_RAISES_ARM:
                    robot.angleMotor.setTargetPosition(500);
                    goToNextState();
                    break;

                case ROBOT_RELEASES_SKYSTONE:
                    robot.grabberServo.setPosition(GRABBER_SERVO_OPEN_POSITION);
                    goToNextState();
                    break;

                case ROBOT_RETRACTS_ARM:
                    robot.extensionMotor.setTargetPosition(EXTENSION_MOTOR_RETRACTED_POSITION);
                    goToNextState();
                    break;

                case ROBOT_LOWERS_ARM:
                    robot.angleMotor.setTargetPosition(ANGLE_MOTOR_DOWN_LIMIT);
                    goToNextState();
                    break;

                case ROBOT_MOVES_BEHIND_FOUNDATION:
                    robot.drive(drivePower, behindFoundationPosition);
                    goToNextState();
                    break;


                case ROBOT_STRAFES_CLOSER_TO_CENTER:
                    robot.strafe(strafePower, towardsCenterPosition);
                    goToNextState();
                    break;


                case ROBOT_MOVES_BACKWARDS:
                    robot.drive(drivePower, towardsRedLinePosition);
                    goToNextState();
                    break;


                case ROBOT_STOPS:
                    robot.stop();
                    goToNextState();
                    break;


                default:
                    state = STATE_END;
                    break;
            }

        }
    }

    public void goToNextState() { state++; }
    public void goToState(int newState) {
        state = newState; }

}