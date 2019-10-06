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

public class SonalAutonomous2019 extends LinearOpMode {
    
    Robot robot = new Robot();
    int state;
    //position variables
    double extendedArmPosition = 1;
    double loweredArmPosition = 2000;
    double clawReleasingPosition = 1.0;
    double retractedArmPosition = 2000;
    double raisedArmPosition = 0;

    //cases
    static final int ROBOT_EXTENDS_ARM = 1;
    static final int ROBOT_LOWERS_ARM = 2;
    static final int ROBOT_RELEASES_SKYSTONE = 3;
    static final int ROBOT_RETRACTS_ARM = 4;
    static final int ROBOT_RAISES_ARM = 5;
    static final int ROBOT_MOVES_BEHIND_FOUNDATION = 6;
    static final int ROBOT_STRAFES_CLOSER_TO_CENTER = 7;
    static final int ROBOT_MOVES_BACKWARDS = 8;
    static final int ROBOT_STOPS = 9;

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


            }
        }
    }
}