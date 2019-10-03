package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Robot;

public class MovingFoundation extends LinearOpMode {

    Robot robot = new Robot();

    int state = 1;

    static final int DISTANCE_FROM_FRONT_WALL = 19; // inches
    static final int DISTANCE_FROM_FOUNDATION = 25; // millimeters
    static final int DISTANCE_FROM_SIDE_WALL = 2;  // inches

    static final int DRIVE_TO_WALL = 1;
    static final int DRIVE_TO_FOUNDATION = 2;
    static final int DRAG_FOUNDATION = 3;
    static final int STATE_END = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initForRunToPosition(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Current State: ", state);

            switch(state) {
                case DRIVE_TO_WALL:

                    robot.strafe(1.0, 4);
                    robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.drivePower(1.0,1.0,1.0,1.0);
                    if(robot.frontDistanceSensor.getDistance(DistanceUnit.INCH) < DISTANCE_FROM_FRONT_WALL){
                        robot.stop();
                        goToNextState();
                    }
                    break;

                case DRIVE_TO_FOUNDATION:

                    robot.drivePower(0.5,-0.5,-0.5,0.5);
                    if(robot.foundationDistanceSensor.getDistance(DistanceUnit.MM) < DISTANCE_FROM_FOUNDATION){
                        robot.stop();
                        goToNextState();
                    }
                    break;

                case DRAG_FOUNDATION:

                    robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_DOWN_POSITION);
                    robot.drivePower(-1,1,1,-1);
                    if(robot.sideDistanceSensor.getDistance(DistanceUnit.INCH) < DISTANCE_FROM_SIDE_WALL){
                        robot.stop();
                        goToNextState();
                    }
                    break;

                default:
                    state = STATE_END;
                    break;

            }

        }

    }

    public void goToNextState() { state++; }

    public void goToState(int newState) { state = newState; }

}
