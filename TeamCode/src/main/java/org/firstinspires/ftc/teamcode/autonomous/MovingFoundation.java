package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.common.Robot;



@Autonomous(name = "Moving Foundation", group = "test")
public class MovingFoundation extends LinearOpMode {

    Robot robot = new Robot();
    ElapsedTime timer = new ElapsedTime();

    int state = 1;
    boolean timeReset = false;
    
    static final int DRIVE_AWAY_FROM_BLOCK = 1;
    static final int DRIVE_TO_WALL_1 = 2;
    static final int DRIVE_TO_WALL_2 = 3;
    static final int DRIVE_TO_WALL_3 = 4;
    static final int DRIVE_TO_FOUNDATION = 5;
    static final int DRAG_FOUNDATION = 6;
    static final int STATE_END = 7;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initForRunToPosition(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Current State: ", state);
            telemetry.addData("Distance in inches: ", robot.sideDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();

            switch(state) {
                case DRIVE_AWAY_FROM_BLOCK:
                    // strafes to avoid hitting bridge and other blocks
                    if (robot.strafe(0.75, -2)) {
                        robot.stop();
                        goToNextState();
                    }
                    break;

                case DRIVE_TO_WALL_1:
                    // drives forward until it is definitely past bridge
                    if (robot.drive(0.5,55)) {
                        robot.stop();
                        goToNextState();
                    }
                    break;

                case DRIVE_TO_WALL_2:
                    // drives until it senses an object near the robot (the foundation)
                    robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.drivePower(0.5,0.5,0.5,0.5);
                    if (robot.sideDistanceSensor.getDistance(DistanceUnit.INCH) < 10) {
                        robot.stop();
                        goToNextState();
                    }
                    break;

                case DRIVE_TO_WALL_3:
                    // drives a little more to be at center of foundation
                    robot.setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
                    if (robot.drive(0.5,21.0)) {
                        robot.stop();
                        goToNextState();
                    }
                    break;

                case DRIVE_TO_FOUNDATION:
                    // drives a little over the measured distance to foundation to make sure
                    // we are close enough to foundation
                    double distance = robot.sideDistanceSensor.getDistance(DistanceUnit.INCH) + 4;

                    if (robot.strafe(0.25,distance)) {
                        robot.stop();
                        goToNextState();
                    }
                    break;

                case DRAG_FOUNDATION:
                    // starts timer, makes sure it doesn't keep resetting timer
                    if (!timeReset) {
                        timer.reset();
                        timeReset = true;
                    }
                    // puts foundation servos in the down position
                    robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_DOWN_POSITION);
                    if (timer.seconds() > 1.0) {
                        // pulls back foundation once servos are down
                        if (robot.strafe(0.75, -60)) {
                            robot.stop();
                            robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_UP_POSITION);
                            goToNextState();
                        }
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
