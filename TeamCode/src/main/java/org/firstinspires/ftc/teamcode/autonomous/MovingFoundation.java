package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.Robot;



@Autonomous(name = "Moving Foundation", group = "test")
public class MovingFoundation extends LinearOpMode {

    Robot robot = new Robot();
    ElapsedTime timer = new ElapsedTime();
    BNO055IMU gyroSensor;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation angles;

    int state = 1;

    static final int DRIVE_AWAY_FROM_BLOCK = 1;
    static final int DRIVE_TO_WALL_1 = 2;
    static final int DRIVE_TO_WALL_2 = 3;
    static final int DRIVE_TO_WALL_3 = 4;
    static final int DRIVE_TO_FOUNDATION = 5;
    static final int DRIVE_TO_FOUNDATION_2 = 6;
    static final int DRAG_FOUNDATION = 7;
    static final int STATE_END = 8;

    @Override
    public void runOpMode() throws InterruptedException {
        gyroSensor = hardwareMap.get(BNO055IMU.class, "gyroSensor");
        gyroSensor.initialize(parameters);
        robot.initForRunToPosition(hardwareMap);
        boolean isBlue = true;
        boolean timerReset = false;
        waitForStart();

        while(opModeIsActive()) {

            telemetry.addData("Blue Distance: ", robot.blueDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Red Distance: ", robot.redDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("State: ", state);
            telemetry.update();

            switch(state) {
                case DRIVE_AWAY_FROM_BLOCK:
                    /*robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.drivePower(0.5, -0.5, -0.5, 0.5);
                    if (isBlue) {
                        if (robot.blueDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
                            robot.stop();
                            goToNextState();
                        }
                    } else {
                        if (robot.redDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
                            robot.stop();
                            goToNextState();
                        }
                    }
                     */

                    if (robot.strafe(0.75,8)) {
                        robot.stop();
                        goToNextState();
                    }
                    break;

                case DRIVE_TO_WALL_1:

                    //robot.setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
                    if (isBlue) {
                        if (robot.drive(0.5, 56)) {
                            robot.stop();
                            goToNextState();
                        }
                    } else {
                        if (robot.drive(0.5, -56)) {
                            robot.stop();
                            goToNextState();
                        }
                    }
                    break;

                case DRIVE_TO_WALL_2:

                    robot.angleMotor.setTargetPosition((int)(robot.ARM_ANGLE_MOTOR_TICKS_PER_ROTATION * (25 + robot.ARM_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG) / 360));
                    robot.angleMotor.setPower(1.0);
                    robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (isBlue) {
                        robot.drivePower(0.5,0.5,0.5,0.5);
                        if (robot.blueDistanceSensor.getDistance(DistanceUnit.INCH) < 10) {
                            robot.stop();
                            goToNextState();
                        }
                    } else {
                        robot.drivePower(-0.5,-0.5,-0.5,-0.5);
                        if (robot.redDistanceSensor.getDistance(DistanceUnit.INCH) < 10) {
                            robot.stop();
                            goToNextState();
                        }
                    }
                    break;

                case DRIVE_TO_WALL_3:
                    robot.setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
                    if (isBlue) {
                        if (robot.drive(0.5,20.0)) {
                            robot.stop();
                            goToNextState();
                        }
                    } else {
                        if (robot.drive(0.5, -20.0)) {
                            robot.stop();
                            goToNextState();
                        }
                    }
                    break;

                case DRIVE_TO_FOUNDATION:
                    double distance;
                    if (isBlue) {
                        distance = robot.blueDistanceSensor.getDistance(DistanceUnit.INCH) + 6.0;
                    } else {
                        distance = robot.redDistanceSensor.getDistance(DistanceUnit.INCH) + 6.0;
                    }
                    if (robot.strafe(0.75,distance)) {
                        robot.stop();
                        goToNextState();
                    }
                    break;

                case DRIVE_TO_FOUNDATION_2:
                    if (robot.strafe(0.25, 3)) {
                        robot.stop();
                        goToNextState();
                    }

                case DRAG_FOUNDATION:
                    angles = gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    double angle = angles.firstAngle;
                    if (!timerReset) {
                        timer.reset();
                        timerReset = true;
                    }
                    robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_DOWN_POSITION);
                    if (timer.seconds() >= 1.5) {
                        if (robot.strafe(0.75, -64)) {
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