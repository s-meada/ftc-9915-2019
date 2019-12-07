package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.TeleopRobot;

@TeleOp(name="TeleopInitial_C",group="Skystone")
public class TeleopInitial extends OpMode {

    TeleopRobot robot = new TeleopRobot();

    ElapsedTime timer = new ElapsedTime();

    double speedMultiplier = 1.0;

    String rotationDirection = "up";
    double x = 13.25;
    double y = -3.5;
    double verticalServoAngleFactor = 1.7;
    double capstoneServoPosition = 0.36;
    boolean capstoneServoDown = false;
    @Override
    public void init() {
        robot.initRegular(hardwareMap);
        robot.setModeChassisMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        //mecanum drive
        if (gamepad1.a){
            speedMultiplier = 0.5;
            robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (gamepad1.b){
            speedMultiplier = 1.0;
            robot.setModeChassisMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double speed = (-gamepad1.left_stick_y * speedMultiplier) + (-gamepad2.left_stick_y * 0.3);
        double strafe = gamepad1.left_stick_x * speedMultiplier;
        double turn = -gamepad1.right_stick_x * speedMultiplier;


        robot.leftFrontMotor.setPower(speed + strafe + turn);
        robot.rightFrontMotor.setPower(speed - strafe - turn);
        robot.rightBackMotor.setPower(speed + strafe - turn);
        robot.leftBackMotor.setPower(speed - strafe + turn);

        if (gamepad1.right_bumper) robot.light.setPower(1.0);
        if (gamepad1.left_bumper) robot.light.setPower(0.0);

        double xChange = gamepad2.right_stick_x * 0.2;
        double yChange = -gamepad2.right_stick_y * 0.2;
        x += xChange;
        y += yChange;
        if (x > 27) x = 27;
        if (x < 14 && y < -1.0) x = 14;
        if (y > 23) y = 23;
        if (y < -3.5) y = -3.5;
        robot.moveArmXY(x,y);
        telemetry.addData("X: ", x);
        telemetry.addData("Y: ", y);

        double verticalAngleOffset = (((Math.toDegrees(Math.atan2(y, x)) + robot.ARM_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG)) / 360) * verticalServoAngleFactor;
        double backlashAdjust = robot.backlash / robot.ARM_ANGLE_MOTOR_TICKS_PER_ROTATION * verticalServoAngleFactor;
        double verticalServoPosition = 0.52 + verticalAngleOffset + backlashAdjust;
        robot.verticalServo.setPosition(verticalServoPosition);
//        if (timer.seconds() > 0.5 && timer.seconds() < 1) {
//            if (rotationDirection == "up") robot.rotationServo.setPosition(0.49);
//            if (rotationDirection == "left") robot.rotationServo.setPosition(0.84);
//            if (rotationDirection == "right") robot.rotationServo.setPosition(0.13);
//        }

//        double currentPositionRotation = robot.rotationServo.getPosition();
//        int positionChangeRotation = (int) (gamepad2.left_stick_x * 0.01);
//        robot.rotationServo.setPosition(currentPositionRotation + positionChangeRotation);

        if (gamepad1.x) robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_DOWN_POSITION);

        if (gamepad1.y) robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_UP_POSITION);


        //close grabber
        if (gamepad2.a) {
            robot.grabberServo.setPosition(1.0);
            robot.grabberServoTwo.setPosition(0.20);
        }
        //open grabber
        if (gamepad2.b) {
            robot.grabberServo.setPosition(0.5);
            robot.grabberServoTwo.setPosition(0.75);
        }
        //open for capstone
        if (gamepad2.dpad_down) {
            robot.grabberServo.setPosition(0.55);
            robot.grabberServoTwo.setPosition(0.75);
        }
        if (gamepad2.x) {
            capstoneServoDown = true;
            robot.capstoneServo.setPosition(robot.CAPSTONE_ANGLE_DOWN);
            robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_DOWN_POSITION);
        }

        if (gamepad2.y) {
            capstoneServoDown = false;
            robot.capstoneServo.setPosition(robot.CAPSTONE_ANGLE_UP);
        }

        if (gamepad2.left_bumper) {
            robot.capstoneServoClaw.setPosition(robot.CAPSTONE_CLAW_CLOSED);
        }

        if (gamepad2.right_bumper) {
            robot.capstoneServoClaw.setPosition(robot.CAPSTONE_CLAW_OPEN);
        }

        if(gamepad2.dpad_right) {
            robot.rotationServo.setPosition(0.13);
//            robot.verticalServo.setPosition(0.5);
//            timer.reset();
//            rotationDirection = "right";
        }

        if(gamepad2.dpad_left) {
            robot.rotationServo.setPosition(0.84);
//            robot.verticalServo.setPosition(0.5);
//            timer.reset();
//            rotationDirection = "left";
        }

        if(gamepad2.dpad_up){
            robot.rotationServo.setPosition(0.49);
//            robot.verticalServo.setPosition(0.5);
//            timer.reset();
//            rotationDirection = "up";
        }


        telemetry.update();

    }
}
