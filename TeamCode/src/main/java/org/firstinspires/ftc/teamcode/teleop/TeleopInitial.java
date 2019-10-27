package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

import static android.animation.ValueAnimator.REVERSE;

@TeleOp(name="TeleopInitial_C",group="Skystone")
public class TeleopInitial extends OpMode {

    Robot robot = new Robot();

    ElapsedTime timer = new ElapsedTime();

    double speedMulitplier = 1.0;

    String rotationDirection = "up";
    double x = 13.25;
    double y = -3.5;
//    double currentPositionAngle = -3;
//    double currentPositionExtension = 13.25;
    double verticalServoAngleFactor = 1.7 ;

    @Override
    public void init() {
        robot.initRegular(hardwareMap);
    }

    @Override
    public void loop() {

        //mecanum drive
        if (gamepad1.a) speedMulitplier = 0.5;
        if (gamepad1.b) speedMulitplier = 1.0;
        double speed = (-gamepad1.left_stick_y * speedMulitplier) + (-gamepad2.left_stick_y * 0.3);
        double strafe = gamepad1.left_stick_x * speedMulitplier;
        double turn = -gamepad1.right_stick_x * speedMulitplier;


        robot.leftFrontMotor.setPower(speed + strafe + turn);
        robot.rightFrontMotor.setPower(speed - strafe - turn);
        robot.rightBackMotor.setPower(speed + strafe - turn);
        robot.leftBackMotor.setPower(speed - strafe + turn);

        if (gamepad1.right_bumper) robot.light.setPower(1.0);
        if (gamepad1.left_bumper) robot.light.setPower(0.0);

//        double positionChangeAngle = -gamepad2.right_stick_y * 0.5;
//        currentPositionAngle += positionChangeAngle;
//        if (currentPositionAngle > 70.3125) currentPositionAngle = 70.3125;
//        if (currentPositionAngle < -3) currentPositionAngle = -3;
//
////        double extensionPositionOffset = (currentPositionAngle * extensionMotorAngleFactor * 0.2);
//        double positionChangeExtension = ((gamepad2.right_trigger - gamepad2.left_trigger) * 0.2);
//        currentPositionExtension += positionChangeExtension;
//        if (currentPositionExtension > 25) currentPositionExtension = 25.0;
//        if (currentPositionExtension < 13.25) currentPositionExtension = 13.25;
//
//        robot.moveArm(currentPositionAngle, currentPositionExtension);
//        telemetry.addData("Angle", currentPositionAngle);
//        telemetry.addData("Extension", currentPositionExtension);

        double xChange = gamepad2.right_stick_x * 0.2;
        double yChange = -gamepad2.right_stick_y * 0.2;
        x += xChange;
        y += yChange;
        if (x > 27) x = 27;
        if (x < 14.5 && y < -1.0) x = 14.5;
        if (y > 23) y = 23;
        if (y < -3.5) y = -3.5;
        robot.moveArmXY(x,y);
        telemetry.addData("X: ", x);
        telemetry.addData("Y: ", y);

        double verticalAngleOffset = (((Math.toDegrees(Math.atan2(y, x)) + robot.ARM_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG)) / 360) * verticalServoAngleFactor;
        double verticalServoPosition = 0.5 + verticalAngleOffset;
        if (timer.seconds() > 0.5 && timer.seconds() < 1) {
            if (rotationDirection == "up") robot.rotationServo.setPosition(0.49);
            if (rotationDirection == "left") robot.rotationServo.setPosition(0.84);
            if (rotationDirection == "right") robot.rotationServo.setPosition(0.13);
        }
        if (timer.seconds() > 1) {
            robot.verticalServo.setPosition(verticalServoPosition);
        }

        double currentPositionRotation = robot.rotationServo.getPosition();
        int positionChangeRotation = (int) (gamepad2.left_stick_x * 0.01);
        robot.rotationServo.setPosition(currentPositionRotation + positionChangeRotation);

        if (gamepad1.x) robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_DOWN_POSITION);

        if (gamepad1.y) robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_UP_POSITION);

        if (gamepad2.a) {
            robot.grabberServo.setPosition(1.0);
            robot.grabberServoTwo.setPosition(0.20);
        }
        if (gamepad2.b) {
            robot.grabberServo.setPosition(0.5);
            robot.grabberServoTwo.setPosition(0.75);
        }
        if (gamepad2.dpad_down) {
            robot.grabberServo.setPosition(0.9);
            robot.grabberServoTwo.setPosition(0.3);
        }

//        telemetry.addData("grabberServo", robot.grabberServo.getPosition());
//        telemetry.addData("grabberServoTwo", robot.grabberServoTwo.getPosition());

        if(gamepad2.dpad_right) {
            robot.verticalServo.setPosition(0.5);
            timer.reset();
            rotationDirection = "right";
        }

        if(gamepad2.dpad_left) {
            robot.verticalServo.setPosition(0.5);
            timer.reset();
            rotationDirection = "left";
        }

        if(gamepad2.dpad_up){
            robot.verticalServo.setPosition(0.5);
            timer.reset();
            rotationDirection = "up";
        }


        telemetry.update();

    }
}
