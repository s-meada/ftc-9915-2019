package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.TeleopRobot;

@TeleOp(name="TeleopInitial_C",group="Skystone")
public class TeleopInitial extends OpMode {

    TeleopRobot robot = new TeleopRobot();

    ElapsedTime timer = new ElapsedTime();

    boolean yOverride = false;
    double speedMultiplier = 1.0;

    String rotationDirection = "up";
    double x = 13.75;
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
        yChange += gamepad2.right_trigger * 0.6;

        if (gamepad1.dpad_down) yOverride = true;
        if (gamepad1.dpad_up) yOverride = false;
        x += xChange;
        y += yChange;
        if (x > 28) x = 28;
        if (x < 13.5 && y < 0) x = 13.5;
        if (y > 23) y = 23;
        if (yOverride) {
            if (y < -5.5) y = -5.5;
        }
        else {
            if (y < -3.5) y = -3.5;
        }
        robot.moveArmXY(x,y);

//Move the arm down FAST to get under the bridge if pressing right joystick:
        if (gamepad2.right_stick_button) {
            y = -2.4; x = 13.5;
//Also lower/close the claw so it doesn't get stuck on the bridge either:
            robot.capstoneServoClaw.setPosition(robot.CAPSTONE_CLAW_CLOSED);
        }

        double verticalAngleOffset = (((Math.toDegrees(Math.atan2(y, x)) + robot.ARM_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG)) / 360) * verticalServoAngleFactor;
        double backlashAdjust = robot.backlash / robot.ARM_ANGLE_MOTOR_TICKS_PER_ROTATION * verticalServoAngleFactor;
        double verticalServoPosition = 0.52 + verticalAngleOffset + backlashAdjust;
        robot.verticalServo.setPosition(verticalServoPosition);

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

        if (gamepad2.x) {
//            robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_DOWN_POSITION);
            capstoneServoDown = true;
            double capstoneDistance = robot.capstoneSensor.getDistance(DistanceUnit.INCH);
            x -= capstoneDistance - 3.0;
            robot.moveArmXY(x,y);
            timer.reset();
            while (timer.milliseconds() <= 500) continue;
            robot.capstoneServo.setPosition(robot.CAPSTONE_ANGLE_DOWN);
            telemetry.addData("Capstone Distance", capstoneDistance);
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
//moving the capstone arm angle up slightly when releasing the capstone avoids pushing the back of the capstone
            if(robot.capstoneServo.getPosition()<=robot.CAPSTONE_ANGLE_DOWN +0.01)robot.capstoneServo.setPosition(robot.CAPSTONE_ANGLE_DOWN +.025);

        }

        if(gamepad2.dpad_right) {
            robot.rotationServo.setPosition(0.11);
        }

        if(gamepad2.dpad_left) {
            robot.rotationServo.setPosition(0.84);
        }

        if(gamepad2.dpad_up){
            robot.rotationServo.setPosition(0.47);
        }


        //telemetry
        telemetry.addData("Arm X: ", x);
        telemetry.addData("Arm Y: ", y);
        telemetry.addData("Rotation Servo: ", robot.rotationServo.getPosition());
        telemetry.update();

    }
}
