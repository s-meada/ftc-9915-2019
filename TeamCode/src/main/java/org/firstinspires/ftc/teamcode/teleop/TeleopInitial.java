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
    public static final double ANGLE_MOTOR_COUNTS_PER_REV = 7168.0;
    public static final double EXTENSION_MOTOR_COUNTS_PER_REV = 537.6;

    double speedMulitplier = 1.0;

    //declare motors
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor leftBack;
    DcMotor angleMotor;
    DcMotor extensionMotor;

    //declare servos
    Servo rotationServo;
    Servo grabberServo;
    Servo grabberServoTwo;
    Servo verticalServo;
    Servo foundationServo;

    String rotationDirection = "up";
    double currentPositionAngle = 0;
    double currentPositionExtension = 0;
    double verticalServoAngleFactor = 1.7 / ANGLE_MOTOR_COUNTS_PER_REV;
    double extensionMotorAngleFactor = EXTENSION_MOTOR_COUNTS_PER_REV / ANGLE_MOTOR_COUNTS_PER_REV;
    //boolean stoneTucked = false;

    @Override
    public void init() {

        robot.initRegular(hardwareMap);
        //init driving motors

//        rightFront = hardwareMap.dcMotor.get("RightFront");
//        leftFront = hardwareMap.dcMotor.get("LeftFront");
//        rightBack = hardwareMap.dcMotor.get("RightBack");
//        leftBack = hardwareMap.dcMotor.get("LeftBack");
//
//
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        //init motors
//
//        angleMotor = hardwareMap.dcMotor.get("angleMotor");
//        angleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        angleMotor.setTargetPosition(0);
//        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        angleMotor.setPower(1);
//
//
//        extensionMotor = hardwareMap.dcMotor.get("extensionMotor");
//        extensionMotor.setTargetPosition(0);
//        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        extensionMotor.setPower(1);
//
//
//
//        //init servos
//        rotationServo = hardwareMap.servo.get("rotationServo");
//        grabberServo = hardwareMap.servo.get("grabberServo");
//        grabberServoTwo = hardwareMap.servo.get("grabberServoTwo");
//        verticalServo = hardwareMap.servo.get("verticalServo");
//        foundationServo = hardwareMap.servo.get("foundationServo");
//
//
//        rotationServo.setPosition(0.47);
//        grabberServo.setPosition(0.5);
//        grabberServoTwo.setPosition(0.75);
//        verticalServo.setPosition(0.5);
//        foundationServo.setPosition(0.5);

        //init driving motor

    }

    @Override
    public void loop() {

        //mecanum drive
        if (gamepad1.a) speedMulitplier = 0.5;
        if (gamepad1.b) speedMulitplier = 1.0;
        double speed = -gamepad1.left_stick_y * speedMulitplier;
        double strafe = gamepad1.left_stick_x * speedMulitplier;
        double turn = gamepad1.right_stick_x * speedMulitplier;


        robot.leftFrontMotor.setPower(speed + strafe + turn);
        robot.rightFrontMotor.setPower(speed - strafe - turn);
        robot.rightBackMotor.setPower(speed + strafe - turn);
        robot.leftBackMotor.setPower(speed - strafe + turn);

        double positionChangeAngle = -gamepad2.right_stick_y * 0.3;
        currentPositionAngle += positionChangeAngle;

        double extensionPositionOffset = currentPositionAngle * extensionMotorAngleFactor * 0.5;
        double positionChangeExtension = (int) ((gamepad2.right_trigger - gamepad2.left_trigger) * 0.5);
        currentPositionExtension += positionChangeExtension + extensionPositionOffset;

        robot.moveArm(currentPositionAngle, currentPositionExtension);
//        if(currentPositionAngle   > 1400) currentPositionAngle = 1400;
//        if(currentPositionAngle < 0) currentPositionAngle = 0;
        telemetry.addData("Angle", currentPositionAngle);
        telemetry.addData("Extension", currentPositionExtension);

//        angleMotor.setTargetPosition(currentPositionAngle);

        double verticalAngleOffset = currentPositionAngle * verticalServoAngleFactor;
        double verticalServoPosition = 0.4 + verticalAngleOffset;
        if (timer.seconds() > 0.5 && timer.seconds() < 1) {
            if (rotationDirection == "up") robot.rotationServo.setPosition(0.47);
            if (rotationDirection == "right") robot.rotationServo.setPosition(0.82);
            if (rotationDirection == "left") robot.rotationServo.setPosition(0.11);
        }
        if (timer.seconds() > 1) {
            robot.verticalServo.setPosition(verticalServoPosition);
        }

        double currentPositionRotation = robot.rotationServo.getPosition();
        int positionChangeRotation = (int) (gamepad2.left_stick_x * 0.01);
        robot.rotationServo.setPosition(currentPositionRotation + positionChangeRotation);

//        int extensionPositionOffset = (int)((double)currentPositionAngle*extensionMotorAngleFactor);
//
//        int positionChangeExtension = (int)((gamepad2.right_trigger - gamepad2.left_trigger) * 10);
//        currentPositionExtension += positionChangeExtension;
//        telemetry.addData("Extension ",currentPositionExtension);
//
//        if(currentPositionExtension > 1630 ) currentPositionExtension = 1630;
//        if(currentPositionExtension < 0) currentPositionExtension = 0;
//
//        extensionMotor.setTargetPosition(currentPositionExtension + extensionPositionOffset);

        if (gamepad1.x) robot.foundationServo.setPosition(0.83);

        if (gamepad1.y) robot.foundationServo.setPosition(0.5);

        if (gamepad2.a) {
            robot.grabberServo.setPosition(1.0);
            robot.grabberServoTwo.setPosition(0.25);
        }
        if (gamepad2.b) {
            robot.grabberServo.setPosition(0.5);
            robot.grabberServoTwo.setPosition(0.75);
        }

        if(gamepad2.dpad_right) {
            verticalServo.setPosition(0.5);
            timer.reset();
            rotationDirection = "right";
        }

        if(gamepad2.dpad_left) {
            verticalServo.setPosition(0.5);
            timer.reset();
            rotationDirection = "left";
        }

        if(gamepad2.dpad_up){
            verticalServo.setPosition(0.5);
            timer.reset();
            rotationDirection = "up";
        }


        telemetry.update();

        }


    }
}