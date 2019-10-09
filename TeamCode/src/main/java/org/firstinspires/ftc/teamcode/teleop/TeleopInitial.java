package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.animation.ValueAnimator.REVERSE;




@TeleOp(name="TeleopInitial_C",group="Skystone")
public class TeleopInitial extends OpMode {

    ElapsedTime timer = new ElapsedTime();
    public static final double ANGLE_MOTOR_COUNTS_PER_REV = 7168.0;
    public static final double EXTENSION_MOTOR_COUNTS_PER_REV = 537.6;

    boolean slowMode = false;

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

    String rotationDirection = "up";
    int currentPositionAngle = 0;
    int currentPositionExtension = 0;
    double verticalServoAngleFactor = 1.333/ANGLE_MOTOR_COUNTS_PER_REV;
    double extensionMotorAngleFactor = EXTENSION_MOTOR_COUNTS_PER_REV/ANGLE_MOTOR_COUNTS_PER_REV;
    //boolean stoneTucked = false;

    @Override
    public void init() {

        //init driving motors

        rightFront = hardwareMap.dcMotor.get("RightFront");
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        leftBack = hardwareMap.dcMotor.get("LeftBack");


        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        //init motors

        angleMotor = hardwareMap.dcMotor.get("angleMotor");
        angleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        angleMotor.setTargetPosition(0);

        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setPower(1);


        extensionMotor = hardwareMap.dcMotor.get("extensionMotor");
        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(1);



        //init servos
        rotationServo = hardwareMap.servo.get("rotationServo");
        grabberServo = hardwareMap.servo.get("grabberServo");
        grabberServoTwo = hardwareMap.servo.get("grabberServoTwo");
        verticalServo = hardwareMap.servo.get("verticalServo");


        rotationServo.setPosition(0.5);
        grabberServo.setPosition(0.5);    //pos 0 = closed?
        grabberServo.setPosition(0.75);
        verticalServo.setPosition(0.5);   //vertical with arm horizontal

    }

    @Override
    public void loop() {

        //mecanum drive
        if (gamepad1.a) slowMode = true;
        if (gamepad1.b) slowMode = false;
        double speed = -gamepad1.left_stick_y; //may or may not be reversed
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


        leftFront.setPower(speed+strafe+turn);
        rightFront.setPower(speed-strafe-turn);
        rightBack.setPower(speed+strafe-turn);
        leftBack.setPower(speed-strafe+turn);

//
        int positionChangeAngle = (int)(-gamepad2.right_stick_y * 6);
        currentPositionAngle += positionChangeAngle;
        if(currentPositionAngle > 1400) currentPositionAngle = 1400;
        if(currentPositionAngle < 0) currentPositionAngle = 0;
        telemetry.addData("Angle",currentPositionAngle);

        angleMotor.setTargetPosition(currentPositionAngle);

        //calculates the adjustment to the vertical-position-holding servo and
        double verticalAngleOffset = (double)currentPositionAngle*verticalServoAngleFactor;
        double verticalServoPosition = 0.55 + verticalAngleOffset;
        //if(stoneTucked) verticalServoPosition = 0.93;
        if (timer.seconds() > 0.5){
            if (rotationDirection == "up") rotationServo.setPosition(0.5);
            if (rotationDirection == "right") rotationServo.setPosition(0.18);
            if (rotationDirection == "left") rotationServo.setPosition(0.82); //TEST
        }
        if (timer.seconds() > 1){
            verticalServo.setPosition(verticalServoPosition);
        }


        int extensionPositionOffset = (int)((double)currentPositionAngle*extensionMotorAngleFactor);


//        int currentPositionExtension = extensionMotor.getCurrentPosition();
        int positionChangeExtension = (int)((gamepad2.right_trigger - gamepad2.left_trigger) * 10);
        currentPositionExtension += positionChangeExtension;
        telemetry.addData("Extension ",currentPositionExtension);

        if(currentPositionExtension > 1730 ) currentPositionExtension = 1730;
        if(currentPositionExtension < 0) currentPositionExtension = 0;

        extensionMotor.setTargetPosition(currentPositionExtension + extensionPositionOffset);



        if(gamepad2.a) { //test positions TBD
            grabberServo.setPosition(1.0);
            grabberServoTwo.setPosition(0.25);
        }
        if(gamepad2.b) {
            grabberServo.setPosition(0.5);
            grabberServoTwo.setPosition(0.75);
        }

        if(gamepad2.dpad_right){
            verticalServo.setPosition(0.5);
            timer.reset();
            rotationDirection = "right";
        }

        if(gamepad2.dpad_up){
            verticalServo.setPosition(0.5);
            timer.reset();
            rotationDirection = "up";
        }

        if(gamepad2.dpad_left){
            verticalServo.setPosition(0.5);
            timer.reset();
            rotationDirection = "left";
        }
        //if(gamepad2.dpad_up) stoneTucked = true;
        //if(gamepad2.dpad_down) stoneTucked = false;

        double currentPositionRotation = rotationServo.getPosition();
        int positionChangeRotation = (int) (gamepad2.left_stick_x * 0.01);
        rotationServo.setPosition(currentPositionRotation + positionChangeRotation);

        telemetry.update();

    }



}