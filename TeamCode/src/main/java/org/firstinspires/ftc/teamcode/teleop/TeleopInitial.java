package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static android.animation.ValueAnimator.REVERSE;

@TeleOp(name="TeleopInitial",group="Skystone")
public class TeleopInitial extends OpMode {

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
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor.setTargetPosition(0);

        extensionMotor = hardwareMap.dcMotor.get("extensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setTargetPosition(0);


        //init servos
        rotationServo = hardwareMap.servo.get("rotationServo");
        grabberServo = hardwareMap.servo.get("grabberServo");

        rotationServo.setPosition(0);
        grabberServo.setPosition(0);
    }

    @Override
    public void loop() {

        //mecanum drive
        double speed = -gamepad1.left_stick_y; //may or may not be reversed
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        leftFront.setPower(speed+strafe+turn);
        rightFront.setPower(speed-strafe-turn);
        rightBack.setPower(speed+strafe-turn);
        leftBack.setPower(speed-strafe+turn);


        int currentPositionAngle = angleMotor.getCurrentPosition();
        int positionChangeAngle = (int)(gamepad2.right_stick_y * 10);
        angleMotor.setTargetPosition(currentPositionAngle + positionChangeAngle);
        angleMotor.setPower(1);

        int currentPositionExtension = extensionMotor.getCurrentPosition();
        int positionChangeExtension = (int)((gamepad2.right_trigger - gamepad2.left_trigger) * 10);
        extensionMotor.setTargetPosition(currentPositionExtension + positionChangeExtension);
        extensionMotor.setPower(1);

        if(gamepad2.a) { //test positions TBD
            grabberServo.setPosition(1.0);
        }
        if(gamepad2.b) {
            grabberServo.setPosition(0);
        }

        if(gamepad2.y){
            rotationServo.setPosition(0.5); //x = 0
        }

        if(gamepad2.x){
            rotationServo.setPosition(0); //y = 0
        }

    }



}
//***Needs limits for angleMotor and extensionMotor***