package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    Servo foundationServo;

    int armPosition = 0;
    int extensionPosition = 0;

    boolean slowMode = false;
    double speedMultiplier = 1;

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


        extensionMotor = hardwareMap.dcMotor.get("extensionMotor");
        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        //init servos
        rotationServo = hardwareMap.servo.get("rotationServo");
        grabberServo = hardwareMap.servo.get("grabberServo");
        foundationServo = hardwareMap.servo.get("foundationServo");

        rotationServo.setPosition(0);
        grabberServo.setPosition(0);    //pos 0 = closed?
        foundationServo.setPosition(0.15);
    }

    @Override
    public void loop() {


        telemetry.addData("Angle:", armPosition);
        telemetry.addData("Extension:",extensionPosition);

        //mecanum drive
        double speed = -gamepad1.left_stick_y; //may or may not be reversed
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad1.a) {
            slowMode = true;
        }

        else if (gamepad1.b) {
            slowMode = false;
        }
        speedMultiplier = slowMode ? 0.5 : 1.0;
        leftFront.setPower(speedMultiplier*(speed+strafe+turn));
        rightFront.setPower(speedMultiplier*(speed-strafe-turn));
        rightBack.setPower(speedMultiplier*(speed+strafe-turn));
        leftBack.setPower(speedMultiplier*(speed-strafe+turn));

        armPosition += (int)(gamepad2.right_stick_y * 50);
        if(armPosition > 1300) {
            armPosition = 1300;
        }
        if(armPosition < 0) {
            armPosition = 0;
        }
        angleMotor.setTargetPosition(armPosition);
        angleMotor.setPower(1);



        extensionPosition += (int)((gamepad2.right_trigger - gamepad2.left_trigger) * 75);
        if(extensionPosition > 1600) {
            extensionPosition = 1600;
        }
        if(extensionPosition < 0) {
            extensionPosition = 0;
        }
        extensionMotor.setTargetPosition(extensionPosition);
        extensionMotor.setPower(1);


        if(gamepad2.a) { //test positions TBD
            grabberServo.setPosition(0.9);
        }
        if(gamepad2.b) {
            grabberServo.setPosition(0.3);
        }

        if(gamepad2.y){
            rotationServo.setPosition(0.5); //x = 0; not tested
        }

        if(gamepad2.x){
            rotationServo.setPosition(0); //y = 0; not tested
        }

        double currentPositionRotation = rotationServo.getPosition();
        int positionChangeRotation = (int) (gamepad2.left_stick_x * 25);

        if(gamepad2.dpad_down){
            foundationServo.setPosition(0.82);
        }

        if(gamepad2.dpad_up){
            foundationServo.setPosition(0.15);
        }
    }



}
//***Needs limits for angleMotor and extensionMotor***