package org.firstinspires.ftc.teamcode.Training2019;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//right bottom: moves up
//right top: moves down
//left bottom:moves up
//right top: moves down
//extensions doesn't work
//turning doesn't work
@TeleOp(name = "A OpMode", group = "training")
public class Test_Real extends OpMode {
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor leftBack;
    DcMotor extensionMotor;
    DcMotor angleMotor;
    Servo leftGate;
    Servo rightGate;
    CRServo collectionServo;

    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftGate = hardwareMap.servo.get("leftGate");
        rightGate = hardwareMap.servo.get("rightGate");
        collectionServo = hardwareMap.crservo.get("collectionServo");
        extensionMotor = hardwareMap.dcMotor.get("extensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        angleMotor = hardwareMap.dcMotor.get("angleMotor");
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Declares the motors
        //rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftGate.setPosition(0.8);
        rightGate.setPosition(0.85);
        //Sets closed position

        extensionMotor.setTargetPosition(0);
        angleMotor.setTargetPosition(0);
    }

    @Override
    public void loop() {
        telemetry.addData("State", "Loop");
        double TargetPosition = 0;
        double TargetPosition_angle = 0;
        double speed = gamepad1.left_stick_y;//Determines
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        //mecanum drive
        leftFront.setPower(speed+strafe+turn);
        rightFront.setPower(speed-strafe-turn);
        rightBack.setPower(speed+strafe-turn);
        leftBack.setPower(speed-strafe+turn);
        //extension section
        TargetPosition+=35*(gamepad2.right_trigger-gamepad2.left_trigger);
        //The multiplier increases the speed of the motor making it effective
        if (TargetPosition>9000){
            //if the target position is over the maximum, set value to the maximum
            TargetPosition = 9000;
        }
        if (TargetPosition<0){
            TargetPosition = 0;
        }
        extensionMotor.setPower(1.0);
        telemetry.addData("Extension location", "TargetPosition ");
        extensionMotor.setTargetPosition((-1)*(int) TargetPosition);
        //Go to the target position of the angle
        //angle section
        TargetPosition_angle += 6.25*gamepad2.right_stick_y;
        if(TargetPosition_angle>2150){
            //above maximum level
            TargetPosition_angle = 2150;
        }
        if (TargetPosition_angle<0){
            //below minimum level
            TargetPosition_angle = 0;
        }
        angleMotor.setPower(0.5);
        angleMotor.setTargetPosition((-1)*(int) TargetPosition_angle);
        //gate section
        if (gamepad2.dpad_up){
            //Sets the rightgate open
            rightGate.setPosition(1.0);
        }
        if (gamepad2.dpad_down){
            //Sets the leftgate open
            leftGate.setPosition(0.3);
        }
        if (gamepad2.dpad_left){
            //Sets both gates close
            leftGate.setPosition(0.8);
            rightGate.setPosition(0.85);
        }
        //collector section
        if (gamepad2.y){
            //start collecting
            collectionServo.setPower(0.5);
        }
        if (gamepad2.a){
            //stop collecting
            collectionServo.setPower(0);
        }
        if (gamepad2.x){
            //reverse speed
            collectionServo.setPower(-0.5);
        }

    }


}
