package org.firstinspires.ftc.teamcode.Training2019;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "X OpMode", group = "training")
public class RobotcodeJava extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo servoArm;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        servoArm = hardwareMap.servo.get("servoArm");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servoArm.setPosition(1.0);

    }

    @Override
    public void loop() {
        telemetry.addData("State", "Loop");
        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.right_stick_y);
        if(gamepad1.a){
            servoArm.setPosition(1.0);
        }
        if(gamepad1.b){
            servoArm.setPosition(0.6);
        }
    }
}
