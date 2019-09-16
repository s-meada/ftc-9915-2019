package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static android.animation.ValueAnimator.REVERSE;

@TeleOp(name="TeleopInitial",group="Skystone")
public class TeleopInitial extends OpMode {
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor leftBack;

    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("RightFront");
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {
        double speed = gamepad1.left_stick_y;//Determines
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        leftFront.setPower(speed+strafe+turn);
        rightFront.setPower(speed-strafe-turn);
        rightBack.setPower(speed+strafe-turn);
        leftBack.setPower(speed-strafe+turn);
    }



}
