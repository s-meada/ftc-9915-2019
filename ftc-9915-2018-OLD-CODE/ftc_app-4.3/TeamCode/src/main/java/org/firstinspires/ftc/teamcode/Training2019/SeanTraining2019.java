//package org.firstinspires.ftc.teamcode.Training2019;
////Reseting the encoder into motor.setMode(DcMotor.RunMode.Stop_And_REset_Encoder)
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@TeleOp(name = "Sean OpMode", group = "training")
//public class SeanTraining2019 extends OpMode {
//    ElapsedTime timer = new ElapsedTime();
//    DcMotor leftMotor;
//    DcMotor rightMotor;
//    Servo servo1;
//    @Override
//    public void init(){
//        leftMotor = hardwareMap.dcMotor.get("leftMotor");
//        rightMotor = hardwareMap.dcMotor.get("rightMotor");
//        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        timer.reset();
//        servo1 = hardwareMap.servo.get("servoArm");
//        servo1.setPosition(0.0);
//    }
//
//    @Override
//    public void loop(){
//        telemetry.addData("State", "Loop");
//        servo1.setPosition(0.5);
//        double speed = gamepad1.left_stick_y;
//        double strafe = gamepad1.left_stick_x;
//        double turn = gamepad1.right_stick_x;
//        if (timer.milliseconds()>2500){
//            leftMotor.setPower(0.0);
//        }
//        else{
//            leftMotor.setPower(0.25);
//        }
//        TargetPosition+=25*(gamepad2.right_trigger-gamepad1.left_trigger)
//
//        /*if(gamepad1.a){
//            leftMotor.setPower(0.25);
//            rightMotor.setPower(0.25);
//        }
//
//        else{
//            leftMotor.setPower(gamepad1.left_stick_y);
//        }
//        */
//
//    }
//}
