//package org.firstinspires.ftc.teamcode.Training2019;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//public class AdvancedRobotCode extends OpMode {
//    CRServo crServo;
//    DcMotor motor;
//
//    @Override
//    public void init(){
//        crServo = hardwareMap.crservo.get("crServo");
//        motor = hardwareMap.dcMotor.get("");
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//    }
//
//    @Override
//    public void loop() {
//        crServo.setPower(0.5);
//        motor.setTargetPosition(1120);
//        motor.setPower(1.0);
//        double speed = gamepad1.left_stick_y;
//        double strafe = gamepad1.left_stick_x;
//        double turn = gamepad1.right_stick_x;
//        robot.leftFrontDriveMotor.setPower(speed+strafe+turn);
//        robot.rightFrontDriveMotor.setPower(speed-strafe-turn);
//        robot.rightBackDriveMotor.setPower(speed-strafe+turn);
//        robot.leftBackDriveMotor.setPower(speed+strafe-turn);
//        /*motor runs 1 full rotation
//        code for mecanum drive6
//        The sideway x joystick is for moving right and left
//        The updown x joysitck is forward and backward
//        The sideway y joystick is for turning left and right
//        double speed = gamepad1.left_stick_y;
//        double strafe = gamepad1.left_stick_x;
//        double tujoystickrn = gamepad1.right_stick_x;
//        left  is moved for diagnol movement
//        this.leftFrontDriveMotor.setPower(speed+strafe+turn);
//        this.rightFrontDriveMotor.setPower(speed-trafe-turn);
//        this.rightBackDriveMotor.setPower(speed-strafe+turn);
//        this.leftBackDriveMotor.setPower(speed+strafe-turn);
//        relicexntederTargetPOsition+=25*(gamepad2.right_trigger-gamepad1.left_trigger)
//        if relic extendertargetposition < 0
//            relicextendertargetposition = 0
//        robto.relicarmextender.settargetposition(robotextendertargetposition)
//         if relicrotatertargetposition >)
//             = 0
//         robot.relicarmrotater.settarrelicrotatertargetpositiongetposision(relicrotatertargetposision)
//         robot.relicarmrotater.setpower(1.0)
//         */
//    }
//}
