package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Gyro Test", group = "test")
public class GyroTest extends OpMode {
    Robot robot = new Robot();
    @Override
    public void init() {
        robot.initRegular(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Gyro Turning Angle", robot.getTurningAngle());
        telemetry.addData("Angle in Radians", Math.toRadians(robot.getTurningAngle()));
    }
}
