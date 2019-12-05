package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous (name = "TestingDistanceSensors", group = "test")
@Disabled
public class TestingDistanceSensors extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initForRunToPosition(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Blue distance sensor: ", robot.blueDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Red distance sensor: ", robot.redDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
