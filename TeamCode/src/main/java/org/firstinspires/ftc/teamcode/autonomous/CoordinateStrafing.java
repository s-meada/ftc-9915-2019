package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Coordinate Strafing Tests", group = "test")
public class CoordinateStrafing extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            if(robot.driveMecanum(0.5, 0.5, Math.PI/4, 12)) {
                robot.stop();
            }
        }
    }
}