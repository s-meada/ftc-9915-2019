package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Program Switch Test", group = "test")
public class ProgramSwitchTest extends OpMode {
    Robot robot = new Robot();

    @Override
    public void init() {
        robot.initRegular(hardwareMap);
    }

    @Override
    public void loop() {
        boolean one = robot.programSwitchOne.getState();
        boolean two = robot.programSwitchTwo.getState();
        int program = 0;
        if (!one && !two) program = 0;
        if (one && !two) program = 1;
        if (!one && two) program = 2;
        if (one && two) program = 3;
        telemetry.addData("Program Switch: ", program);
    }
}