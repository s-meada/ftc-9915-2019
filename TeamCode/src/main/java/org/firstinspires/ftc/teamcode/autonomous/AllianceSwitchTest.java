package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "Alliance Switch Test", group = "test")
public class AllianceSwitchTest extends OpMode {
    Robot robot = new Robot();

    @Override
    public void init() {
        robot.initRegular(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Alliance Switch", robot.allianceSwitch.getState());
    }
}
