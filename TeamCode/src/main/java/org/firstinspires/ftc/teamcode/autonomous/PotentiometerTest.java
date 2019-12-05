package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

@Autonomous(name = "PotentiometerTest", group = "test")
@Disabled
public class PotentiometerTest extends OpMode {
    Robot robot = new Robot();

    @Override
    public void init() {
        robot.initRegular(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Potentiometer Voltage: ", robot.potentiometerOne.getVoltage());
    }
}