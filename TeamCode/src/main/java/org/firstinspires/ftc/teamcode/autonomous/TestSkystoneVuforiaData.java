package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.Robot;

import java.util.HashMap;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name = "Test SkystoneVuforiaData", group = "test")
@Disabled
public class TestSkystoneVuforiaData extends LinearOpMode {
    Robot robot = new Robot();
    SkystoneVuforiaData vision;
    boolean activated = false;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.initRegular(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if (!activated) {
                // If this is in init() in iterative OpMode, the program will say stuck in init() and restart the app before vuforia can finish initializing.
                // This may be part of the crashing problem.
                vision = new SkystoneVuforiaData(hardwareMap, robot);
                vision.targetsSkyStone.activate();
                activated = true;
            }
            // The getSkystoneCoordinates() method returns null if the skystone is not detected
            HashMap<String, Double> skyStoneCoordinates = vision.getSkystoneCoordinates();
            if (skyStoneCoordinates != null) {
                telemetry.addData("Skystone Pos (in)", "(X, Y, Y Corrected) = %.3f, %.3f, %.3f",
                        skyStoneCoordinates.get("X"), skyStoneCoordinates.get("Y"), skyStoneCoordinates.get("Y Corrected"));
                telemetry.addData("Y Offset","%.3f",  skyStoneCoordinates.get("Y Offset"));
                telemetry.addData("Heading","%.3f", skyStoneCoordinates.get("Heading"));
                robot.light.setPower(0);
            } else {
                telemetry.addLine("No Skystone Detected");
            }
            telemetry.update();
        }
    }
}
