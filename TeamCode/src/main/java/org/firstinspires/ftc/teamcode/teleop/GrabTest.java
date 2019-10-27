package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Robot;


@TeleOp(name = "GrabTest", group = "test")
public class GrabTest extends OpMode {

    Robot robot = new Robot();

    int state = 1;
    int blockFirstEdge;
    int blockSecondEdge;
    double blockRange;
    int grabPosition;


    static final int DRIVE_TO_BLOCK = 1;
    static final int DRIVE_ALONG_BLOCK = 2;
    static final int DRIVE_TO_EDGE = 3;
    static final int DRIVE_TO_GRAB = 4;

    static final int STATE_END = 5;

    @Override
    public void init() {
        robot.initRegular(hardwareMap);

    }



       public void loop() {
           double range = robot.blueDistanceSensor.getDistance(DistanceUnit.MM);
           telemetry.addData("Current State: ", state);
           telemetry.addData("Distance in mm: ", range);
           telemetry.update();

           switch (state) {
               case DRIVE_TO_BLOCK:
                   if(!robot.moveArmXY(14.5,2.))break;


                   robot.grabberServo.setPosition(robot.GRABBER_SERVO_OPEN_POSITION);
                   robot.grabberServoTwo.setPosition(robot.GRABBER_SERVO_TWO_OPEN_POSITION);
                       robot.drivePower(0.15, 0.15, 0.15, 0.15);
                   if (range > 300) break;
                   blockFirstEdge = robot.leftFrontMotor.getCurrentPosition();
                   goToNextState();

                   break;

               case DRIVE_ALONG_BLOCK:

                   if (robot.leftFrontMotor.getCurrentPosition() < blockFirstEdge + robot.robotTicksPerInch)
                       break;
                   blockRange = range;
                   goToNextState();

                   break;

               case DRIVE_TO_EDGE:

                   if (range < blockRange + 50) break;
                   blockSecondEdge = robot.leftFrontMotor.getCurrentPosition();
                   grabPosition = (blockFirstEdge + blockSecondEdge) / 2 + (int) (7 * robot.robotTicksPerInch);
                   goToNextState();

                   break;

               case DRIVE_TO_GRAB:

                   if (robot.leftFrontMotor.getCurrentPosition() < grabPosition) break;
                   robot.stop();
                   if(robot.moveArmXY(blockRange / 25.4 + 12.0, -3.0)) {
                       robot.grabberServo.setPosition(robot.GRABBER_SERVO_CLOSE_POSITION);
                       robot.grabberServoTwo.setPosition(robot.GRABBER_SERVO_TWO_CLOSE_POSITION);
                   }
                   break;


               default:
                   state = STATE_END;
                   break;

           }
       }





    public void goToNextState() { state++; }



}
