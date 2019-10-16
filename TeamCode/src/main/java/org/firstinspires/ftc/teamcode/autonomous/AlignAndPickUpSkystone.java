package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_TWO_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_TWO_OPEN_POSITION;

@Autonomous(name = "Ashley Auto", group = "auto")
public class AlignAndPickUpSkystone extends LinearOpMode {

    Robot robot = new Robot();
    ElapsedTime timer = new ElapsedTime();

    double robotXDistanceFromSkystoneCenter;
    double robotYDistanceFromSkystoneCenter;
    double distanceForArmToExtend;
    int armUpAngle = 20;
    int armAngleOnSkystone = -40;

    static final int MOVE_ARM_UP                    = 1;
    static final int FIND_CENTER_OF_SKYSTONE_VS_ARM = 2;
    static final int ADJUST_ROBOT_POSITION          = 3;
    static final int MOVE_ARM_OUT                   = 4;
    static final int MOVE_SERVOS                    = 5;
    static final int MOVE_ARM_DOWN                  = 6;
    static final int STRAFE_TO_SKYSTONE             = 7;
    static final int GRAB_SKYSTONE                  = 8;
    static final int PUT_ARM_DOWN                   = 9;
    static final int STRAFE_AWAY_FROM_SKYSTONE      = 10;

    static final int STATE_END                      = 11;

    int state = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initForRunToPosition(hardwareMap);

        SkystoneVuforiaData vision = new SkystoneVuforiaData(hardwareMap, robot);

        waitForStart();

        vision.targetsSkyStone.activate();

        while(opModeIsActive()) {
            telemetry.addData("State", state);

            switch(state) {
                case MOVE_ARM_UP:
                    if(robot.moveArm(armUpAngle, 0)) {
                        robot.light.setPower(1);
                        goToNextState();
                    }
                    break;

                case FIND_CENTER_OF_SKYSTONE_VS_ARM:
                    // The getSkystoneCoordinates() method returns null if the skystone is not detected
                    HashMap<String, Float> skyStoneCoordinates = vision.getSkystoneCoordinates();
                    if(skyStoneCoordinates != null){
                        robotXDistanceFromSkystoneCenter = skyStoneCoordinates.get("X");
                        robotYDistanceFromSkystoneCenter = skyStoneCoordinates.get("Y");
                        telemetry.addData("Skystone Pos (in)", "(X, Y) = %.1f, %.1f",
                                robotXDistanceFromSkystoneCenter, robotYDistanceFromSkystoneCenter);
                        robot.light.setPower(0);
                        goToNextState();
                    }
                    else {
                        telemetry.addLine("No Skystone Detected");
                    }
                    telemetry.update();
                    break;

                case ADJUST_ROBOT_POSITION:
                    if(robot.drive(0.75, -robotYDistanceFromSkystoneCenter - 1.0)) {
                        goToNextState();
                    }
                    break;

                case MOVE_ARM_OUT:
                    distanceForArmToExtend = -robotXDistanceFromSkystoneCenter + 7.75;
                    telemetry.addData("Distance from skystone", distanceForArmToExtend);
                    telemetry.update();
                    if(robot.moveArm(armUpAngle, distanceForArmToExtend)) {
                        goToNextState();
                    }
                    break;

                case MOVE_SERVOS:
                    robot.angleServo.setPosition(0.5);
                    robot.grabberServo.setPosition(GRABBER_SERVO_OPEN_POSITION);
                    robot.grabberServoTwo.setPosition(GRABBER_SERVO_TWO_OPEN_POSITION);
                    goToNextState();
                    break;

                case MOVE_ARM_DOWN:
                    if(robot.moveArm(armAngleOnSkystone, distanceForArmToExtend)) {
                        goToNextState();
                    }
                    break;

                case STRAFE_TO_SKYSTONE:
                    if(robot.strafe(0.75, 5)) {
                        timer.reset();
                        goToNextState();
                    }
                    break;

                case GRAB_SKYSTONE:
                    robot.grabberServo.setPosition(GRABBER_SERVO_CLOSE_POSITION);
                    robot.grabberServoTwo.setPosition(GRABBER_SERVO_TWO_CLOSE_POSITION);
                    if(timer.milliseconds() > 700) {
                        goToNextState();
                    }
                    break;

                case PUT_ARM_DOWN:
                    if(robot.moveArm(-25, 16)){
                        goToNextState();
                    }
                    break;

                case STRAFE_AWAY_FROM_SKYSTONE:
                    if(robot.strafe(0.75, -10)) {
                        goToNextState();
                    }
                    break;

                default:
                    state = STATE_END;
                    telemetry.update();
                    break;
            }

        }
    }

    // Increment the state variable to go to the next state
    public void goToNextState() {
        state++;
    }

    // Go to a specific state
    public void goToState(int newState) {
        state = newState;
    }
}
