/*
Robot detects the outerskystone. It strafes towards skystone. Stores range position. Moves arm. Grips skystone.
Retracts arm. Drives forward 12 inches beyond the bridge. Moves arm. Releases Skystone. Drives backwards 16 inches
behind the stored position. Drives forward 12 inches beyond the bridge. Moves arm. Releases skystone. Drives backwards
16 inches to park.
 */
package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.Robot;

import java.util.HashMap;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_TWO_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_TWO_OPEN_POSITION;

@Autonomous(name = "Master Auto", group = "auto")
public class PathThreeTwoSkystonesandPark extends LinearOpMode {

    // Global variables go before runOpMode()
    Robot robot = new Robot();

    int superMasterState = 1;
    int masterState = 1;
    int subState = 1;
    int count = 0;




    AllianceColor allianceColor = AllianceColor.BLUE;

    //---Super Master States---//
    static final int FIRST_SKYSTONE = 1;
    static final int SECOND_SKYSTONE = 2;

    // --- Master States --- //

    /* Each autonomous coder programmed a part of the autonomous code separately. In the MasterAutonomous class, all of the member's programs are combined
    and run linearly. Here, all of the states of each member's programs are defined.
     */
    static final int STRAFE_TO_SKYSTONE_V2 = 1;
    static final int ALIGN_AND_PICK_UP_SKYSTONE = 2;
    static final int MOVE_FOUNDATION = 3;


    // --- StrafeTowardsDetectedSkystone States and Variables --- //
    // Capital letters and underscores for constants
    static final int STRAFE_TO_VIEWING_POSITION = 1;
    static final int DETECT_SKYSTONE = 2;
    static final int STATE_END = 3;

    int skyStonePosition = 1;
    double strafePower = 0.5;
    double drivePower = 0.5;
    double viewingPosition = 18.5; //strafe

    double skyStoneCoordinateX;
    double skyStoneCoordinateY;


    // --- AlignAndPickUpSkystone States and Variables --- //

    ElapsedTime timer = new ElapsedTime();

    double robotXDistanceFromSkystoneCenter;
    double robotYDistanceFromSkystoneCenter;
    double distanceForArmToExtend;
    int armUpAngle = 45;
    double armAngleOnSkystone = 1;
    double distanceFromSkystoneOffset = 0;
    int maxArmExtensionDistance = 25;

    static final int MOVE_ARM_UP = 1;
    static final int FIND_CENTER_OF_SKYSTONE_VS_ARM = 2;
    static final int MOVE_ARM_OUT = 3;
    static final int MOVE_SERVOS = 4;
    static final int MOVE_ARM_DOWN = 5;
    static final int STRAFE_TO_SKYSTONE_2_FIRST = 6;
    static final int ADJUST_ROBOT_POSITION = 7;
    static final int STRAFE_TO_SKYSTONE_2_SECOND = 8;
    static final int FINISH_ARM_EXTENSION = 9;
    static final int GRAB_SKYSTONE = 10;
    static final int PUT_ARM_DOWN = 11;

    static final int STATE_END_2 = 12;


    // --- MovingFoundation States and Variables --- //
    // modified moving foundation, will change name to DROP_SKYSTONE_AND_RETURN//
    /* In the first repeat, I end at REPEAT_CODE. In the second repeat, I skip REPEAT_CODE and go to DRIVE_BACK_TO_PARK. */

    static final int DRIVE_AWAY_FROM_BLOCK = 1;
    static final int ADJUST_ANGLE = 2;
    static final int DRIVE_TO_WALL_1 = 3;
    static final int MOVE_ARM = 4;
    static final int RELEASE_SKYSTONE = 5;
    static final int RAISE_ARM = 6;
    static final int RETRACT_ARM = 7;
    static final int DRIVE_BACK_TO_QUARRY = 8;
    static final int REPEAT_CODE = 9;
    static final int DRIVE_BACK_TO_PARK = 10;
    static final int STOP = 11;
    static final int END_STATE = 12;

    double distance;
    double angleAdjustmentSign = 0;
    double angle;


    @Override
    public void runOpMode() throws InterruptedException {
        // init()
        robot.initForRunToPosition(hardwareMap);
        SkystoneVuforiaData vision = new SkystoneVuforiaData(hardwareMap, robot);
        superMasterState=FIRST_SKYSTONE;

        //Here the robot discerns the alliance color based on the switch
        if (robot.allianceSwitch.getState()) {
            allianceColor = AllianceColor.BLUE;
        } else {
            allianceColor = AllianceColor.RED;
        }

        waitForStart(); // MUST add this yourself


        vision.targetsSkyStone.activate();

        while (opModeIsActive()) {  // MUST add this yourself
            telemetry.addData("Master State", masterState);
            telemetry.update();
            switch (masterState) {
                case STRAFE_TO_SKYSTONE_V2:
                    if (StrafeTowardsDetectedSkystoneV2(vision)) {
                        // Necessary to make driving in the next state work after strafing
                        goToNextMasterState();
                    }
                    break;

                case ALIGN_AND_PICK_UP_SKYSTONE:
                    if (AlignAndPickUpSkystone(vision)) {
                        goToNextMasterState();
                    }
                    break;

                case MOVE_FOUNDATION:
                    if (MoveFoundation()) {
                        goToNextMasterState();
                    }
                    break;
            }

        }
    }


    /*
     * For each of these methods, you can add more things you want the robot to do each time it goes to a new subState
     */
    // Increment the subState variable to go to the next subState
    public void goToNextSubState() {
        subState++;
    }

    // Go to a specific subState
    public void goToSubState(int newState) {
        subState = newState;
    }

    public void goToNextMasterState() {
        robot.resetChassisEncoders();
        subState = 1;
        masterState++;
    }

    public void goToMasterState(int newState) {
        robot.resetChassisEncoders();
        subState = 1;
        masterState = newState;
    }

    /* This is Trevor's code. The robot moves to a pre-determined viewing position to detect skystones


     */
    public boolean StrafeTowardsDetectedSkystoneV2(SkystoneVuforiaData vision) {
        boolean isComplete = false;
        // loop()
        telemetry.addData("Current State", subState);

        switch (subState) {
            case STRAFE_TO_VIEWING_POSITION:
                if (robot.strafe(strafePower, viewingPosition)) ;
            {
                robot.light.setPower(1.0);
                goToNextSubState();
            }
            break;

            /*
            This is Ashley's code. The robot uses Vuforia to locate the coordinates of the skystone. After detecting the skystone, the robot aligns itself, extends its arm,
            and picks up the skystone.

             */
            case DETECT_SKYSTONE:
                HashMap<String, Double> skyStoneCoordinates = vision.getSkystoneCoordinates();
                if (skyStoneCoordinates != null) {
                    robotXDistanceFromSkystoneCenter = skyStoneCoordinates.get("X");
                    robotYDistanceFromSkystoneCenter = skyStoneCoordinates.get("Y");
                    telemetry.addData("Skystone Pos (in)", "(X, Y) = %.1f, %.1f",
                            robotXDistanceFromSkystoneCenter, robotYDistanceFromSkystoneCenter);
                    robot.light.setPower(0);
                    Log.i("MasterAutonomous", "Robot Y Distance from Skystone: " + (-robotYDistanceFromSkystoneCenter));
                    Log.i("MasterAutonomous", "Robot X Distance from Skystone: " + (-robotXDistanceFromSkystoneCenter));
                    goToNextSubState();
                } else {
                    telemetry.addLine("No Skystone Detected");
                }
                telemetry.update();
                break;

            default:
                subState = STATE_END;
                isComplete = true;
                break;

        }
        return isComplete;
    }

    public boolean AlignAndPickUpSkystone(SkystoneVuforiaData vision) {
        boolean isComplete = false;

        telemetry.addData("State", subState);

        switch (subState) {
            case MOVE_ARM_UP:
                if (robot.moveArm(armUpAngle, 0)) {
                    robot.light.setPower(1);
                    goToNextSubState();
                }
                break;

            case FIND_CENTER_OF_SKYSTONE_VS_ARM:
                // The getSkystoneCoordinates() method returns null if the skystone is not detected
                HashMap<String, Double> skyStoneCoordinates = vision.getSkystoneCoordinates();
                if (skyStoneCoordinates != null) {
                    robotXDistanceFromSkystoneCenter = skyStoneCoordinates.get("X");
                    robotYDistanceFromSkystoneCenter = skyStoneCoordinates.get("Y");
                    telemetry.addData("Skystone Pos (in)", "(X, Y) = %.1f, %.1f",
                            robotXDistanceFromSkystoneCenter, robotYDistanceFromSkystoneCenter);
                    robot.light.setPower(0);
                    Log.i("MasterAutonomous", "Robot Y Distance from Skystone: " + (-robotYDistanceFromSkystoneCenter));
                    Log.i("MasterAutonomous", "Robot X Distance from Skystone: " + (-robotXDistanceFromSkystoneCenter));
                    goToNextSubState();
                } else {
                    telemetry.addLine("No Skystone Detected");
                }
                telemetry.update();
                break;

            case MOVE_ARM_OUT:
                distanceForArmToExtend = -robotXDistanceFromSkystoneCenter + 8;
                if (distanceForArmToExtend > maxArmExtensionDistance) {
                    distanceFromSkystoneOffset = distanceForArmToExtend - maxArmExtensionDistance;
                }

                telemetry.addData("Distance from skystone", distanceForArmToExtend);
                telemetry.update();
                if (robot.moveArm(armUpAngle, distanceForArmToExtend - 10)) {
                    robot.light.setPower(0);
                    goToNextSubState();
                }
                break;

            case MOVE_SERVOS:
                robot.angleServo.setPosition(0.55);
                robot.grabberServo.setPosition(GRABBER_SERVO_OPEN_POSITION);
                robot.grabberServoTwo.setPosition(GRABBER_SERVO_TWO_OPEN_POSITION);
                goToNextSubState();
                break;

            case MOVE_ARM_DOWN:
                if (robot.moveArm(armAngleOnSkystone, distanceForArmToExtend - 5)) {
                    goToNextSubState();
                }
                break;

            case STRAFE_TO_SKYSTONE_2_FIRST:
                if (robot.strafe(0.25, 5)) {
                    goToNextSubState();
                }
                break;

            case ADJUST_ROBOT_POSITION:
                if (-robotYDistanceFromSkystoneCenter > 3) {
                    if (robot.drive(0.75, -robotYDistanceFromSkystoneCenter - 3)) {
                        goToNextSubState();
                    }
                } else {
                    if (robot.drive(0.75, -robotYDistanceFromSkystoneCenter)) {
                        goToNextSubState();
                    }
                }
                break;

            case STRAFE_TO_SKYSTONE_2_SECOND:
                if (robot.strafe(0.25, 10)) {
                    goToNextSubState();
                }
                break;

            case FINISH_ARM_EXTENSION:
                if (robot.moveArm(armAngleOnSkystone, distanceForArmToExtend)) {
                    timer.reset();
                    goToNextSubState();
                }
                break;

            case GRAB_SKYSTONE:
                robot.grabberServo.setPosition(GRABBER_SERVO_CLOSE_POSITION);
                robot.grabberServoTwo.setPosition(GRABBER_SERVO_TWO_CLOSE_POSITION);
                if (timer.milliseconds() > 500) {
                    goToNextSubState();
                }
                break;

            case PUT_ARM_DOWN:
                if (robot.moveArm(1, 15)) {
                    goToNextSubState();
                }
                break;

            default:
                isComplete = true;
                subState = STATE_END_2;
                telemetry.update();
                break;
        }
        return isComplete;
    }

    /*This is Avery's code. Gripping the skystone, the robot moves away from the stones and towards the end wall, where it drags the foundation
    to the building zone. The robot uses a gyro sensor to precisely turn to the correct angle from which to drag the skystone.*/
    /*the first time, superMasterState = FIRST_SKYSTONE. The second time, superMasterState = SECOND_SKYSTONE */
    public boolean MoveFoundation() {
        boolean isComplete = false;
        boolean isBlue = allianceColor == AllianceColor.BLUE;
        boolean timerReset = false;

        telemetry.addData("Blue Distance: ", robot.blueDistanceSensor.getDistance(INCH));
        telemetry.addData("Red Distance: ", robot.redDistanceSensor.getDistance(INCH));
        telemetry.addData("State: ", subState);
        telemetry.update();

        switch (subState) {
            case DRIVE_AWAY_FROM_BLOCK:
                    /*robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.drivePower(0.5, -0.5, -0.5, 0.5);
                    if (isBlue) {
                        if (robot.blueDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
                            robot.stop();
                            goToNextSubState();
                        }
                    } else {
                        if (robot.redDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
                            robot.stop();
                            goToNextSubState();
                        }
                    }
                     */

                if (robot.strafe(0.50, -10)) {
                    robot.stop();
                    goToNextSubState();
                }
                break;

            case ADJUST_ANGLE:
                angle = robot.getTurningAngle();
                telemetry.addData("Angle", angle);
                if (angle > -0.4) {
                    angleAdjustmentSign = -1;
                } else if (angle < -0.1) {
                    angleAdjustmentSign = 1;
                } else {
                    robot.stop();
                    Log.i("MasterAutonomous", "Gyro Angle: " + angle + " degrees");
                    Log.i("MasterAutonomous", "Finished Angle adjustment");
                    goToNextSubState();
                    break;
                }

                robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftFrontMotor.setPower(0.05 * angleAdjustmentSign);
                robot.rightFrontMotor.setPower(0.05 * -angleAdjustmentSign);
                robot.leftBackMotor.setPower(0.05 * angleAdjustmentSign);
                robot.rightBackMotor.setPower(0.05 * -angleAdjustmentSign);

                angle = robot.getTurningAngle();
                if (angle > -0.4 && angle < -0.1) {
                    telemetry.addData("Angle", angle);
                    Log.i("MasterAutonomous", "Gyro Angle: " + angle + " degrees");
                    Log.i("MasterAutonomous", "Finished Angle adjustment");
                    goToNextSubState();
                }
                break;


            case DRIVE_TO_WALL_1:

                //robot.setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
                double distanceToEndOfQuarry = isBlue ? 12 + robotYDistanceFromSkystoneCenter : -robotYDistanceFromSkystoneCenter - 7;

                int distanceToGate = 36;

                if (isBlue) {
                    if (robot.drive(0.9, distanceToGate)) {
                        Log.i("MasterAutonomous", "Distance to Drive: " + (distanceToGate) + " INCHES");
                        robot.stop();
                        goToNextSubState();
                    }
                }
                break;

            case MOVE_ARM:
                if (robot.moveArm(0, 16)) {
                    goToNextSubState();
                }
                break;

            case RELEASE_SKYSTONE:
                robot.grabberServo.setPosition(GRABBER_SERVO_OPEN_POSITION);
                robot.grabberServo.setPosition(GRABBER_SERVO_TWO_OPEN_POSITION);
                goToNextSubState();
                break;

            case RAISE_ARM:
                if (robot.moveArm(2, 16)) {
                    goToNextSubState();
                }
                break;

            case RETRACT_ARM:
                if (robot.moveArm(2, 14)) {
                    goToNextSubState();
                }
                break;

            case DRIVE_BACK_TO_QUARRY:
                if (robot.drive(1, -36)) {
                    goToSubState(9);
                }
                break;

            //now, if SuperMasterState=FIRST_SKYSTONE, MasterState=STRAFE_TOWARDS_DETECTED_SKYSTONE_V2 and SuperMasterState=SECOND_SKYSTONE
            case REPEAT_CODE:
                subState = REPEAT_CODE;
                if (superMasterState == FIRST_SKYSTONE) {
                    superMasterState++;
                    goToMasterState(STRAFE_TO_SKYSTONE_V2);
                }
                break;
            //if SuperMasterState=SECOND_SKYSTONE, the robot will DRIVE_BACK_TO_PARK and stop
            default:
                isComplete = true;
                subState = END_STATE;
                break;
        }
        return isComplete;
    }
}


/*
    public boolean StrafeTowardsDetectedSkystoneV2(SkystoneVuforiaData vision ) {
        boolean isComplete = false;
        // loop()
        telemetry.addData("Current State", subState);

        switch(subState) {
            case STRAFE_TO_VIEWING_POSITION:
                if(robot.strafe(strafePower, viewingPosition)); {
                robot.light.setPower(1.0);
                goToNextSubState();
            }
            break;

*/
/*
This is Ashley's code. The robot uses Vuforia to locate the coordinates of the skystone. After detecting the skystone, the robot aligns itself, extends its arm,
and picks up the skystone.

*/
/*
            case DETECT_SKYSTONE:
                HashMap<String, Double> skyStoneCoordinates = vision.getSkystoneCoordinates();
                if(skyStoneCoordinates != null){
                    robotXDistanceFromSkystoneCenter = skyStoneCoordinates.get("X");
                    robotYDistanceFromSkystoneCenter = skyStoneCoordinates.get("Y");
                    telemetry.addData("Skystone Pos (in)", "(X, Y) = %.1f, %.1f",
                            robotXDistanceFromSkystoneCenter, robotYDistanceFromSkystoneCenter);
                    robot.light.setPower(0);
                    Log.i("MasterAutonomous", "Robot Y Distance from Skystone: " + (-robotYDistanceFromSkystoneCenter));
                    Log.i("MasterAutonomous", "Robot X Distance from Skystone: " + (-robotXDistanceFromSkystoneCenter));
                    goToNextSubState();
                }
                else {
                    telemetry.addLine("No Skystone Detected");
                }
                telemetry.update();
                break;

            default:
                subState = STATE_END;
                isComplete = true;
                break;

        }
        return isComplete;
    }

    public boolean AlignAndPickUpSkystone(SkystoneVuforiaData vision) {
        boolean isComplete = false;

        telemetry.addData("State", subState);

        switch(subState) {
            case MOVE_ARM_UP:
                if(robot.moveArm(armUpAngle, 0)) {
                    robot.light.setPower(1);
                    goToNextSubState();
                }
                break;

            case FIND_CENTER_OF_SKYSTONE_VS_ARM:
                // The getSkystoneCoordinates() method returns null if the skystone is not detected
                HashMap<String, Double> skyStoneCoordinates = vision.getSkystoneCoordinates();
                if(skyStoneCoordinates != null){
                    robotXDistanceFromSkystoneCenter = skyStoneCoordinates.get("X");
                    robotYDistanceFromSkystoneCenter = skyStoneCoordinates.get("Y");
                    telemetry.addData("Skystone Pos (in)", "(X, Y) = %.1f, %.1f",
                            robotXDistanceFromSkystoneCenter, robotYDistanceFromSkystoneCenter);
                    robot.light.setPower(0);
                    Log.i("MasterAutonomous", "Robot Y Distance from Skystone: " + (-robotYDistanceFromSkystoneCenter));
                    Log.i("MasterAutonomous", "Robot X Distance from Skystone: " + (-robotXDistanceFromSkystoneCenter));
                    goToNextSubState();
                }
                else {
                    telemetry.addLine("No Skystone Detected");
                }
                telemetry.update();
                break;

            case MOVE_ARM_OUT:
                distanceForArmToExtend = -robotXDistanceFromSkystoneCenter + 8;
                if(distanceForArmToExtend > maxArmExtensionDistance) {
                    distanceFromSkystoneOffset = distanceForArmToExtend - maxArmExtensionDistance;
                }

                telemetry.addData("Distance from skystone", distanceForArmToExtend);
                telemetry.update();
                if(robot.moveArm(armUpAngle, distanceForArmToExtend - 10)) {
                    robot.light.setPower(0);
                    goToNextSubState();
                }
                break;

            case MOVE_SERVOS:
                robot.angleServo.setPosition(0.55);
                robot.grabberServo.setPosition(GRABBER_SERVO_OPEN_POSITION);
                robot.grabberServoTwo.setPosition(GRABBER_SERVO_TWO_OPEN_POSITION);
                goToNextSubState();
                break;

            case MOVE_ARM_DOWN:
                if(robot.moveArm(armAngleOnSkystone, distanceForArmToExtend - 5)) {
                    goToNextSubState();
                }
                break;

            case STRAFE_TO_SKYSTONE_2_FIRST:
                if(robot.strafe(0.25, 5)) {
                    goToNextSubState();
                }
                break;

            case ADJUST_ROBOT_POSITION:
                if(-robotYDistanceFromSkystoneCenter > 3) {
                    if (robot.drive(0.75, -robotYDistanceFromSkystoneCenter - 3)) {
                        goToNextSubState();
                    }
                }
                else {
                    if (robot.drive(0.75, -robotYDistanceFromSkystoneCenter)) {
                        goToNextSubState();
                    }
                }
                break;

            case STRAFE_TO_SKYSTONE_2_SECOND:
                if(robot.strafe(0.25, 10)) {
                    goToNextSubState();
                }
                break;

            case FINISH_ARM_EXTENSION:
                if(robot.moveArm(armAngleOnSkystone, distanceForArmToExtend)) {
                    timer.reset();
                    goToNextSubState();
                }
                break;

            case GRAB_SKYSTONE:
                robot.grabberServo.setPosition(GRABBER_SERVO_CLOSE_POSITION);
                robot.grabberServoTwo.setPosition(GRABBER_SERVO_TWO_CLOSE_POSITION);
                if(timer.milliseconds() > 500) {
                    goToNextSubState();
                }
                break;

            case PUT_ARM_DOWN:
                if(robot.moveArm(1, 15)){
                    goToNextSubState();
                }
                break;

            default:
                isComplete = true;
                subState = STATE_END_2;
                telemetry.update();
                break;
        }
        return isComplete;
    }

/*This is Avery's code. Gripping the skystone, the robot moves away from the stones and towards the end wall, where it drags the foundation
to the building zone. The robot uses a gyro sensor to precisely turn to the correct angle from which to drag the skystone.*/
//    public boolean MoveFoundation() {
//        boolean isComplete = false;
//        boolean isBlue = allianceColor == AllianceColor.BLUE;
//        boolean timerReset = false;
//
//        telemetry.addData("Blue Distance: ", robot.blueDistanceSensor.getDistance(INCH));
//        telemetry.addData("Red Distance: ", robot.redDistanceSensor.getDistance(INCH));
//        telemetry.addData("State: ", subState);
//        telemetry.update();
//
//        switch(subState) {
//            case DRIVE_AWAY_FROM_BLOCK:
//    /*robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
//    robot.drivePower(0.5, -0.5, -0.5, 0.5);
//    if (isBlue) {
//        if (robot.blueDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
//            robot.stop();
//            goToNextSubState();
//        }
//    } else {
//        if (robot.redDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
//            robot.stop();
//            goToNextSubState();
//        }
//    }
//     */
//
//                if (robot.strafe(0.50,-10)) {
//                    robot.stop();
//                    goToNextSubState();
//                }
//                break;
//
//            case ADJUST_ANGLE:
//                angle = robot.getTurningAngle();
//                telemetry.addData("Angle", angle);
//                if(angle > -0.4) {
//                    angleAdjustmentSign = -1;
//                }
//                else if(angle < -0.1) {
//                    angleAdjustmentSign = 1;
//                }
//                else {
//                    robot.stop();
//                    Log.i("MasterAutonomous", "Gyro Angle: " + angle + " degrees");
//                    Log.i("MasterAutonomous", "Finished Angle adjustment");
//                    goToNextSubState();
//                    break;
//                }
//
//                robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.leftFrontMotor.setPower(0.05 * angleAdjustmentSign);
//                robot.rightFrontMotor.setPower(0.05 * -angleAdjustmentSign);
//                robot.leftBackMotor.setPower(0.05 * angleAdjustmentSign);
//                robot.rightBackMotor.setPower(0.05 * -angleAdjustmentSign);
//
//                angle = robot.getTurningAngle();
//                if(angle > -0.4 && angle < -0.1) {
//                    telemetry.addData("Angle", angle);
//                    Log.i("MasterAutonomous", "Gyro Angle: " + angle + " degrees");
//                    Log.i("MasterAutonomous", "Finished Angle adjustment");
//                    goToNextSubState();
//                }
//                break;
//
//
//            case DRIVE_TO_WALL_1:
//
//                //robot.setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
//                double distanceToEndOfQuarry = isBlue ? 12 + robotYDistanceFromSkystoneCenter : -robotYDistanceFromSkystoneCenter - 7;
//
//                int distanceToGate = 36;
//
//                if (isBlue) {
//                    if (robot.drive(0.9, distanceToGate)) {
//                        Log.i("MasterAutonomous", "Distance to Drive: " + (distanceToGate) + " INCHES");
//                        robot.stop();
//                        goToNextSubState();
//                    }
//
//                    break;
//
//                    case MOVE_ARM:
//                        if (robot.moveArm(0, 16)) {
//                            goToNextSubState();
//                        }
//                        break;
//
//                    case RELEASE_SKYSTONE:
//                        robot.grabberServo.setPosition(GRABBER_SERVO_OPEN_POSITION);
//                        robot.grabberServo.setPosition(GRABBER_SERVO_TWO_OPEN_POSITION);
//                        goToNextSubState();
//                        break;
//
//                    case RAISE_ARM:
//                        if (robot.moveArm(2, 16)) {
//                            goToNextSubState();
//                        }
//                        break;
//
//                    case RETRACT_ARM:
//                        if (robot.moveArm(2,14)) {
//                            goToSubState(9);
//                        }
//
//                    case DRIVE_BACK_TO_PARK:
//                        if (robot.drive(1, -18)) {
//                            goToNextSubState();
//                        }
//                        break;
//
//                    case STOP:
//                        robot.stop();
//                        goToNextSubState();
//                        break;
//
//                    default:
//                        isComplete = true;
//                        subState = END_STATE;
//                        break;
//
//        return isComplete;
//        */



