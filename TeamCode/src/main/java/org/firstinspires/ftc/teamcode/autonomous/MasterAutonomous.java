package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

import java.util.HashMap;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_TWO_CLOSE_POSITION;
import static org.firstinspires.ftc.teamcode.common.Robot.GRABBER_SERVO_TWO_OPEN_POSITION;

@Autonomous(name = "Master Auto", group = "auto")
public class MasterAutonomous extends LinearOpMode {

    // Global variables go before runOpMode()
    Robot robot = new Robot();

    int masterState = 1;
    int subState = 1;

    AllianceColor allianceColor = AllianceColor.BLUE;

    boolean skystoneDetected;

    // --- Master States --- //

    /* Each autonomous coder programmed a part of the autonomous code separately. In the MasterAutonomous class, all of the member's programs are combined
    and run linearly. Here, all of the states of each member's programs are defined.
     */
    static final int STRAFE_TO_SKYSTONE_V2 = 1;
    static final int ALIGN_AND_PICK_UP_SKYSTONE = 2;
    static final int MOVE_FOUNDATION = 3;
    static final int SONAL_AUTONOMOUS = 4;


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
    double distanceToSkystone = 0;

    static final int MOVE_ARM_UP = 1;
    //    static final int FIND_CENTER_OF_SKYSTONE_VS_ARM = 2;
    static final int MOVE_ARM_OUT = 2;
    static final int MOVE_SERVOS = 3;
    static final int MOVE_ARM_DOWN = 4;
    static final int STRAFE_TO_SKYSTONE_2_FIRST = 5;
    static final int ADJUST_ROBOT_POSITION = 6;
    static final int ADJUST_WITH_RANGE = 7;
    static final int STRAFE_TO_SKYSTONE_2_SECOND = 8;
    static final int FINISH_ARM_EXTENSION = 9;
    static final int GRAB_SKYSTONE = 10;
    static final int PUT_ARM_DOWN = 11;

    static final int STATE_END_2 = 12;


    // --- MovingFoundation States and Variables --- //
    /* This is Avery's code. After picking up the skystone, the robot drives to the foundation and drags it.

     */

    static final int DRIVE_AWAY_FROM_BLOCK = 1;
    static final int ADJUST_ANGLE = 2;
    static final int DRIVE_TO_WALL_1 = 3;
    //    static final int DRIVE_TO_WALL_2 = 4;
    static final int MOVE_ARM = 4;
    static final int DRIVE_TO_WALL_3 = 5;
    static final int DRIVE_TO_FOUNDATION = 6;
    static final int DRIVE_TO_FOUNDATION_2 = 7;
    static final int ROBOT_MOVES_ARM = 8;
    static final int ROBOT_RETRACTS_ARM = 9;
    static final int ROBOT_RELEASES_SKYSTONE = 10;
    static final int ROBOT_RAISES_ARM = 11;
    static final int DRAG_FOUNDATION = 12;
    static final int STATE_END_3 = 13;

    double distance;
    double angleAdjustmentSign = 0;
    double angle;
    boolean hasSetMode = false;

    // --- SonalAutonomous States and Variables --- //
    //variables
    double drivePower2 = 0.5;
    double strafePower2 = -0.75;
//TODO:  Behind foundation position was reduced by 2 to prevent overrunning the parking tape, especially on red side. NOT ANYMORE
    double behindFoundationPosition = 34; // BLUE: 36.75, RED: 34
    double towardsCenterPosition = 31;
    double towardsRedLinePosition = 21; // BLUE: 22, RED 21
    boolean blueAlliance;

    //cases
//    static final int ROBOT_MOVES_ARM = 1;
//    static final int ROBOT_RETRACTS_ARM = 2;
//    static final int ROBOT_RELEASES_SKYSTONE = 3;
//    static final int ROBOT_RAISES_ARM = 4;
    static final int ROBOT_MOVES_BEHIND_FOUNDATION = 1;
    static final int ROBOT_LOWERS_ARM = 2;
    static final int ROBOT_STRAFES_CLOSER_TO_CENTER = 3;
    static final int ROBOT_MOVES_BACKWARDS = 4;
    static final int ROBOT_STOPS = 5;
    static final int END_STATE = 6;


    @Override
    public void runOpMode() throws InterruptedException {
        // init()
        robot.initForRunToPosition(hardwareMap);

        //Here the robot discerns the alliance color based on the switch
        if (robot.allianceSwitch.getState()) {
            allianceColor = AllianceColor.BLUE;
        } else {
            allianceColor = AllianceColor.RED;
        }


        SkystoneVuforiaData vision = new SkystoneVuforiaData(hardwareMap, robot);

        vision.targetsSkyStone.activate();


        waitForStart();

        while (opModeIsActive()) {
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

                case SONAL_AUTONOMOUS:
                    if (SonalAutonomous()) {
                        goToNextMasterState();
                    }
                    break;

                default:
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
        timer.reset();
        subState = 1;
        masterState++;
    }

    public void goToMasterState(int newMasterState) {
        robot.resetChassisEncoders();
        masterState = newMasterState;
        subState = 1;
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
                timer.reset();
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
                    robotYDistanceFromSkystoneCenter = skyStoneCoordinates.get("Y Corrected");
                    telemetry.addData("Skystone Pos (in)", "(X, Y) = %.1f, %.1f",
                            robotXDistanceFromSkystoneCenter, robotYDistanceFromSkystoneCenter);
                    robot.light.setPower(0);
                    Log.i("MasterAutonomous", "Robot Y Distance from Skystone: " + (-robotYDistanceFromSkystoneCenter));
                    Log.i("MasterAutonomous", "Robot X Distance from Skystone: " + (-robotXDistanceFromSkystoneCenter));
                    skystoneDetected = true;
                    goToNextSubState();
                } else {
                    telemetry.addLine("No Skystone Detected");
                    if(timer.seconds() > 5) {
                        skystoneDetected = false;
                        goToMasterState(MOVE_FOUNDATION);
                    }
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
        boolean isBlue = allianceColor == AllianceColor.BLUE;
        telemetry.addData("State", subState);

        switch (subState) {
            case MOVE_ARM_UP:
                if (robot.moveArm(armUpAngle, 0)) {
                    robot.light.setPower(1);
                    goToNextSubState();
                }
                break;

//            case FIND_CENTER_OF_SKYSTONE_VS_ARM:
//                // The getSkystoneCoordinates() method returns null if the skystone is not detected
//                HashMap<String, Double> skyStoneCoordinates = vision.getSkystoneCoordinates();
//                if(skyStoneCoordinates != null){
//                    robotXDistanceFromSkystoneCenter = skyStoneCoordinates.get("X");
//                    robotYDistanceFromSkystoneCenter = skyStoneCoordinates.get("Y Corrected");
//                    telemetry.addData("Skystone Pos (in)", "(X, Y) = %.1f, %.1f",
//                            robotXDistanceFromSkystoneCenter, robotYDistanceFromSkystoneCenter);
//                    robot.light.setPower(0);
//                    Log.i("MasterAutonomous", "Robot Y Distance from Skystone: " + (-robotYDistanceFromSkystoneCenter));
//                    Log.i("MasterAutonomous", "Robot X Distance from Skystone: " + (-robotXDistanceFromSkystoneCenter));
//                    goToNextSubState();
//                }
//                else {
//                    telemetry.addLine("No Skystone Detected");
//                }
//                telemetry.update();
//                break;

            case MOVE_ARM_OUT:
                distanceForArmToExtend = -robotXDistanceFromSkystoneCenter + 8;
                if (distanceForArmToExtend > maxArmExtensionDistance) {
                    distanceFromSkystoneOffset = distanceForArmToExtend - maxArmExtensionDistance;
                }

                telemetry.addData("Distance from skystone", distanceForArmToExtend);
                telemetry.update();
                if (robot.moveArm(armUpAngle, distanceForArmToExtend - 10)) {
                    vision.targetsSkyStone.deactivate();
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
//                if(-robotYDistanceFromSkystoneCenter > 3) {
//                    if (robot.drive(0.75, -robotYDistanceFromSkystoneCenter - 3)) {
//                        goToNextSubState();
//                    }
//                }
//                else {
                if (robot.drive(0.75, -robotYDistanceFromSkystoneCenter)) {
                    goToNextSubState();
                }
//                }
                break;

            case ADJUST_WITH_RANGE:
                double bluePosition1 = 21.4;
                double bluePosition2 = 28.9;
                double bluePosition3 = 36.0;

                double redPosition1 = 21.0;
                double redPosition2 = 28.0;
                double redPosition3 = 36.0;

                if (isBlue) {
                    if (robot.rightDistanceSensorBlue.getDistance(INCH) < 25) {
                        distanceToSkystone = bluePosition1;
                    } else if (robot.rightDistanceSensorBlue.getDistance(INCH) < 32) {
                        distanceToSkystone = bluePosition2;
                    } else {
                        distanceToSkystone = bluePosition3;
                    }

                } else {
                    if (robot.rightDistanceSensorRed.getDistance(INCH) < 24) {
                        distanceToSkystone = redPosition1;
                    } else if (robot.rightDistanceSensorRed.getDistance(INCH) < 32) {
                        distanceToSkystone = redPosition2;
                    } else {
                        distanceToSkystone = redPosition3;
                    }
                }

               if(robot.adjustRangeDistance(isBlue ? robot.rightDistanceSensorBlue :
                       robot.rightDistanceSensorRed, distanceToSkystone, isBlue)) {
                   goToNextSubState();
               }
               break;

            case STRAFE_TO_SKYSTONE_2_SECOND:
                if (robot.strafe(0.5, 5)) {
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
                if (robot.moveArm(2, 15)) {
                    goToNextSubState();
                }
                break;

            default:
                isComplete = true;
                telemetry.addData("Distance to wall", robot.rightDistanceSensorRed.getDistance(INCH));
                subState = STATE_END_2;
                telemetry.update();
                break;
        }
        return isComplete;
    }

    /*This is Avery's code. Gripping the skystone, the robot moves away from the stones and towards the end wall, where it drags the foundation
    to the building zone. The robot uses a gyro sensor to precisely turn to the correct angle from which to drag the skystone.*/
    public boolean MoveFoundation() {
        boolean isComplete = false;
        boolean isBlue = allianceColor == AllianceColor.BLUE;
        boolean timerReset = false;

        //Telemetry sensor reads commented out due to lagging loop times
       // telemetry.addData("Blue Distance: ", robot.blueDistanceSensor.getDistance(INCH));
        //telemetry.addData("Red Distance: ", robot.redDistanceSensor.getDistance(INCH));
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

//                if (robot.strafe(0.75,-1)) {
//                    robot.stop();
                if(!skystoneDetected) {
                    if(robot.strafe(0.5, 10)) {
                        goToNextSubState();
                    }
                }
                else {
                    goToNextSubState();
                }
//                }
                break;

            case ADJUST_ANGLE:
                if (!hasSetMode) {
                    robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
                    hasSetMode = true;
                }
                angle = robot.getTurningAngle();
                telemetry.addData("Angle", angle);
                if (angle > 0.3) { //too far clockwise
                    angleAdjustmentSign = -1; //counterclockwise
                } else if (angle < -0.3) { //too far counterclockwise
                    angleAdjustmentSign = 1; //clockwise
                } else {
                    robot.stop();
                    Log.i("MasterAutonomous", "Gyro Angle: " + angle + " degrees");
                    Log.i("MasterAutonomous", "Finished Angle adjustment");
                    goToNextSubState();
                    break;
                }

                robot.leftFrontMotor.setPower(0.1 * angleAdjustmentSign);
                robot.rightFrontMotor.setPower(0.1 * -angleAdjustmentSign);
                robot.leftBackMotor.setPower(0.1 * angleAdjustmentSign);
                robot.rightBackMotor.setPower(0.1 * -angleAdjustmentSign);

//                angle = robot.getTurningAngle();
//                if(angle > -1.0 && angle < -0.1) {
//                    telemetry.addData("Angle", angle);
//                    Log.i("MasterAutonomous", "Gyro Angle: " + angle + " degrees");
//                    Log.i("MasterAutonomous", "Finished Angle adjustment");
//                    goToNextSubState();
//                }
                break;


            case DRIVE_TO_WALL_1:

                //robot.setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
       //TODO:  These offsets were changed from zero because we occasionally missed going far enough on blue, and we sometimes went too far on red.
                double blueQuarryOffset = 2;
                double redQuarryOffset = -4;
                double distanceToEndOfQuarry = isBlue ? blueQuarryOffset + robotYDistanceFromSkystoneCenter : redQuarryOffset - robotYDistanceFromSkystoneCenter;

                int distanceToFoundationEdge = 52;
                int distanceToFoundationCenter = 20;

                double distanceToFoundation = distanceToEndOfQuarry + distanceToFoundationEdge + distanceToFoundationCenter;
                if (isBlue) {
                    if (robot.drive(0.9, distanceToFoundation)) {
                        Log.i("MasterAutonomous", "Distance to Drive: " + (distanceToFoundation) + " INCHES");
                        robot.stop();
                        goToNextSubState();
                    }
                } else {
                    if (robot.drive(0.9, -distanceToFoundation)) {
                        Log.i("MasterAutonomous", "Distance to Drive: " + (-distanceToFoundation) + " INCHES");
                        robot.stop();
                        goToNextSubState();
                    }
                }
                break;

//            case DRIVE_TO_WALL_2:
//                robot.angleMotor.setTargetPosition((int)(robot.ARM_ANGLE_MOTOR_TICKS_PER_ROTATION * (25 + robot.ARM_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG) / 360));
//                robot.angleMotor.setPower(1.0);
//                robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
//                if (isBlue) {
//                    robot.drivePower(0.5,0.5,0.5,0.5);
//                    if (robot.blueDistanceSensor.getDistance(INCH) < 13 && robot.redDistanceSensor.getDistance(INCH) < 13) {
//                        Log.i("MasterAutonomous", "Blue Distance Sensor Reading: " + robot.blueDistanceSensor.getDistance(INCH) + " INCHES");
//                        robot.stop();
//                        goToNextSubState();
//                    }
//                } else {
//                    robot.drivePower(-0.5,-0.5,-0.5,-0.5);
//                    if (robot.redDistanceSensor.getDistance(INCH) < 13 && robot.blueDistanceSensor.getDistance(INCH) < 13) {
//                        Log.i("MasterAutonomous", "Red Distance Sensor Reading: " + robot.redDistanceSensor.getDistance(INCH) + " INCHES");
//                        robot.stop();
//                        goToNextSubState();
//                    }
//                }
//                break;

            case MOVE_ARM:
                if(skystoneDetected) {
                    robot.angleMotor.setTargetPosition((int) (robot.ARM_ANGLE_MOTOR_TICKS_PER_ROTATION * (25 + robot.ARM_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG) / 360));
                    robot.angleMotor.setPower(1.0);
                }
                goToNextSubState();
                break;

            case DRIVE_TO_WALL_3:
                robot.setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
                if (isBlue) {
                    if (robot.drive(0.5, 13)) {
                        robot.stop();
                        timer.reset();
//                        distance = robot.blueDistanceSensor.getDistance(DistanceUnit.INCH) + 2;
                        goToNextSubState();
                    }
                } else {
                    if (robot.drive(0.5, -13)) {
//                        distance = robot.redDistanceSensor.getDistance(DistanceUnit.INCH) + 2;
                        robot.stop();
                        timer.reset();
                        goToNextSubState();
                    }
                }
                break;

            case DRIVE_TO_FOUNDATION:
                robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);

                double blueDistance = robot.blueDistanceSensor.getDistance(INCH);
                //deceleration loop
                double bluePower = Math.max(0.2,Math.min(0.7,(blueDistance-2)*0.1));
 //TODO:  Distance for reaching foundation dropped from 2.7 to 2.5 to ensure success.
                if(blueDistance < 2.5) {
                    bluePower = 0;
                }

                robot.leftBackMotor.setPower(-bluePower);
                robot.rightBackMotor.setPower(bluePower);
                Log.i("MasterAutonomous", "Blue Distance Sensor Reading: " + blueDistance + " INCHES");

                double redDistance = robot.redDistanceSensor.getDistance(INCH);
                //deceleration loop
                double redPower = Math.max(0.2,Math.min(0.7,(redDistance-2)*0.1));
                if (redDistance < 2.5) {
                    redPower = 0;
                }

                robot.leftFrontMotor.setPower(redPower);
                robot.rightFrontMotor.setPower(-redPower);
                Log.i("MasterAutonomous", "Red Distance Sensor Reading: " +  redDistance + " INCHES");

                if (redPower == 0.0 && bluePower == 0.0 || timer.seconds() >=3) {
                    goToNextSubState();
                }

                break;

            case DRIVE_TO_FOUNDATION_2:
                robot.stop();
                robot.resetChassisEncoders();
                robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_DOWN_POSITION);
                timer.reset();
                goToNextSubState();
                break;

            case ROBOT_MOVES_ARM:
//                if (robot.moveArm(6, 16)) {
                goToNextSubState();
//                }
                break;

            case ROBOT_RETRACTS_ARM:
                if(skystoneDetected){
                    if (robot.moveArm(6, 14)) {
                        goToNextSubState();
                    }
                }
                else {
                    goToNextSubState();
                }
                break;

            case ROBOT_RELEASES_SKYSTONE:
                robot.grabberServo.setPosition(GRABBER_SERVO_OPEN_POSITION);
                robot.grabberServoTwo.setPosition(GRABBER_SERVO_TWO_OPEN_POSITION);
                goToNextSubState();
                break;

            case ROBOT_RAISES_ARM:
                if(skystoneDetected) {
                    if (robot.moveArm(6, 12)) {
                        goToNextSubState();
                    }
                }
                else {
                    goToNextSubState();
                }
                break;


            case DRAG_FOUNDATION:

                //wait for foundation movers to go down
                if(timer.milliseconds() < 500){
                    break;
                }
                angle = robot.getTurningAngle();
                double angleOffset = 5 * Math.signum(angle);
                Log.i("MasterAutonomous", "Robot Angle: " + angle + " degrees");
                double distance = robot.backDistanceSensor.getDistance(MM);
                telemetry.addData("Distance from wall", distance);
                if (isBlue) {
                    robot.driveMecanumContinuous(-0.2, 0.8, (0.1*angle));
                    if (distance < 65) {
                        robot.stop();
                        robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_UP_POSITION);
                        goToNextSubState();
                    }
//                    }
                } else {
                    robot.driveMecanumContinuous(0.2, 0.8, (0.1*angle));
                    if (distance < 65) {
                        robot.stop();
                        robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_UP_POSITION);
                        goToNextSubState();
                    }
//                    }
                }
                break;

            default:
                isComplete = true;
                subState = STATE_END_3;
                break;

        }
        return isComplete;
    }

    /* This is Sonal's code. The robot releases the skystone onto the foundation and then moves backwards to park on the tape under the bridge.
     */
    public boolean SonalAutonomous() {
        boolean isComplete = false;
        telemetry.addData("Current State", subState);
        telemetry.update();
        blueAlliance = allianceColor == AllianceColor.BLUE;
        switch (subState) {

            case ROBOT_MOVES_BEHIND_FOUNDATION:
                if (timer.milliseconds() > 500) {
                    if (blueAlliance) {
                        if (robot.drive(drivePower2, -behindFoundationPosition)) {
                            goToNextSubState();
                        }
                    } else {
                        if (robot.drive(drivePower2, behindFoundationPosition)) {
                            goToNextSubState();
                        }
                    }
                }
                break;

            case ROBOT_LOWERS_ARM:
                if(skystoneDetected) {
                    if (robot.moveArm(-10, 12)) {
                        goToNextSubState();
                    }
                }
                else {
                    goToNextSubState();
                }
                break;

            case ROBOT_STRAFES_CLOSER_TO_CENTER:
                if (blueAlliance) {
                    if (robot.strafe(strafePower2, towardsCenterPosition)) {
                        robot.grabberServo.setPosition(GRABBER_SERVO_CLOSE_POSITION);
                        robot.grabberServoTwo.setPosition(GRABBER_SERVO_TWO_CLOSE_POSITION);
                        goToNextSubState();
                    }
                } else if (!(blueAlliance)) {
                    if (robot.strafe(-strafePower2, towardsCenterPosition)) {
                        robot.grabberServo.setPosition(GRABBER_SERVO_CLOSE_POSITION);
                        robot.grabberServoTwo.setPosition(GRABBER_SERVO_TWO_CLOSE_POSITION);
                        goToNextSubState();
                    }
                }
                break;


            case ROBOT_MOVES_BACKWARDS:
                if (blueAlliance) {
                    if (robot.drive(drivePower2, -towardsRedLinePosition)) {
                        goToNextSubState();
                    }
                } else {
                    if (robot.drive(drivePower2, towardsRedLinePosition)) {
                        goToNextSubState();
                    }
                }
                break;


            case ROBOT_STOPS:
                robot.stop();
                goToNextSubState();
                break;


            default:
                isComplete = true;
                subState = END_STATE;
                break;
        }
        return isComplete;
    }
}