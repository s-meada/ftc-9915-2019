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
public class MasterAutonomous extends LinearOpMode {

    // Global variables go before runOpMode()
    Robot robot = new Robot();

    int masterState = 1;
    int subState = 1;

    AllianceColor allianceColor = AllianceColor.BLUE;

    // --- Master States --- //
    static final int STRAFE_TO_SKYSTONE_V2      = 1;
    static final int ALIGN_AND_PICK_UP_SKYSTONE = 2;
    static final int MOVE_FOUNDATION            = 3;
    static final int SONAL_AUTONOMOUS           = 4;


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

    static final int MOVE_ARM_UP                    = 1;
    static final int FIND_CENTER_OF_SKYSTONE_VS_ARM = 2;
    static final int MOVE_ARM_OUT                   = 3;
    static final int MOVE_SERVOS                    = 4;
    static final int MOVE_ARM_DOWN                  = 5;
    static final int STRAFE_TO_SKYSTONE_2_FIRST     = 6;
    static final int ADJUST_ROBOT_POSITION          = 7;
    static final int STRAFE_TO_SKYSTONE_2_SECOND    = 8;
    static final int FINISH_ARM_EXTENSION           = 9;
    static final int GRAB_SKYSTONE                  = 10;
    static final int PUT_ARM_DOWN                   = 11;

    static final int STATE_END_2                    = 12;


    // --- MovingFoundation States and Variables --- //

    static final int DRIVE_AWAY_FROM_BLOCK = 1;
    static final int ADJUST_ANGLE = 2;
    static final int DRIVE_TO_WALL_1 = 3;
    static final int DRIVE_TO_WALL_2 = 4;
    static final int DRIVE_TO_WALL_3 = 5;
    static final int DRIVE_TO_FOUNDATION = 6;
    static final int DRIVE_TO_FOUNDATION_2 = 7;
    static final int DRAG_FOUNDATION = 8;
    static final int STATE_END_3 = 9;

    double distance;
    double angleAdjustmentSign = 0;
    double angle;

    // --- SonalAutonomous States and Variables --- //
    //variables
    double drivePower2 = 0.5;
    double strafePower2 = -0.75;
    double behindFoundationPosition = 35;
    double towardsCenterPosition = 31;
    double towardsRedLinePosition = 25;
    boolean blueAlliance = allianceColor == AllianceColor.BLUE;

    //cases
    static final int ROBOT_MOVES_ARM = 1;
    static final int ROBOT_RELEASES_SKYSTONE = 2;
    static final int ROBOT_RAISES_ARM = 3;
    static final int ROBOT_RETRACTS_ARM = 4;
    static final int ROBOT_MOVES_BEHIND_FOUNDATION = 5;
    static final int ROBOT_LOWERS_ARM = 6;
    static final int ROBOT_STRAFES_CLOSER_TO_CENTER = 7;
    static final int ROBOT_MOVES_BACKWARDS = 8;
    static final int ROBOT_STOPS = 9;
    static final int END_STATE = 10;


    @Override
    public void runOpMode() throws InterruptedException {
        // init()
        robot.initForRunToPosition(hardwareMap);
        SkystoneVuforiaData vision = new SkystoneVuforiaData(hardwareMap,robot);

        waitForStart(); // MUST add this yourself

        vision.targetsSkyStone.activate();

        while(opModeIsActive()) {  // MUST add this yourself
           telemetry.addData("Master State", masterState);
           telemetry.update();
            switch (masterState) {
               case STRAFE_TO_SKYSTONE_V2:
                   if(StrafeTowardsDetectedSkystoneV2(vision)) {
                       // Necessary to make driving in the next state work after strafing
                       goToNextMasterState();
                   }
                   break;

               case ALIGN_AND_PICK_UP_SKYSTONE:
                   if(AlignAndPickUpSkystone(vision)) {
                       goToNextMasterState();
                   }
                   break;

               case MOVE_FOUNDATION:
                    if(MoveFoundation()) {
                        goToNextMasterState();
                    }
                    break;

               case SONAL_AUTONOMOUS:
                   if(SonalAutonomous()) {
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
    public void goToNextSubState(){
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

            case DETECT_SKYSTONE:
                HashMap <String, Float> skyStoneCoordinates = vision.getSkystoneCoordinates();
                if(skyStoneCoordinates != null){
                    skyStoneCoordinateX = skyStoneCoordinates.get("X");
                    skyStoneCoordinateY = skyStoneCoordinates.get("Y");
                    telemetry.addData("Skystone Coordinates", "(" + skyStoneCoordinateX + "," + skyStoneCoordinateY+")");
                    goToNextSubState();
                }

                else{
                    telemetry.addData("Skystone Coordinates", "(" + skyStoneCoordinateX + "," + skyStoneCoordinateY + ")");
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
                HashMap<String, Float> skyStoneCoordinates = vision.getSkystoneCoordinates();
                if(skyStoneCoordinates != null){
                    robotXDistanceFromSkystoneCenter = skyStoneCoordinates.get("X");
                    robotYDistanceFromSkystoneCenter = skyStoneCoordinates.get("Y");
                    telemetry.addData("Skystone Pos (in)", "(X, Y) = %.1f, %.1f",
                            robotXDistanceFromSkystoneCenter, robotYDistanceFromSkystoneCenter);
                    robot.light.setPower(0);
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
                if(robot.moveArm(armUpAngle, distanceForArmToExtend - 5)) {
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
                if(robot.drive(0.75, -robotYDistanceFromSkystoneCenter)) {
                    goToNextSubState();
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
                if(timer.milliseconds() > 700) {
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

    public boolean MoveFoundation() {
        boolean isComplete = false;
        boolean isBlue = allianceColor == AllianceColor.BLUE;
        boolean timerReset = false;

        telemetry.addData("Blue Distance: ", robot.blueDistanceSensor.getDistance(INCH));
        telemetry.addData("Red Distance: ", robot.redDistanceSensor.getDistance(INCH));
        telemetry.addData("State: ", subState);
        telemetry.update();

        switch(subState) {
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

                if (robot.strafe(0.50,-10)) {
                    robot.stop();
                    goToNextSubState();
                }
                break;

            case ADJUST_ANGLE:
                angle = robot.getTurningAngle();
                telemetry.addData("Angle", angle);
                Log.i("MasterAutonomous", "Gyro Angle: " + angle + " degrees");
                if(angle > -0.3) {
                    angleAdjustmentSign = -1;
                }
                else if(angle < -0.05) {
                    angleAdjustmentSign = 1;
                }
                else {
                    robot.stop();
                    Log.i("MasterAutonomous", "Finished Angle adjustment");
                    goToNextSubState();
                    break;
                }

                robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftFrontMotor.setPower(0.25 * angleAdjustmentSign);
                robot.rightFrontMotor.setPower(0.25 * -angleAdjustmentSign);
                robot.leftBackMotor.setPower(0.25 * angleAdjustmentSign);
                robot.rightBackMotor.setPower(0.25 * -angleAdjustmentSign);

                angle = robot.getTurningAngle();
                if(angle > -0.25 && angle < 0.01) {
                    telemetry.addData("Angle", angle);
                    Log.i("MasterAutonomous", "Gyro Angle: " + angle + " degrees");
                    Log.i("MasterAutonomous", "Finished Angle adjustment");
                    goToNextSubState();
                }
                break;


            case DRIVE_TO_WALL_1:

                //robot.setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
                double distanceToEndOfQuarry = 12 + robotYDistanceFromSkystoneCenter;
                int distanceToFoundationEdge = 55;
                int distanceToFoundationCenter = 17;

                double distanceToFoundation = distanceToEndOfQuarry + distanceToFoundationEdge + distanceToFoundationCenter;
                if (isBlue) {
                    Log.i("MasterAutonomous", "Distance to Drive: " + (distanceToFoundation) + " INCHES");
                    if (robot.drive(0.5, distanceToFoundation)) {
                        robot.stop();
                        goToNextSubState();
                    }
                } else {
                    Log.i("MasterAutonomous", "Distance to Drive: " + (-distanceToFoundation) + " INCHES");
                    if (robot.drive(0.5, -distanceToFoundation)) {
                        robot.stop();
                        goToNextSubState();
                    }
                }
                break;

            case DRIVE_TO_WALL_2:
                robot.angleMotor.setTargetPosition((int)(robot.ARM_ANGLE_MOTOR_TICKS_PER_ROTATION * (25 + robot.ARM_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG) / 360));
                robot.angleMotor.setPower(1.0);
                robot.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
                if (isBlue) {
                    Log.i("MasterAutonomous", "Blue Distance Sensor Reading: " + robot.blueDistanceSensor.getDistance(INCH) + " INCHES");
                    robot.drivePower(0.5,0.5,0.5,0.5);
                    if (robot.blueDistanceSensor.getDistance(INCH) < 13 && robot.redDistanceSensor.getDistance(INCH) < 13) {
                        robot.stop();
                        goToNextSubState();
                    }
                } else {
                    Log.i("MasterAutonomous", "Red Distance Sensor Reading: " + robot.redDistanceSensor.getDistance(INCH) + " INCHES");
                    robot.drivePower(-0.5,-0.5,-0.5,-0.5);
                    if (robot.redDistanceSensor.getDistance(INCH) < 13 && robot.blueDistanceSensor.getDistance(INCH) < 13) {
                        robot.stop();
                        goToNextSubState();
                    }
                }
                break;

            case DRIVE_TO_WALL_3:
                robot.setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
                if (isBlue) {
                    if (robot.drive(0.5,5)) {
                        robot.stop();
                        distance = robot.blueDistanceSensor.getDistance(DistanceUnit.INCH) + 2;
                        goToNextSubState();
                    }
                } else {
                    if (robot.drive(0.5, -5)) {
                        distance = robot.redDistanceSensor.getDistance(DistanceUnit.INCH) + 2;
                        robot.stop();
                        goToNextSubState();
                    }
                }
                break;

            case DRIVE_TO_FOUNDATION:
                if (robot.strafe(0.75,distance)) {
                    robot.stop();
                    goToNextSubState();
                }
                break;

            case DRIVE_TO_FOUNDATION_2:
                if (robot.strafe(0.25, 3)) {
                    robot.stop();
                    timer.reset();
                    robot.resetChassisEncoders();
                    goToNextSubState();
                }
                break;

            case DRAG_FOUNDATION:
                angle = robot.getTurningAngle();
                double angleOffset = 5 * Math.signum(angle);
                robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_DOWN_POSITION);
                if (timer.seconds() >= 1) {
                    if (robot.driveMecanum(0,1, -Math.toRadians(angle + angleOffset), -100)) {
                        robot.stop();
                        robot.foundationServo.setPosition(robot.FOUNDATION_SERVO_UP_POSITION);
                        goToNextSubState();
                    }
                }
                break;

            default:
                isComplete = true;
                subState = STATE_END_3;
                break;

        }
        return isComplete;
    }

    public boolean SonalAutonomous() {
        boolean isComplete = false;
        telemetry.addData("Current State", subState);
        telemetry.update();
        switch (subState) {

            case ROBOT_MOVES_ARM:
                if (robot.moveArm(0, 16)) {
                    goToNextSubState();
                }
                break;

            case ROBOT_RELEASES_SKYSTONE:
                robot.grabberServo.setPosition(GRABBER_SERVO_OPEN_POSITION);
                robot.grabberServoTwo.setPosition(GRABBER_SERVO_TWO_OPEN_POSITION);
                goToNextSubState();
                break;

            case ROBOT_RAISES_ARM:
                if (robot.moveArm(2, 16)) {
                    goToNextSubState();
                }
                break;


            case ROBOT_RETRACTS_ARM:
                if (robot.moveArm(2, 14)) {
                    goToNextSubState();
                }
                break;

            case ROBOT_MOVES_BEHIND_FOUNDATION:
                if (robot.drive(drivePower2, -behindFoundationPosition)) {
                    goToNextSubState();
                }
                break;

            case ROBOT_LOWERS_ARM:
                if (robot.moveArm(-10,12)) {
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
                }
                else if (!(blueAlliance)) {
                    if (robot.strafe(-strafePower2, towardsCenterPosition)) {
                        goToNextSubState();
                    }
                }

                break;


            case ROBOT_MOVES_BACKWARDS:
                if (robot.drive(drivePower2, -towardsRedLinePosition)) {
                    goToNextSubState();
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
