package org.firstinspires.ftc.teamcode.common;
import android.util.Log;

import java.util.ArrayList;
import java.util.Collections;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

public class Robot {
    /* This is a class defining all of the constants, limits, initial positions, methods, motors, servos, switches, and sensors that are used in
    autonomous and teleop.
    */
    public boolean encodersReseted = false;

    // --- Robot Geometry --- //
    // Wheels
    double wheelDiameter = 4;
    double wheelInchesPerRotation = Math.PI * wheelDiameter;
    int motorTicksPerRotation = 1120;
    double gearRatioMotorToWheel = 24.0/16.0;
    // double type for higher accuracy when multiplying by distanceInch in driveForward() method
    double robotTicksPerInch = motorTicksPerRotation / (gearRatioMotorToWheel * wheelInchesPerRotation);

    // Arm - units: inches
    public static final double Y_DISTANCE_FROM_CAMERA_TO_ARM = 3.0;
    public static final double ARM_STARTING_LENGTH_FROM_EDGE_OF_ROBOT = 4.0;
    public static final double ARM_STARTING_LENGTH = 13.25;
    public static final double ARM_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG = 1.0;
    public static final double ARM_ANGLE_MOTOR_TICKS_PER_ROTATION = 7168.0;
    public static final double EXTENSION_MOTOR_TICKS_PER_ROTATION = 537.6;
    public static final double EXTENSION_SPROCKETS_INCHES_PER_ROTATION = 4;
    public static final double EXTENSION_MOTOR_ANGLE_FACTOR = EXTENSION_MOTOR_TICKS_PER_ROTATION / ARM_ANGLE_MOTOR_TICKS_PER_ROTATION;

    // --- Vuforia --- //
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Constant for Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;


    // --- Constants --- //

    public static final int MOTOR_ENCODER_TOLERANCE = 50;

    // Arm
    public static final int ANGLE_MOTOR_UP_LIMIT = 1400;
    public static final int ANGLE_MOTOR_DOWN_LIMIT = 0;

    public static final int EXTENSION_MOTOR_RETRACTED_LIMIT = 0;
    public static final int EXTENSION_MOTOR_EXTENDED_LIMIT = 1630;

    public static final double GRABBER_SERVO_OPEN_POSITION = 0.5;
    public static final double GRABBER_SERVO_CLOSE_POSITION = 1.0;
    public static final double GRABBER_SERVO_TWO_OPEN_POSITION = 0.75;
    public static final double GRABBER_SERVO_TWO_CLOSE_POSITION = 0.25;

    public static final double ROTATION_SERVO_START_POSITION = 0.47;

    public static final double ANGLE_SERVO_INIT_POSITION = 0.92;

    public static final double CAPSTONE_CLAW_INIT_POSITION = 0.85;
    public static final double CAPSTONE_ANGLE_INIT = 0.35;

    // Foundation
    public static final double FOUNDATION_SERVO_UP_POSITION = 0.82;
    public static final double FOUNDATION_SERVO_DOWN_POSITION = 0.15;


    // --- Robot Hardware Variables --- //

    // Chassis motors
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;

    // Arm motors
    public DcMotorEx angleMotor;
    public DcMotorEx extensionMotor;

    // Servos
    public Servo rotationServo;
    public Servo grabberServo;
    public Servo grabberServoTwo;
    public Servo foundationServo;
    public Servo angleServo;
    public Servo capstoneServoClaw;
    public Servo capstoneServo;

    // Sensors
    public WebcamName webcam;

    BNO055IMU gyroSensor;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation angles;

    public DistanceSensor blueDistanceSensor;
    public DistanceSensor redDistanceSensor;
    public DistanceSensor backDistanceSensor;
    public DistanceSensor rightDistanceSensorBlue;
    public DistanceSensor rightDistanceSensorRed;

    public DigitalChannel allianceSwitch;
    public DigitalChannel programSwitchOne;
    public DigitalChannel programSwitchTwo;

    public AnalogInput potentiometerOne;

    // LED Light Strip
    public DcMotor light;

    // --- Robot init() methods --- //
    // The init() methods here have a hardwareMap parameter. When using them, just type "hardwareMap" as the argument.
    // This allows the method to access your robot's configuration file to init the robot

    /*
     * Robot init() method for driving to a position. Use if you want to use the drive(), strafe(), or driveMecanum() methods
     */
    public void initForRunToPosition(HardwareMap hardwareMap) {
        this.init(hardwareMap);
        setModeChassisMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Robot init() method; use if you want to have the chassis motors run without a specified target position
     */
    public void initRegular(HardwareMap hardwareMap) {
        this.init(hardwareMap);
        setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * This init() method is called by the other init() methods. Do not call this method outside of this class.
     */
    private void init(HardwareMap hardwareMap) {
        // Chassis
        this.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LeftFront");
        this.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RightFront");
        this.leftBackMotor = hardwareMap.get(DcMotorEx.class, "LeftBack");
        this.rightBackMotor = hardwareMap.get(DcMotorEx.class, "RightBack");

        this.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.leftFrontMotor.setTargetPositionTolerance(MOTOR_ENCODER_TOLERANCE);
        this.rightFrontMotor.setTargetPositionTolerance(MOTOR_ENCODER_TOLERANCE);
        this.leftBackMotor.setTargetPositionTolerance(MOTOR_ENCODER_TOLERANCE);
        this.leftBackMotor.setTargetPositionTolerance(MOTOR_ENCODER_TOLERANCE);


        // Arm

        angleMotor = hardwareMap.get(DcMotorEx.class, "angleMotor");
        angleMotor.setTargetPosition(0);
        angleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        angleMotor.setTargetPositionTolerance(MOTOR_ENCODER_TOLERANCE);

        extensionMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");
        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extensionMotor.setTargetPositionTolerance(MOTOR_ENCODER_TOLERANCE);

        rotationServo = hardwareMap.servo.get("rotationServo");
        grabberServo = hardwareMap.servo.get("grabberServo");
        grabberServoTwo = hardwareMap.servo.get("grabberServoTwo");
        foundationServo = hardwareMap.servo.get("foundationServo");
        angleServo = hardwareMap.servo.get("verticalServo");
        capstoneServoClaw = hardwareMap.servo.get("capstoneServoClaw");
        capstoneServo = hardwareMap.servo.get("capstoneServo");

        rotationServo.setPosition(ROTATION_SERVO_START_POSITION);
        grabberServo.setPosition(GRABBER_SERVO_CLOSE_POSITION);
        grabberServoTwo.setPosition(GRABBER_SERVO_TWO_CLOSE_POSITION);
        foundationServo.setPosition(FOUNDATION_SERVO_UP_POSITION);
        angleServo.setPosition(ANGLE_SERVO_INIT_POSITION); // REPLACE with initial position of this servo
        capstoneServoClaw.setPosition(CAPSTONE_CLAW_INIT_POSITION);
        capstoneServo.setPosition(CAPSTONE_ANGLE_INIT);

        // Sensors
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        blueDistanceSensor = hardwareMap.get(DistanceSensor.class, "blueDistanceSensor");
        redDistanceSensor = hardwareMap.get(DistanceSensor.class, "redDistanceSensor");
        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDistanceSensor");
        rightDistanceSensorBlue = hardwareMap.get(DistanceSensor.class, "rightDistanceSensorBlue");
        rightDistanceSensorRed = hardwareMap.get(DistanceSensor.class, "rightDistanceSensorRed");
        gyroSensor = hardwareMap.get(BNO055IMU.class, "gyroSensor");
        gyroSensor.initialize(parameters);
        this.allianceSwitch = hardwareMap.digitalChannel.get("allianceSwitch");
        this.allianceSwitch.setMode(DigitalChannel.Mode.INPUT);
        this.programSwitchOne = hardwareMap.digitalChannel.get("programSwitchOne");
        this.programSwitchOne.setMode(DigitalChannel.Mode.INPUT);
        this.programSwitchTwo = hardwareMap.digitalChannel.get("programSwitchTwo");
        this.programSwitchTwo.setMode(DigitalChannel.Mode.INPUT);
        this.potentiometerOne = hardwareMap.analogInput.get("potentiometerOne");

        // Light
        light = hardwareMap.dcMotor.get("light");

        // Motor PIDF Coefficients
        leftFrontMotor.setVelocityPIDFCoefficients(6., .5, 0., 10.9);
        rightFrontMotor.setVelocityPIDFCoefficients(6., .5, 0., 10.9);
        leftBackMotor.setVelocityPIDFCoefficients(6., .5, 0., 10.9);
        rightBackMotor.setVelocityPIDFCoefficients(6., .5, 0., 10.9);
        angleMotor.setVelocityPIDFCoefficients(6., .5, 0., 10.9);
        extensionMotor.setVelocityPIDFCoefficients(6., .5, 0., 10.9);
    }


    // --- Translation methods --- //

    /*
     * Drives forward if distanceInch is positive; drives backward if distanceInch is negative
     * @param power: the power set to the motors, must be positive
     * @param distanceInch: distance for robot to move in inches
     * @return whether the robot has reached that distance
     */
    public boolean drive(double power, double distanceInch) {
        if(!encodersReseted) {
            this.resetChassisEncoders();
            encodersReseted = true;
        }
        // Getting the sign of the argument to determine which direction we're driving
        int direction = (int)Math.signum(distanceInch);

        this.leftFrontMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));
        this.rightFrontMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));
        this.leftBackMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));
        this.rightBackMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));

        this.leftFrontMotor.setPower(direction * power);
        this.rightFrontMotor.setPower(direction * power);
        this.leftBackMotor.setPower(direction * power);
        this.rightBackMotor.setPower(direction * power);

        setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);
        if (!this.leftFrontMotor.isBusy() || !this.leftBackMotor.isBusy() || !this.rightFrontMotor.isBusy() || !this.rightBackMotor.isBusy()) {
            encodersReseted = false;
            return true;
        } else {
            return false;
        }
    }

    /*
     * Strafes right if distanceInch is positive; strafes left if distanceInch is negative
     * @param power: the power set to the motors, must be positive (signs are determined within the method)
     * @param distanceInch: distance for robot to strafe in inches
     * @return whether the robot has reached that distance
     */
    public boolean strafe(double power, double distanceInch) {
        if(!encodersReseted) {
            this.resetChassisEncoders();
            encodersReseted = true;
        }
        // Getting the sign of the argument to determine which direction we're strafing
        int direction = (int)Math.signum(distanceInch);

        this.leftFrontMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));
        this.rightFrontMotor.setTargetPosition(-(int)(distanceInch * robotTicksPerInch));
        this.leftBackMotor.setTargetPosition(-(int)(distanceInch * robotTicksPerInch));
        this.rightBackMotor.setTargetPosition((int)(distanceInch * robotTicksPerInch));

        this.leftFrontMotor.setPower(direction * power);
        this.rightFrontMotor.setPower(-direction * power);
        this.leftBackMotor.setPower(-direction * power);
        this.rightBackMotor.setPower(direction * power);

        setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);

        if (!this.leftFrontMotor.isBusy() || !this.leftBackMotor.isBusy() || !this.rightFrontMotor.isBusy() || !this.rightBackMotor.isBusy()) {
            encodersReseted = false;
            return true;
        } else {
            return false;
        }
    }

    /*
     * Mecanum drive method that takes x, y, w (rotation), and distance for
     * moving to another position on the field with respect to the robot's current position
     * @param y: amount of movement forward/backward
     * @param x: amount of movement right/left
     * @param w: (turning) direction in radians
     * @param distance: distance for robot to move in inches
     * @return whether the robot has reached that distance
     */
    public boolean driveMecanum(double y, double x, double w, double distance) {
        if(!encodersReseted) {
            this.resetChassisEncoders();
            encodersReseted = true;
        }
        // y and x are negated to make the robot move in the right direction according to signs of the argument values
        y = -y;
        x = -x;

        double lf = y+x-w;
        double rf = y-x+w;
        double lb = y-x-w;
        double rb = y+x+w;

        this.leftFrontMotor.setPower(lf);
        this.rightFrontMotor.setPower(rf);
        this.leftBackMotor.setPower(lb);
        this.rightBackMotor.setPower(rb);

        int targetPosition = -(int)(robotTicksPerInch * distance);

        this.leftFrontMotor.setTargetPosition((int)Math.signum(lf) * targetPosition);
        this.rightFrontMotor.setTargetPosition((int)Math.signum(rf) * targetPosition);
        this.leftBackMotor.setTargetPosition((int)Math.signum(lb) * targetPosition);
        this.rightBackMotor.setTargetPosition((int)Math.signum(rb) * targetPosition);

        setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);

        if (!this.leftFrontMotor.isBusy() || !this.rightFrontMotor.isBusy() || !this.leftBackMotor.isBusy() || !this.rightBackMotor.isBusy()) {
            encodersReseted = false;
            return true;
        } else {
            return false;
        }
    }

    /*
     * Mecanum drive method that takes x, y, w (rotation) and drives continuously
     * @param y: amount of movement forward/backward
     * @param x: amount of movement right/left
     * @param w: (turning) direction in radians
     */
    public void driveMecanumContinuous(double y, double x, double w) {
        // y and x are negated to make the robot move in the right direction according to signs of the argument values
        y = -y;
        x = -x;


        double lf = y+x-w;
        double rf = y-x+w;
        double lb = y-x-w;
        double rb = y+x+w;

        double [] power = {lf,rf,lb,rb};
        ArrayList<Double> aboveThreshold = new ArrayList<>();

        for(double motorPower: power){
            if(Math.abs(motorPower) > 1){
                aboveThreshold.add(Math.abs(motorPower));
            }
        }
        aboveThreshold.add(1.0);
        double maxValue = Collections.max(aboveThreshold);

        setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);

        this.leftFrontMotor.setPower(lf/maxValue);
        this.rightFrontMotor.setPower(rf/maxValue);
        this.leftBackMotor.setPower(lb/maxValue);
        this.rightBackMotor.setPower(rb/maxValue);
    }

    /*
     * Stop the robot by setting the power of all motors to 0.0
     */
    public void stop() {
        this.leftBackMotor.setPower(0.0);
        this.rightBackMotor.setPower(0.0);
        this.leftFrontMotor.setPower(0.0);
        this.rightFrontMotor.setPower(0.0);
    }

    /*
     * Set the runMode of all the chassis motors
     * @param runMode: the runMode the chassis motors should be set to
     */
    public void setModeChassisMotors(DcMotor.RunMode runMode) {
        this.leftFrontMotor.setMode(runMode);
        this.rightFrontMotor.setMode(runMode);
        this.leftBackMotor.setMode(runMode);
        this.rightBackMotor.setMode(runMode);
    }

    public void resetChassisEncoders() {
        if(this.leftFrontMotor.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            setModeChassisMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /*
     * Drive method for when the distance to drive is unspecified
     * Before using this method, ensure the motors are in the run mode RUN_USING_ENCODER
     */
    public void drivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower){
        this.leftFrontMotor.setPower(leftFrontPower);
        this.rightFrontMotor.setPower(rightFrontPower);
        this.leftBackMotor.setPower(leftBackPower);
        this.rightBackMotor.setPower(rightBackPower);
    }

    /*
     * Method for moving the arm to a given angle and extension length
     * @param angle: the angle for the arm to go to in degrees
     * @param extensionLength: the length the arm should extend/retract to with respect to its starting length
     * @return whether the arm has finished moving to the new angle and extension length
     */
    public boolean moveArm(double angle, double extensionLength){
        int angleMotorPosition = (int)(ARM_ANGLE_MOTOR_TICKS_PER_ROTATION * (angle + ARM_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG) / 360); //Change angle offset
        if (angleMotorPosition > ANGLE_MOTOR_UP_LIMIT) angleMotorPosition = ANGLE_MOTOR_UP_LIMIT;
        if (angleMotorPosition < ANGLE_MOTOR_DOWN_LIMIT) angleMotorPosition = ANGLE_MOTOR_DOWN_LIMIT;
        this.angleMotor.setTargetPosition(angleMotorPosition);
        this.angleMotor.setPower(1.0);

        int extensionMotorPosition = (int)((EXTENSION_MOTOR_TICKS_PER_ROTATION * (extensionLength - ARM_STARTING_LENGTH)) / EXTENSION_SPROCKETS_INCHES_PER_ROTATION);
        if (extensionMotorPosition > EXTENSION_MOTOR_EXTENDED_LIMIT) extensionMotorPosition = EXTENSION_MOTOR_EXTENDED_LIMIT;
        if (extensionMotorPosition < EXTENSION_MOTOR_RETRACTED_LIMIT) extensionMotorPosition = EXTENSION_MOTOR_RETRACTED_LIMIT;

        int extensionPositionOffset = (int)((double)angleMotorPosition * EXTENSION_MOTOR_ANGLE_FACTOR);

        this.extensionMotor.setTargetPosition(extensionMotorPosition + extensionPositionOffset);
        this.extensionMotor.setPower(1.0);

        return !this.angleMotor.isBusy() && !this.extensionMotor.isBusy();
    }

    /*
     * Method for moving the arm to a given (x, y) point
     * @param x: the x-component of the arm's new location
     * @param y: the y-component of the arm's new location
     * @return whether the arm has finished moving to the new (x, y) point
     */
    public boolean moveArmXY(double x, double y) {
        return moveArm(Math.toDegrees(Math.atan2(y, x)), Math.hypot(x, y));
    }

    public double getTurningAngle() {
        angles = gyroSensor.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public boolean adjustRangeDistance(DistanceSensor rangeSensor, double targetDistance, boolean isBlue) {
        int sign;
        if(isBlue) {
            sign = -1;
        }
        else {
            sign = 1;
        }

        double error = rangeSensor.getDistance(INCH) - targetDistance;
        if(error > 3.5) {
            error = 3.5;
        }

        Log.i("adjustRangeDistance", String.valueOf(error));
        if(Math.abs(error) >= 1) {
            this.drive(0.2, error * sign);
        }
        else {
            return true;
        }
        return false;
    }
}