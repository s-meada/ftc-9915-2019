package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class Robot {

    public boolean encodersReseted = false;

    // --- Robot Geometry --- //
    // Wheels
    double wheelDiameter = 4;
    double wheelInchesPerRotation = Math.PI * wheelDiameter;
    int motorTicksPerRotation = 1120;
    double gearRatioMotorToWheel = 32.0/24.0;
    // double type for higher accuracy when multiplying by distanceInch in driveForward() method
    double robotTicksPerInch = motorTicksPerRotation / (gearRatioMotorToWheel * wheelInchesPerRotation);

    // Arm - units: inches
    public final double ARM_STARTING_LENGTH_FROM_EDGE_OF_ROBOT = 4.0;
    public final double ARM_STARTING_LENGTH = 13.25;
    public final double ARM_INITIAL_ANGLE_STARTING_DIFFERENCE_FROM_0_DEG = 30.0;
    public final double ARM_ANGLE_MOTOR_TICKS_PER_ROTATION = 7168.0;

    public final double EXTENSION_MOTOR_TICKS_PER_ROTATION = 537.6;
    public final double EXTENSION_SPROCKETS_INCHES_PER_ROTATION = 4;
    public final double EXTENSION_MOTOR_ANGLE_FACTOR = EXTENSION_MOTOR_TICKS_PER_ROTATION/ ARM_ANGLE_MOTOR_TICKS_PER_ROTATION;


    // --- Vuforia --- //
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Constant for Stone Target
    public static final float stoneZ = 2.00f * mmPerInch;


    // --- Constants --- //

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

    // Foundation
    public static final double FOUNDATION_SERVO_UP_POSITION = 0.15;
    public static final double FOUNDATION_SERVO_DOWN_POSITION = 0.82;


    // --- Robot Hardware Variables --- //

    // Chassis motors
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;

    // Arm motors
    public DcMotor angleMotor;
    public DcMotor extensionMotor;

    // Servos
    public Servo rotationServo;
    public Servo grabberServo;
    public Servo grabberServoTwo;
    public Servo foundationServo;
    public Servo angleServo;

    // Sensors
    public WebcamName webcam;
    public DistanceSensor frontDistanceSensor;
    //public DistanceSensor frontDistanceSensor;

    public DistanceSensor sideDistanceSensor;
    public DistanceSensor foundationDistanceSensor;

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
        this.leftFrontMotor = hardwareMap.dcMotor.get("LeftFront");
        this.rightFrontMotor = hardwareMap.dcMotor.get("RightFront");
        this.leftBackMotor = hardwareMap.dcMotor.get("LeftBack");
        this.rightBackMotor = hardwareMap.dcMotor.get("RightBack");

        this.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Arm

        angleMotor = hardwareMap.dcMotor.get("angleMotor");
        angleMotor.setTargetPosition(0);
        angleMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extensionMotor = hardwareMap.dcMotor.get("extensionMotor");
        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rotationServo = hardwareMap.servo.get("rotationServo");
        grabberServo = hardwareMap.servo.get("grabberServo");
        grabberServoTwo = hardwareMap.servo.get("grabberServoTwo");
        foundationServo = hardwareMap.servo.get("foundationServo");
        angleServo = hardwareMap.servo.get("verticalServo");

        rotationServo.setPosition(ROTATION_SERVO_START_POSITION);
        grabberServo.setPosition(GRABBER_SERVO_CLOSE_POSITION);
        grabberServoTwo.setPosition(GRABBER_SERVO_TWO_CLOSE_POSITION);
        foundationServo.setPosition(FOUNDATION_SERVO_UP_POSITION);
        angleServo.setPosition(ANGLE_SERVO_INIT_POSITION); // REPLACE with initial position of this servo

        // Sensors
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
//        frontDistanceSensor = hardwareMap.get(DistanceSensor.class, "frontDistanceSensor");
//        sideDistanceSensor = hardwareMap.get(DistanceSensor.class,"sideDistanceSensor");
//        foundationDistanceSensor = hardwareMap.get(DistanceSensor.class,"foundationDistanceSensor");

        // Light
        light = hardwareMap.dcMotor.get("light");
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

        this.leftFrontMotor.setPower(y+x-w);
        this.rightFrontMotor.setPower(y-x+w);
        this.leftBackMotor.setPower(y-x-w);
        this.rightBackMotor.setPower(y+x+w);

        int targetPosition = -(int)(robotTicksPerInch * distance);

        this.leftFrontMotor.setTargetPosition(targetPosition);
        this.rightFrontMotor.setTargetPosition(targetPosition);
        this.leftBackMotor.setTargetPosition(targetPosition);
        this.rightBackMotor.setTargetPosition(targetPosition);

        setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);

        if (!this.leftFrontMotor.isBusy() || !this.rightFrontMotor.isBusy() || !this.leftBackMotor.isBusy() || !this.rightBackMotor.isBusy()) {
            encodersReseted = false;
            return true;
        } else {
            return false;
        }
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
}