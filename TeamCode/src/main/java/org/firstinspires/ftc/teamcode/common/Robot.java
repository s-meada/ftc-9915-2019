package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

public class Robot {
    double wheelDiameter = 6;
    double wheelInchesPerRotation = Math.PI * wheelDiameter;
    int motorTicksPerRotation = 1120;
    double gearRatioMotorToWheel = 40.0/80.0;
    // double type for higher accuracy when multiplying by distanceInch in driveForward() method
    double robotTicksPerInch = motorTicksPerRotation / (gearRatioMotorToWheel * wheelInchesPerRotation);

    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;

    public void init(HardwareMap hardwareMap) {
        this.leftFrontMotor = hardwareMap.dcMotor.get("LeftFront");
        this.rightFrontMotor = hardwareMap.dcMotor.get("RightFront");
        this.leftBackMotor = hardwareMap.dcMotor.get("LeftBack");
        this.rightBackMotor = hardwareMap.dcMotor.get("RightBack");

        this.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Drives forward if distanceInch is positive; drives backward if distanceInch is negative
    // Power must be positive
    public boolean drive(double power, double distanceInch) {
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

        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        return !this.leftFrontMotor.isBusy() || !this.leftBackMotor.isBusy() || !this.rightFrontMotor.isBusy() || !this.rightBackMotor.isBusy();
    }

    // Strafes right if distanceInch is positive; strafes left if distanceInch is negative
    // Power must be positive
    public boolean strafe(double power, double distanceInch) {
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

        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        return !this.leftFrontMotor.isBusy() || !this.leftBackMotor.isBusy() || !this.rightFrontMotor.isBusy() || !this.rightBackMotor.isBusy();
    }

    // Added for 2019 Training
    /*
     * Autonomous mecanum drive method
     * @param y: amount of movement forward/backward
     * @param x: amount of movement right/left
     * @param w: (turning) direction in radians
     * @param distance: distance for robot to move in inches
     * @return whether the robot has reached that distance
     */
    public boolean driveMecanum(double y, double x, double w, double distance) {
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

        this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        return !this.leftFrontMotor.isBusy() || !this.rightFrontMotor.isBusy() || !this.leftBackMotor.isBusy() || !this.rightBackMotor.isBusy();
    }

    public void stop() {
        this.leftBackMotor.setPower(0.0);
        this.rightBackMotor.setPower(0.0);
        this.leftFrontMotor.setPower(0.0);
        this.rightFrontMotor.setPower(0.0);
    }
}