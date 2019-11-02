package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

@TeleOp(name="TeleopResetEncoders",group="Skystone")
public class ResetEncoders extends OpMode {
    public DcMotor angleMotor;
    public DcMotor extensionMotor;

    public void init(){
        angleMotor = hardwareMap.dcMotor.get("angleMotor");
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor = hardwareMap.dcMotor.get("extensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void loop(){

    }

}
