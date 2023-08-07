package org.firstinspires.ftc.teamcode.ChasisTank.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Brazo extends SubsystemBase {

    DcMotorEx motor3;
    HardwareMap hardwareMap;
    Telemetry telemetry;


    public Brazo(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        motor3 = hardwareMap.get(DcMotorEx.class,"brazo");
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);

        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void upArm(int pos, double power){
        motor3.setTargetPosition(pos);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setPower(power);
    }

    public void setPosition(int pos, double power){
        motor3.setTargetPosition(pos);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setPower(power);



}


    @Override
    public void periodic(){
        telemetry.addData( "brazo", motor3.getCurrentPosition());
    }



}
