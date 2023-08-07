package org.firstinspires.ftc.teamcode.ChasisTank.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake2S extends SubsystemBase {

ServoEx leftservo;
ServoEx rightservo;
HardwareMap hardwareMap;
Telemetry telemetry;

public Intake2S(Telemetry telemetry, HardwareMap hardwareMap){

this.telemetry = telemetry;
this.hardwareMap = hardwareMap;
leftservo = new SimpleServo(hardwareMap,"servoizq", 0, 180);
rightservo = new SimpleServo(hardwareMap,"servoder", 0, 180);


leftservo.setInverted(true);

}
//right
public void close(){
    leftservo.turnToAngle(100);
    rightservo.turnToAngle(180);
}
//left
public void open(){
    leftservo.turnToAngle(50);
    rightservo.turnToAngle(130);


}


}






