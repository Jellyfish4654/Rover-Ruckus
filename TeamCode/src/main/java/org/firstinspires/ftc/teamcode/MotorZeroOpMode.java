package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name = "zero")
public class MotorZeroOpMode extends OpMode {

    DcMotor motor;
    public void init(){
        motor = hardwareMap.dcMotor.get("slurp");

    }

    public void loop(){

        motor.setPower(15);
    }

}
