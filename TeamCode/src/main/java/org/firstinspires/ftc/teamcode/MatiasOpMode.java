package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "matias")
public class MatiasOpMode extends OpMode {
    DcMotor xAxis;


    public void init(){
        xAxis = hardwareMap.dcMotor.get("xAxis");
    }
    public void loop(){
        xAxis.setPower(gamepad1.left_stick_x);



    }

}
