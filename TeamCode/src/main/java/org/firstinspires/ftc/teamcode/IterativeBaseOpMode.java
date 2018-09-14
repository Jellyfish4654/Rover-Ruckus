package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

 enum State{
    DRIVE,TANK
}




public class IterativeBaseOpMode extends OpMode {

    DcMotor leftDrive,rightDrive;
    State state = State.DRIVE;


    @Override
    public void init(){
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);


    }



    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop(){

    }


}
