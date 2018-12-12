package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class StraightParkingAutoOp extends IterativeBaseOpMode {

    float distanceToGo = 72 * super.GEAR_RATIO;


    @Override
    public void init(){
     super.init();


    }



    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop(){
        distanceToGo -= super.moveDistance(distanceToGo);

    }

}
