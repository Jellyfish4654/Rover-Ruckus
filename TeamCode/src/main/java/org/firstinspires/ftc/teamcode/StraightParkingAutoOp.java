package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Autonomous(name = "straightAuto")
public class StraightParkingAutoOp extends IterativeBaseOpMode {

    float distanceToGo = 72 * super.GEAR_RATIO;


    @Override
    public void init() {
        super.init();
       // detach(super.latch);

    }


    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
       if (distanceToGo > 10) {
          distanceToGo -= super.moveDistance(distanceToGo);
        } else {
            dropMarker();
        }

    }

}
