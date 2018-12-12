package org.firstinspires.ftc.teamcode;

public class TurnParkingAutoOp extends IterativeBaseOpMode {

    float distanceToGoOne = 24 * super.GEAR_RATIO;
    float theta = (float)Math.PI/4;
//25,45,60

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
        if(distanceToGoOne > 1) {distanceToGoOne = super.moveDistance(distanceToGoOne);}
        else if(theta > .1){theta -= super.turnTo(theta);}

    }
}
