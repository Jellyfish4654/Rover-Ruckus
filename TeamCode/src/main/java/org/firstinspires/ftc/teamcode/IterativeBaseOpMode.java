package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

enum State {
    DRIVE, TANK
}


@TeleOp(name = "base")
public class IterativeBaseOpMode extends OpMode {

    final float GEAR_RATIO = 7 / 792; //units are turns per inch , who knows it's correct
    DcMotor leftDrive, rightDrive;
    State state = State.DRIVE;
    DcMotor latch;

//    BNO055IMU imu;
    //DO NOT USE IMU UNTIL ORIENTED AND AXES ARE AGREED UPON

    Servo marker;


    @Override
    public void init() {


        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");

        latch = hardwareMap.dcMotor.get("latch");


        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        latch.setDirection(DcMotorSimple.Direction.FORWARD);

//        latch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        marker = hardwareMap.servo.get("marker");

//        try {
//
//
//            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//            parameters.mode = BNO055IMU.SensorMode.IMU;
//            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//            parameters.loggingEnabled = false;
//
//            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//            // and named "imu".
//            imu = (BNO055IMU) hardwareMap.i2cDevice.get("IMU");
//
//
//        } catch (Exception e) {
//            telemetry.addData("imu config failed, motors initialized properly", e.getMessage());
//
//        }


    }


    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
//        telemetry.addData("imu", imu.getAngularOrientation());
    }


    public float moveDistance(float d) {
        int old = leftDrive.getTargetPosition();
        if (d > 10) {
            leftDrive.setTargetPosition((leftDrive.getTargetPosition() + 3));
            rightDrive.setTargetPosition(rightDrive.getTargetPosition() + 3);

        }


        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return (leftDrive.getTargetPosition() - old) * GEAR_RATIO;


    }


//    public double turnTo(float theta, PIDController pidAngle) {
//
//        //double turnStrength = pidAngle.calculateResultant(imu.getAngularOrientation().firstAngle,theta);
//        return pidAngle.calculateResultant(imu.getAngularOrientation().firstAngle, theta);
//
//
//    }

    public void dropMarker() {
        marker.setPosition(.5); //idk we need to find the real answer, that's a builder thing
    }

    public void detach(DcMotor motor){motor.setTargetPosition(motor.getTargetPosition()-3);}

}
