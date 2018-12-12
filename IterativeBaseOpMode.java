package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

enum State {
    DRIVE, TANK
}


@TeleOp(name = "base")
public class IterativeBaseOpMode extends OpMode {

    final float GEAR_RATIO = 1 / (2 * 9.9f); //units are turns per inch , who knows it's correct
    DcMotor leftDrive, rightDrive;
    State state = State.DRIVE;
    BNO055IMU imu;

    @Override
    public void init() {
        try {
            leftDrive = hardwareMap.dcMotor.get("leftDrive");
            rightDrive = hardwareMap.dcMotor.get("rightDrive");

            leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = (BNO055IMU) hardwareMap.i2cDevice.get("imu");
        } catch (Exception e) {


        }


    }


    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        telemetry.addData("imu", imu.getAngularOrientation());
    }


    public float moveDistance(float d) {
        int old = leftDrive.getTargetPosition();
        if (d < 1 / GEAR_RATIO) {
            leftDrive.setTargetPosition((int) (d * GEAR_RATIO) + leftDrive.getTargetPosition());
            leftDrive.setTargetPosition((int) (d * GEAR_RATIO) + leftDrive.getTargetPosition());
        } else {
            leftDrive.setTargetPosition((leftDrive.getTargetPosition()));
            rightDrive.setTargetPosition(rightDrive.getTargetPosition());

        }

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return (leftDrive.getTargetPosition() - old) * GEAR_RATIO;

    }


    public float turnTo(float theta) {
        int old = leftDrive.getTargetPosition();
        leftDrive.setTargetPosition((int) (14.5 / GEAR_RATIO * theta) + leftDrive.getTargetPosition());
        rightDrive.setTargetPosition(-1 * ((int) (14.5 / GEAR_RATIO * theta) + leftDrive.getTargetPosition()));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        return (float) (2 * (leftDrive.getTargetPosition() - old) / (14.5 / GEAR_RATIO));


    }
}
