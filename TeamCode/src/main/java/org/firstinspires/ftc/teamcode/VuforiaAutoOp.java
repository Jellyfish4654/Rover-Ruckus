package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.VuforiaNav;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

import java.util.HashMap;

@Autonomous(name = "Vuforia Auto Op")
public class VuforiaAutoOp extends LinearOpMode {
    DcMotor leftDrive, rightDrive;
    DcMotor rack;
    Servo marker;

    BNO055IMU imu;

    VuforiaNav navigation = new VuforiaNav();

    double baseAngle, trueAngle;

    public void runOpMode() {
        // Initialization
        navigation.initVuforia();

        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");
        rack = hardwareMap.dcMotor.get("rack");
        marker = hardwareMap.servo.get("marker");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rack.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        long interWait = 30;
        int gold = 0;
        int drop = 13000, clear = 8850;
        double markerHold = 0.2, markerDrop = 0.6;
        boolean dropPress = false, clearPress = false;
        boolean land = true;
        boolean depotSide = true;
        boolean safeEnd = false;
        boolean useNavigation = true;
        boolean navigationDebug = false;

        marker.setPosition(markerHold);

        while (!isStarted() && !isStopRequested()) {
            navigationDebug = gamepad1.dpad_left;

            telemetry.addData("Gold Position: ", gold);
            telemetry.addData("Drop: ", drop);
            telemetry.addData("Clear: ", clear);
            telemetry.addData("depotSide: ", depotSide);
            telemetry.addData("Land: ", land);
            telemetry.addData("Angle: ", "--");
            telemetry.addData("Safe End: ", safeEnd);
            telemetry.addData("Use Navigation: ", useNavigation);
            telemetry.addData("Debug Navigation: ", navigationDebug);
            telemetry.update();
        }

        // START
        navigation.activateNavigation();
        if (!navigationDebug) {
            if (land) {
                land(useNavigation, drop, clear);
            }
        } else {
            while (!isStopRequested()) {
                navigation.updateNav();
            }
        }
    }

    public void turn() {

    }

    public void navTurn() {

    }

    public void land(boolean useNavigation, int drop, int clear) {
        if (useNavigation) {
            rack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rack.setPower(1);

            while (!navigation.atYPos(0)) {
            }

            rack.setPower(0);
        } else {

        }
    }
}
