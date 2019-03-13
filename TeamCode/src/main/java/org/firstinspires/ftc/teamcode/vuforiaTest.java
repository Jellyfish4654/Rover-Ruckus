package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "VuForia Test")

public class vuforiaTest extends LinearOpMode {
    // Vuforia license key
    private static final String VUFORIA_Key = "AV7cAYn/////AAAAGXDR1Nv900lOoewPO1Nq3ypDBIfk+d8X+UJOgVQZn5ZvQIY5Y4yGL6DVf24bEoMOVLCq5sZXPs9937r2zpeSZQaaaJbxeWggveVuvccsVlBdR38brId6fIRi/ssxtkUpVppCaRDO1N6K7IVbAJWrhpv1rG2DqTcS51znxjEYDE34AN6sNkurIq/qs0tLfvI+lx5VYRKdqh5LwnVt2HnpdX836kSbAN/1wnupzlLSKHcVPF9zlmRjCXrHduW8ikVefKAPGNCEzaDj4D+X+YM9iaHj9H8qN23bbaT81Ze3g5WwrXsb6dsX1N3+FqeXbiEUB02lXsmGwtvCJI89xutgPzlDAHqerduaLS2WZbL3oVyS";

    // Field dimensions
    private static final float mmPerInch = 25.4f;
    private static final float mmFieldWidth = (12 * 6) * mmPerInch;
    private static final float mmTargetHeight = (6) * mmPerInch;

    // Camera being used on phone
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    // Camera config
    final int CAMERA_FORWARD_DISPLACEMENT = 110; // eg: Camera is 110 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 200; // eg: Camera is 200 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT = 0; // eg: Camera is ON the robot's center line

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    // Localization engine variable
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {
        // Camera Preview Paramater object creation
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_Key;
        parameters.cameraDirection = CAMERA_CHOICE;

        // Instance Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load data sets for tracking objects. Stored in assets
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /*
         * Starting from Red Alliance Station looking to the center, - X axis runs from
         * Left to Right - Y axis runs from the Red Alliance Station to the Blue
         * Allience Station - Z axis runs from the floor and up Default location is
         * origin at the center of the field, rotated facing up
         */

        // Placing BlueRover target into the middle of the blue perimeter wall

        OpenGLMatrix blueRoverLocationOnField = OpenGlMatrix.translation(0, mmFieldWidth, mmTargetHeight)
                // Translation onto the blue perimeter wall
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, xyz, DEGREES, 90, 0, 0)); // Coordinate system does
                                                                                               // not rotate, order of
                                                                                               // values given after
                                                                                               // setting unit, unit
                                                                                               // set, rotate on x axis
                                                                                               // 90
        // Rotate it 90 degrees on field x axis
        blueRover.setlocation(blueRoverLocationOnField); // Set Matrix locations to blueRover

        // Placing redFootprint target onto the middle of the red perimeter wall
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix.translation(0, -mmFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, xyz, DEGREES, 90, 0, 180));
        redFootprint.setlocation(redFootprintLocationOnField);

        // Placing FrontCraters target onto the middle of the front perimeter wall
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix.translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        // Placing BackSpace target onto the middle of the back perimeter wall
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix.translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        // Set phone camera location
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, CAMERA_CHOICE == FRONT ? 90 : -90, 0,
                        0));

        // Allow for trackable listeners to know where the camera is
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot,
                    parameters.cameraDirection);
        }

        // Wait for game start
        telemetry.addData(">>", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        // Start tracking targets
        targetsRoverRuckus.activate();
        while (opModeIsActive()) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {

            }
        }
    }
}
