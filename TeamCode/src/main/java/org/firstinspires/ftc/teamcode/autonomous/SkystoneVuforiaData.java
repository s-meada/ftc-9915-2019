package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.RobotParameters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.common.Robot.mmPerInch;
import static org.firstinspires.ftc.teamcode.common.Robot.stoneZ;

public class SkystoneVuforiaData {
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false;

    public static final String VUFORIA_KEY = RobotParameters.VUFORIA_KEY;

    public VuforiaLocalizer vuforia = null;
    public VuforiaLocalizer.Parameters parameters = null;
    public VuforiaTrackables targetsSkyStone = null;
    public List<VuforiaTrackable> allTrackables = null;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    public float phoneXRotate    = 0;
    public float phoneYRotate    = 0;
    public float phoneZRotate    = 0;

    private HardwareMap _hardwareMap = null;
    private Robot _robot = null;

    SkystoneVuforiaData(HardwareMap hardwareMap, Robot robot) {
        _hardwareMap = hardwareMap;
        _robot = robot;

        initializeVuforiaSkystone();
        initializeTrackables();

    }

    private void initializeVuforiaSkystone() {
        // --- Vuforia --- //

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         *
         */

        int cameraMonitorViewId = _hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", _hardwareMap.appContext.getPackageName());
        this.parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        this.parameters.vuforiaLicenseKey = VUFORIA_KEY;

        this.parameters.cameraName = _robot.webcam;
        /**
         * We also indicate which camera on the RC we wish to use.
         */

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

    }

    private void initializeTrackables() {

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 7.0f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    // Detecting skystones
    // If a skystone is detected, this method will return a HashMap with the (x, y) coordinates of the skystone
    // If there are no skystones detected, this method will return null
    public HashMap<String, Double> getSkystoneCoordinates() {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : this.allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            HashMap<String, Double> skystoneXYcoordinates = new HashMap<>();
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();

            double Xtranslation = (double)translation.get(0)/mmPerInch;
            double Ytranslation = (double)translation.get(1)/mmPerInch;
            double Yoffset = Xtranslation * Math.tan(Math.toRadians(90 - (rotation.thirdAngle)));
            double Ycorrected = Ytranslation + Yoffset;

            skystoneXYcoordinates.put("X", Xtranslation); // x-coordinate of skystone
            skystoneXYcoordinates.put("Y Offset", Yoffset);
            skystoneXYcoordinates.put("Y", Ytranslation); // y-coordinate of skystone
            skystoneXYcoordinates.put("Y Corrected", Ycorrected);
            skystoneXYcoordinates.put("Heading", (double)rotation.thirdAngle);

            return skystoneXYcoordinates;
        }

        // No skystone detected
        return null;
    }

}