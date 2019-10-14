package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.teamcode.common.Robot;

import static org.firstinspires.ftc.teamcode.common.Robot.mmPerInch;

@Autonomous(name = "Ashley Auto", group = "auto")
public class AlignAndPickUpSkystone extends LinearOpMode {

    Robot robot = new Robot();

    double robotXDistanceFromSkystoneCenter;
    double robotYDistanceFromSkystoneCenter;
    double distanceForArmToExtend;
    int armUpAngle = 20;

    static final int MOVE_ARM_UP                    = 1;
    static final int FIND_CENTER_OF_SKYSTONE_VS_ARM = 2;
    static final int ADJUST_ROBOT_POSITION          = 3;
    static final int MOVE_ARM_OUT                   = 4;

    static final int STATE_END                      = 5; //EDIT as more states are added

    //TODO: Add other states mentioned in pseudo code

    int state = 1;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initForRunToPosition(hardwareMap);

        SkystoneVuforiaData vision = new SkystoneVuforiaData(hardwareMap, robot);

        waitForStart();

        vision.targetsSkyStone.activate();

        while(opModeIsActive()) {
            telemetry.addData("State", state);

            switch(state) {
                case MOVE_ARM_UP:
                    if(robot.moveArm(armUpAngle, 0)) {
                        goToNextState();
                    }
                    break;

                case FIND_CENTER_OF_SKYSTONE_VS_ARM:
                    // check all the trackable targets to see which one (if any) is visible.
                    targetVisible = false;
                    for (VuforiaTrackable trackable : vision.allTrackables) {
                        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
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
                        // express position (translation) of robot in inches.
                        VectorF translation = lastLocation.getTranslation();
                        robotXDistanceFromSkystoneCenter = translation.get(0)/mmPerInch;
                        robotYDistanceFromSkystoneCenter = translation.get(1)/mmPerInch;

                        telemetry.addData("Skystone Pos (in)", "(X, Y) = %.1f, %.1f",
                                robotXDistanceFromSkystoneCenter, robotYDistanceFromSkystoneCenter);

                        goToNextState();
                    }
                    else {
                        telemetry.addData("Visible Target", "none");
                    }
                    telemetry.update();

                    break;

                case ADJUST_ROBOT_POSITION:
                    if(robot.drive(0.75, -robotYDistanceFromSkystoneCenter - robot.Y_DISTANCE_FROM_CAMERA_TO_ARM)) {
                        goToNextState();
                    }
                    break;

                case MOVE_ARM_OUT:
                    distanceForArmToExtend = -robotXDistanceFromSkystoneCenter + robot.ARM_STARTING_LENGTH;
                    telemetry.addData("Distance from skystone", distanceForArmToExtend);
                    telemetry.update();
                    if(robot.moveArm(armUpAngle, distanceForArmToExtend)) {
                        goToNextState();
                    }
                    break;

                default:
                    state = STATE_END;
                    telemetry.update();
                    break;
            }

        }
    }

    // Increment the state variable to go to the next state
    public void goToNextState() {
        state++;
    }

    // Go to a specific state
    public void goToState(int newState) {
        state = newState;
    }
}
