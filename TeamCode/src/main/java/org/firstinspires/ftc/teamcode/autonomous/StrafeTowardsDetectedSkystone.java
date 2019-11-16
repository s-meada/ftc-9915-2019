//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.teamcode.ETC2019OpModes.NavigationWebcamVuforia;
//import org.firstinspires.ftc.teamcode.common.Robot;
//@Autonomous(name = "StrafeSkystone", group = "auto")
//public class StrafeTowardsDetectedSkystone extends LinearOpMode {
//
//    // Global variables go before runOpMode()
//    Robot robot = new Robot();
//
//
//    int state = 1;
//
//    // Capital letters and underscores for constants
//    static final int STRAFE_TO_VIEWING_POSITION = 1;
//    static final int DETERMINE_SKYSTONE = 2;
//    static final int STRAFE_TO_SKYSTONE = 3;
//    static final int STATE_END = 5;
//
//    double strafePower = 0.75;
//    double drivePower = 0.5;
//    double viewingPosition1 = 23.0; //strafe
//    double viewingPosition2 = 7.25; //forward
//    double viewingPosition3 = 12.5; //forward
//    double grabbingPosition = 20; //strafe
//
//    int skyStonePosition = 0;
//
//    boolean skyStoneVisible = false;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // init()
//        robot.initForRunToPosition(hardwareMap);
//
//        SkystoneVuforiaData vision = new SkystoneVuforiaData(hardwareMap, robot);
//
//        waitForStart(); // MUST add this yourself
//
//        vision.targetsSkyStone.activate();
//
//        while (opModeIsActive()) {  // MUST add this yourself
//            // loop()
//            telemetry.addData("Current State", state);
//
//            switch (state) {
//                case STRAFE_TO_VIEWING_POSITION:
//                    robot.strafe(strafePower, viewingPosition1);
//
//                    if (robot.strafe(strafePower, viewingPosition1)) {
//                        skyStonePosition += 1;
//                        goToNextState();
//                    }
//                    break;
//
//                case DETERMINE_SKYSTONE:
//
//                    //check for sky stone at position 1
//                    for (VuforiaTrackable trackable : vision.allTrackables) {
//
//                        //if visible move to next state
//                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                            telemetry.addData("Visible Target", trackable.getName());
//                            skyStoneVisible = true;
//                            goToNextState();
//                            break;
//                        }
//                        //else visible move to position 2
//                        else {
//                            skyStoneVisible = false;
//                            robot.drive(drivePower, viewingPosition2);
//                            skyStonePosition += 1;
//                        }
//                    }
//
//
//                    //if robot is at position 2
//                    if (robot.drive(drivePower, viewingPosition2)) {
//
//                        //check for sky stone at position 2
//                        for (VuforiaTrackable trackable : vision.allTrackables) {
//
//                            //if visible move to next state
//                            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                                telemetry.addData("Visible Target", trackable.getName());
//                                skyStoneVisible = true;
//                                goToNextState();
//                                break;
//                            }
//                            //else move to position 3
//                            else {
//                                skyStoneVisible = false;
//                                robot.drive(drivePower, viewingPosition3);
//                                skyStonePosition += 1;
//                            }
//                        }
//                    }
//
//                    //if robot is at position 3
//                    if(robot.drive(drivePower,viewingPosition2)){
//
//                        skyStonePosition += 1;
//                        //check for sky stone at position 3
//                        for (VuforiaTrackable trackable : vision.allTrackables) {
//
//                            //if visible move to next state
//                            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                                telemetry.addData("Visible Target", trackable.getName());
//                                skyStoneVisible = true;
//                                goToNextState();
//                                break;
//                            }
//                            //else move to position 3
//                            else {
//                                skyStoneVisible = false;
//                                telemetry.addData("Skystone position idetification is", skyStoneVisible);
//                            }
//                        }
//
//                    }
//
//
//                        case STRAFE_TO_SKYSTONE:
//                            telemetry.addData("Determined SkyStone Position:", skyStonePosition);
//                            robot.strafe(strafePower, grabbingPosition);
//                            break;
//
//                        default:
//                            state = STATE_END;
//                            break;
//
//            }
//        }
//    }
//
//
//
//
//
//
//            /*
//             * For each of these methods, you can add more things you want the robot to do each time it goes to a new state
//             */
//            // Increment the state variable to go to the next state
//    public void goToNextState() {
//        state++;
//    }
//
//            // Go to a specific state
//    public void goToState(int newState) {
//        state = newState;
//    }
//
//}