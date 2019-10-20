package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;

import java.util.HashMap;

@Autonomous(name = "Master Auto", group = "auto")
public class MasterAutonomous extends LinearOpMode {

    // Global variables go before runOpMode()
    Robot robot = new Robot();

    int masterState = 1;
    int subState = 1;

    AllianceColor allianceColor = AllianceColor.BLUE;

    // --- Master States --- //
    static final int STRAFE_TO_SKYSTONE_V2 = 1;

    // --- StrafeTowardsDetectedSkystone States and Variables --- //
    // Capital letters and underscores for constants
    static final int STRAFE_TO_VIEWING_POSITION = 1;
    static final int DETECT_SKYSTONE = 2;
    static final int STRAFE_TO_SKYSTONE = 3;
    static final int STATE_END = 4;

    int skyStonePosition = 1;
    double strafePower = 0.5;
    double drivePower = 0.5;
    double viewingPosition = 18.5; //strafe

    double skyStoneCoordinateX;
    double skyStoneCoordinateY;

    // --- //




    @Override
    public void runOpMode() throws InterruptedException {
        // init()
        robot.initForRunToPosition(hardwareMap);
        SkystoneVuforiaData vision = new SkystoneVuforiaData(hardwareMap,robot);

        vision.targetsSkyStone.activate();

        waitForStart(); // MUST add this yourself

        vision.targetsSkyStone.activate();

        while(opModeIsActive()) {  // MUST add this yourself
           switch (masterState) {
               case STRAFE_TO_SKYSTONE_V2:
                   if(StrafeTowardsDetectedSkystoneV2(vision)) {
                       goToNextMasterState();
                   }
                   break;

               default:
                   break;
           }

        }
    }





    /*
     * For each of these methods, you can add more things you want the robot to do each time it goes to a new subState
     */
    // Increment the subState variable to go to the next subState
    public void goToNextSubState(){
        subState++;
    }

    // Go to a specific subState
    public void goToSubState(int newState) {
        subState = newState;
    }

    public void goToNextMasterState() {
        masterState++;
    }

    public boolean StrafeTowardsDetectedSkystoneV2(SkystoneVuforiaData vision ) {
        boolean isComplete = false;
        // loop()
        telemetry.addData("Current State", subState);

        switch(subState) {
            case STRAFE_TO_VIEWING_POSITION:
                robot.strafe(strafePower, viewingPosition);

                if(robot.strafe(strafePower, viewingPosition)); {
                robot.light.setPower(1.0);
                goToNextSubState();
                break;
            }


            case DETECT_SKYSTONE:
                HashMap<String, Float> skyStoneCoordinates = vision.getSkystoneCoordinates();
                if(skyStoneCoordinates != null){
                    skyStoneCoordinateX = skyStoneCoordinates.get("X");
                    skyStoneCoordinateY = skyStoneCoordinates.get("Y");
                    telemetry.addData("Skystone Coordinates", "(" + skyStoneCoordinateX + "," + skyStoneCoordinateY+")");
                }

                else{
                    telemetry.addData("Skystone Coordinates", "(" + skyStoneCoordinateX + "," + skyStoneCoordinateY + ")");
                }

                telemetry.update();
                break;

            case STRAFE_TO_SKYSTONE:
                if(robot.drive(0.75, -skyStoneCoordinateY + 7.75)){
                    goToNextSubState();
                }
                break;

            default:
                subState = STATE_END;
                isComplete = true;
                break;

        }
        return isComplete;
    }
}
