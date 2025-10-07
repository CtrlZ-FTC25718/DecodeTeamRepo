import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/*
 * Team CTRL+Z # 25718
 * @version 2.0, 12/30/2024
*/

@TeleOp(name = "Decode Robot-Centric Teleop")
public class DecodeRobotCentricTeleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);
    private MappedActuators robotActuators;


    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        robotActuators = new MappedActuators(hardwareMap);


    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */
        
        if(gamepad1.left_trigger == 1){
            follower.setTeleOpMovementVectors(-0.25 * gamepad1.right_stick_y, -.25 * gamepad1.right_stick_x, -0.2 * gamepad1.left_stick_x, true);
        }
        else{
            follower.setTeleOpMovementVectors(-0.60 * gamepad1.right_stick_y, -0.60* gamepad1.right_stick_x, -0.5 * gamepad1.left_stick_x, true);
        }

        // Intake - Intake action
        if (gamepad1.dpad_down) {
            // Intake artifact
            robotActuators.spinIntake("intake");
        }
        // Intake - Reject action
        else if (gamepad1.dpad_up){
            // Reject artifact
            robotActuators.spinIntake("reject");
        }
        else if ((gamepad1.dpad_left) || (gamepad1.dpad_right)) {
            // Stop intake
            robotActuators.spinIntake("stop");
        }

        if(gamepad1.y){
            // Start shooter
            robotActuators.spinShooter("start");
        }

        if(gamepad1.a){
            // Stop shooter
            robotActuators.spinShooter("stop");
        }


        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}