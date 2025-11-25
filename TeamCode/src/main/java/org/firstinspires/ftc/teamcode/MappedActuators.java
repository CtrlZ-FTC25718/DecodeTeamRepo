package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MappedActuators {

//    private static final String taskServoOne_Name = "s0";
//    private static final String taskServoTwo_Name = "s1";

    private static final String intakeMotor_Name = "intakeMotor";
    private static final String shooterFrontMotor_Name = "shooterFrontMotor";
    private static final String sorterServo_Name = "sorterServo";
    private static final String doorServo_Name = "door";
    private static final String shooterBlock_Name = "shooterBlocker";
    private static final String shooterBackMotor_Name = "shooterBackMotor";
    private HardwareMap map;
    private Servo sorterServo;
    private DcMotor intake;
    private DcMotor shooterFront;
    private DcMotor shooterBack;
    private Servo door;
    private Servo shooterBlocker;

    private int sorterState = 0; // Sorter State

    public MappedActuators(HardwareMap hardwareMap){
        map = hardwareMap;
//        taskServoOne = map.get(Servo.class, taskServoOne_Name);
        sorterServo = map.get(Servo.class, sorterServo_Name);
        intake = map.get(DcMotor.class, intakeMotor_Name);
        shooterFront = map.get(DcMotor.class, shooterFrontMotor_Name);
        shooterBack = map.get(DcMotor.class, shooterBackMotor_Name);
        door = map.get(Servo.class,doorServo_Name);
        shooterBlocker = map.get(Servo.class,shooterBlock_Name);

        // set intake properties
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set shooter properties
        shooterFront.setDirection(DcMotor.Direction.REVERSE);
        shooterFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterBack.setDirection(DcMotor.Direction.REVERSE);
        shooterBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set sorter properties
        sorterServo.setDirection(Servo.Direction.FORWARD);
        sorterServo.setPosition(0);

        // set shooterBlocker properties
        shooterBlocker.setDirection(Servo.Direction.FORWARD);
        sorterServo.setPosition(0);
    }

    public void spinIntake(String direction){
        // Spin intake based upon direction
        if (direction.equals("intake")){
            //Intake
            intake.setDirection(DcMotor.Direction.REVERSE);
            intake.setPower(1);
        }
        else if (direction.equals("reject")){
            // Reject
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(1);
        }
        else {
            // Stop spinning
            intake.setPower(0);
        }
    }

    public void spinShooter(String action){
        // Start or stop the shooter
        if (action.equals("start")){
            shooterFront.setPower(1);
            shooterBack.setPower(1);
        }

        if (action.equals("stop")){
            shooterFront.setPower(0);
            shooterBack.setPower(0);
        }
    }

    public void sortArtifactForward() {
        if (sorterState == 0) {
            sorterServo.setPosition(0.08);
            sorterState++;
        } else if (sorterState == 1) {
            sorterServo.setPosition(0.15);
            sorterState++;
        } else if (sorterState == 2) {
            sorterServo.setPosition(0.22);
            sorterState++;
        } else if (sorterState == 3) {
            sorterServo.setPosition(0.3);
            sorterState++;
        }
          else {
            // do nothing until we are sure that reverse doesn't jam the artifacts
        }
    }


    public void sortArtifactBackward() {
        if (sorterState == 1) {
            // Do nothing until we know reversing doesn't jam the artifacts
        } else if (sorterState == 2) {
            sorterServo.setPosition(0.08);
            sorterState--;
        } else if (sorterState == 3) {
            sorterServo.setPosition(0.15);
            sorterState--;
        } else if (sorterState == 4) {
            sorterServo.setPosition(0.22);
            sorterState--;
        }
    }
    public void resetSorter() {
        sorterServo.setPosition(0.02);
        sorterState = 0;
    }

    public void doorOpen() {
        door.setPosition(0);
    }

    public void doorClosed() {
        door.setPosition(0.25);
    }

    public void blockShooter() {
        shooterBlocker.setPosition(0.01);
    }

    public void unblockShooter() {
        shooterBlocker.setPosition(0.08);
    }

}
