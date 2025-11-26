package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import kotlinx.coroutines.Delay;

public class MappedActuators {

//    private static final String taskServoOne_Name = "s0";
//    private static final String taskServoTwo_Name = "s1";

    private static final String intakeMotor_Name = "intakeMotor";
    private static final String shooterFrontMotor_Name = "shooterFrontMotor";
    private static final String sorterServo_Name = "sorterServo";
    private static final String doorServo_Name = "door";
    private static final String shooterBlock_Name = "shooterBlocker";
    private static final String shooterBackMotor_Name = "shooterBackMotor";
    private static final String sorterBottomColorSensor_Name = "sorterBottomColorSensor";
    private HardwareMap map;
    private Servo sorterServo;
    private DcMotor intake;
    private DcMotor shooterFront;
    private DcMotor shooterBack;
    private Servo door;
    private Servo shooterBlocker;
    private ColorSensor sorterBottomColorSensor;

    private int sorterState = 0; // Sorter State
    private int [] sorterBottomColorSensorRGB = {0,0,0};

    private double sorterPos1 = 0.08;
    private double sorterPos2 = 0.15;
    private double sorterPos3 = 0.22;
    private double sorterPos4 = 0.3;

    public MappedActuators(HardwareMap hardwareMap){
        map = hardwareMap;
//        taskServoOne = map.get(Servo.class, taskServoOne_Name);
        sorterServo = map.get(Servo.class, sorterServo_Name);
        intake = map.get(DcMotor.class, intakeMotor_Name);
        shooterFront = map.get(DcMotor.class, shooterFrontMotor_Name);
        shooterBack = map.get(DcMotor.class, shooterBackMotor_Name);
        door = map.get(Servo.class,doorServo_Name);
        shooterBlocker = map.get(Servo.class,shooterBlock_Name);
        sorterBottomColorSensor = map.get(ColorSensor.class, sorterBottomColorSensor_Name);

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
    
    public int getSorterState(){
        return sorterState;
    }

    public void sorterGoToState(int state, int direction){
        if (state == 0) { resetSorter();}
        if (state == 1) { sorterServo.setPosition((sorterPos1));}
        if (state == 2) { sorterServo.setPosition((sorterPos2));}
        if (state == 3) { sorterServo.setPosition((sorterPos3));}
        if (state == 4) { sorterServo.setPosition((sorterPos4));}

        if (direction == 1) { sorterState++;}
        else if (direction == -1) { sorterState--;}
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

    public int getSorterBottomColorSensorRValue(){
        return sorterBottomColorSensorRGB[0];
    }
    public int getSorterBottomColorSensorGValue(){
        return sorterBottomColorSensorRGB[1];
    }
    public int getSorterBottomColorSensorBValue(){
        return sorterBottomColorSensorRGB[2];
    }
    public void readsorterBottomColorSensorRGB() {
        sorterBottomColorSensorRGB[0] = sorterBottomColorSensor.red();
        sorterBottomColorSensorRGB[1] = sorterBottomColorSensor.green();
        sorterBottomColorSensorRGB[2] = sorterBottomColorSensor.blue();
    }

    public boolean detectBall(){
        boolean ballDetected = false;
        readsorterBottomColorSensorRGB();
        if (sorterBottomColorSensorRGB[0] > 80 || sorterBottomColorSensorRGB[1] > 80 || sorterBottomColorSensorRGB[2] > 80){
            ballDetected = true;
        }
        return ballDetected;
    }

}
