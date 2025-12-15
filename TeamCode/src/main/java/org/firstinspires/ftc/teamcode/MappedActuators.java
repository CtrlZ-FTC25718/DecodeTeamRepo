package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

import kotlinx.coroutines.Delay;

public class MappedActuators {
    private static final String intakeMotor_Name = "intakeMotor";
    private static final String shooterFrontMotor_Name = "shooterFrontMotor";
    private static final String sorterServo_Name = "sorterServo";
    private static final String doorServo_Name = "door";
    private static final String shooterBlocker_Name = "shooterBlocker";
    private static final String shooterBackMotor_Name = "shooterBackMotor";
    private static final String sorterBottomColorSensor_Name = "sorterBottomColorSensor";
    private static final String rgbIndicator_Name = "rgbIndicator";
    private HardwareMap map;
    private Servo sorterServo;
    private DcMotor intake;
    private DcMotor shooterFront;
    private DcMotor shooterBack;
    public Servo door;
    private Servo shooterBlocker;
    private ColorSensor sorterBottomColorSensor;
    private Servo rgbIndicator;

    private ElapsedTime runtime;

    private int sorterState = 0; // Sorter State

    private double sorterTollerance = 0.01;
    private int[] sorterBottomColorSensorRGB = {0, 0, 0};
    private double intakeServoPosOffset= 0.0;
    private double sorterPos0 = 0.105 + intakeServoPosOffset;
    private double sorterPos1 = 0.175 + intakeServoPosOffset;
    private double sorterPos2 = 0.245 + intakeServoPosOffset;
    private double sorterPos3 = 0.305 + intakeServoPosOffset;
    private double sorterPos4 = 0.380 + intakeServoPosOffset;
    private double sorterPos5 = 0.455 + intakeServoPosOffset;
    private double sorterPos6 = 0.535 + intakeServoPosOffset;

    private double sorterPos7 = 0.610 + intakeServoPosOffset;
    private boolean doorOpen = false;
    private boolean shooterBlocked = false;
    private double shooterBlockerOpenPos = 0.0;
    private double shooterBlockerClosePos = 0.3;
    private double shooterBlockerTOllerance = 0.01;
    private double shooterHighPower = 1.0;
    private double shooterLowPower = 0.85;

    private double doorOpenPos = 0.25;
    private double doorClosedPos = 0.6;
    private double doorTollerance = 0.02;
    private int intakeBallCount = 0;

    private String ballColor = "Empty";


    public MappedActuators(HardwareMap hardwareMap) {
        map = hardwareMap;
        sorterServo = map.get(Servo.class, sorterServo_Name);
        intake = map.get(DcMotor.class, intakeMotor_Name);
        shooterFront = map.get(DcMotor.class, shooterFrontMotor_Name);
        shooterBack = map.get(DcMotor.class, shooterBackMotor_Name);
        door = map.get(Servo.class, doorServo_Name);
        shooterBlocker = map.get(Servo.class, shooterBlocker_Name);
        sorterBottomColorSensor = map.get(ColorSensor.class, sorterBottomColorSensor_Name);
        rgbIndicator = map.get(Servo.class, rgbIndicator_Name);

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

        // set rgbIndicator properties
        rgbIndicator.scaleRange(0.0, 1.0);

        // door state
        doorOpen = false;

        // start time
        runtime = new ElapsedTime();

    }

    public void decrementIntakeBallCount(){intakeBallCount--;}
    public void incrementIntakeBallCount() {intakeBallCount++;}
    public void resetIntakeBallCount(){intakeBallCount = 0;}
    public int getIntakeBallCount(){ return intakeBallCount;}

    public void spinIntake(String direction) {
        // Spin intake based upon direction
        switch (direction) {
            case "reject": // reject
                intake.setDirection(DcMotorSimple.Direction.REVERSE);
                intake.setPower(1);
                break;
            case "intake":// intake
                intake.setDirection(DcMotor.Direction.FORWARD);
                intake.setPower(1);
                break;
            case "stop":
                // Stop spinning
                intake.setPower(0);
                break;
        }
    }

    public void spinShooter(String action, String powerLevel) {
        // Start or stop the shooter
        if (action.equals("start")) {
            if (powerLevel.equals("high")) {
                shooterFront.setPower(shooterHighPower);
                shooterBack.setPower(shooterHighPower);
            }
            else { // "low"
                shooterFront.setPower(shooterLowPower);
                shooterBack.setPower(shooterLowPower);
            }
        }

        if (action.equals("stop")) { // power level doesn't matter
            shooterFront.setPower(0);
            shooterBack.setPower(0);
        }
    }

    public int getSorterState() {
        return sorterState;
    }

    public void setSorterState(int state) {
        sorterState = state;
    }

    public void resetSorter() {

        sorterServo.setPosition(sorterPos0);
        sorterState = 0;
        resetIntakeBallCount();
        door.close();
    }

    public double getSorterPosition(){
        return sorterServo.getPosition();
    }

    public void setSorterPosition(double newPos){
        sorterServo.setPosition(newPos);
    }

    public void sorterGoToState(int state, boolean wiggle) {
//        int wiggleCount = 200, count = 0;
//        double wigglePos = 0.0, wiggleMag = 0.0025;
//        double newSorterPos = 0.0;
//        boolean sorterReachedTargetPos = false;
//
//        if (state <= 0) {
//            resetSorter();
//            newSorterPos = sorterPos0;
//        } else if (state == 1) {
//            sorterServo.setPosition((sorterPos1));
//            setSorterState(1);
//            newSorterPos = sorterPos1;
//        } else if (state == 2) {
//            sorterServo.setPosition((sorterPos2));
//            setSorterState(2);
//            newSorterPos = sorterPos2;
//        } else if (state == 3) {
//            sorterServo.setPosition((sorterPos3));
//            setSorterState(3);
//            newSorterPos = sorterPos3;
//        } else if (state == 4) {
//            sorterServo.setPosition((sorterPos4));
//            setSorterState(4);
//            newSorterPos = sorterPos4;
//        } else if (state == 5) {
//            sorterServo.setPosition((sorterPos5));
//            setSorterState(5);
//            newSorterPos = sorterPos5;
//        } else if (state == 6) {
//            sorterServo.setPosition((sorterPos6));
//            setSorterState(6);
//            newSorterPos = sorterPos6;
//        } else if (state == 7) {
//            sorterServo.setPosition((sorterPos6));
//            setSorterState(7);
//            newSorterPos = sorterPos7;
//        }
//
////        while (withinTol(newSorterPos, sorterServo.getPosition(), sorterTollerance)){
////            sorterReachedTargetPos = false;
////        }
////        sorterReachedTargetPos = true;
//
//        if (wiggle) {
//            wigglePos = sorterServo.getPosition();
//            while (count < wiggleCount) {
//                sorterServo.setPosition((wigglePos - wiggleMag));
//                sorterServo.setPosition((wigglePos + wiggleMag));
//                count++;
//            }
//            sorterServo.setPosition(wigglePos);
//        }
//
//        setRGBIndicatorByIntakeBallCount();

    }

    public boolean getDoorOpenState() {
        return doorOpen;
    }

    public void setDoorOpenState(boolean state) {
        doorOpen = state;
    }

    public void openDoor() {
        door.setPosition(doorOpenPos);
//        while(withinTol(doorClosedPos, door.getPosition(), doorTollerance)){
//            setDoorOpenState(false);
//        }
        setDoorOpenState(true);

    }

    public void closeDoor() {
        door.setPosition(doorClosedPos);
//        while(withinTol(doorClosedPos, door.getPosition(), doorTollerance)){
//            setDoorOpenState(true);
//        }
        setDoorOpenState(false);
    }

    public boolean getShooterBlockerState() {
        return shooterBlocked;
    }

    public void setShooterBlockedState(boolean state) {
        shooterBlocked = state;
    }
    public double getShooterBlockerPosition() {
        return shooterBlocker.getPosition();
    }

    public void setShooterBlockerPosition(double position) {
        shooterBlocker.setPosition(position);
    }

    public void blockShooter() {
        setShooterBlockerPosition(shooterBlockerClosePos);
//        while (withinTol(shooterBlockerClosePos, shooterBlocker.getPosition(),shooterBlockerTOllerance)){
//            setShooterBlockedState(false);
//        }
        setShooterBlockedState(true);
    }

    public void unblockShooter() {
        setShooterBlockerPosition(shooterBlockerOpenPos);
//        while (withinTol(shooterBlockerOpenPos, shooterBlocker.getPosition(),shooterBlockerTOllerance)) {
//            setShooterBlockedState(true);
//        }
        setShooterBlockedState(false);
    }

    public int getSorterBottomColorSensorRValue() {
        return sorterBottomColorSensorRGB[0];
    }

    public int getSorterBottomColorSensorGValue() {
        return sorterBottomColorSensorRGB[1];
    }

    public int getSorterBottomColorSensorBValue() {
        return sorterBottomColorSensorRGB[2];
    }

    public void readsorterBottomColorSensorRGB() {
        sorterBottomColorSensorRGB[0] = sorterBottomColorSensor.red();
        sorterBottomColorSensorRGB[1] = sorterBottomColorSensor.green();
        sorterBottomColorSensorRGB[2] = sorterBottomColorSensor.blue();
    }

    public boolean detectBall() {
        int iter = 0, maxIter = 5, detectedCount = 0;
        boolean ballDetected = false;
        while (iter < maxIter){
            readsorterBottomColorSensorRGB();
            if (sorterBottomColorSensorRGB[0] > 100 || sorterBottomColorSensorRGB[1] > 100 || sorterBottomColorSensorRGB[2] > 100) {
                detectedCount++;
            }
            iter++;
            delay("high");
        }
        if (detectedCount >= iter/2) { ballDetected = true;}
        else { ballDetected = false;}

        return ballDetected;
    }

    public String detectBallColor(){
        String ballColor = "Empty";
        readsorterBottomColorSensorRGB();
        if (getSorterBottomColorSensorGValue() < 120) {
            ballColor = "Purple";
        }

        else if (getSorterBottomColorSensorGValue() > 120) {
            ballColor = "Green";
        }

        else {
            ballColor = "Empty";
        }
        return ballColor;
    }

    public void setRGBIndicatorTo(double color) {
        rgbIndicator.setPosition(color);
    }

    public void setRGBIndicatorByIntakeBallCount(){
        if (intakeBallCount == 0){
            rgbIndicator.setPosition(0.0);
        }
        else if (intakeBallCount == 1){
            rgbIndicator.setPosition(0.35);
        }
        else if (intakeBallCount == 2){
            rgbIndicator.setPosition(0.5);
        }
        else if (intakeBallCount == 3) {
            rgbIndicator.setPosition(0.555);
        }
        else {
            rgbIndicator.setPosition(1);
        }
    }

    public void shootBall(String powerLevel) {
        spinIntake("stop");
        spinShooter("start", powerLevel);
        openDoor(); // shoot ball at the door first
        decrementIntakeBallCount();
        delay("very high"); delay("very high"); delay("very high");

        // Shoot rest of the balls
        while (intakeBallCount > 0) {
            unblockShooter();
            sorterGoToState(getSorterState() + 1, true);
            delay("high"); delay("high");
            decrementIntakeBallCount();
        }
        // Shoot two extra (to handle the case with just 2 balls in the sorter
        sorterGoToState(getSorterState() + 1, true);
        delay("high"); delay("high");
        sorterGoToState(getSorterState() + 1, true);
        delay("high"); delay("high");

        // sorter is guaranteed to be empty, so reset
        resetSorter();
        blockShooter();
        spinShooter("stop", powerLevel);
        closeDoor();
        setRGBIndicatorTo(0.0);
        spinIntake("intake"); // Not sure why "intake" rejects here.
    }

    private boolean withinTol(double target, double actual, double tol){
        return Math.abs(target - actual) <= tol;
    }

    public void delay(String level){
        int count = 0, countLevel = 0;
        switch (level) {
            case "low":
                countLevel = 10000;
                break;
            case "medium":
                countLevel = 20000;
                break;
            case "high":
                countLevel = 30000;
                break;
            default:
                countLevel = 40000;
                break;
        }

        while (count < countLevel) {count++;};
    }
}
