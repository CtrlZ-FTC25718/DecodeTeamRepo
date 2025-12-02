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
    private int[] sorterBottomColorSensorRGB = {0, 0, 0};

    private double sorterPos1 = 0.085;
    private double sorterPos2 = 0.150;
    private double sorterPos3 = 0.225;
    private double sorterPos4 = 0.295;
    private double sorterPos5 = 0.365;
    private double sorterPos6 = 0.440;

    private double sorterPos7 = 0.51;
    private boolean doorOpen = false;
    private double shooterOpenPos = 0.0;
    private double shooterClosePos = 0.3;
    private double shooterHighPower = 1.0;
    private double shooterLowPower = 0.85;

    private double doorOpenPos = 0.3;
    private double doorClosedPos = 0.8;
    private int intakeBallCount = 0;


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
        if (direction.equals("intake")) {
            //Intake
            intake.setDirection(DcMotor.Direction.REVERSE);
            intake.setPower(1);
        } else if (direction.equals("reject")) {
            // Reject
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(1);
        } else if (direction.equals("stop")) {
            // Stop spinning
            intake.setPower(0);
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

        sorterServo.setPosition(0.02);
        sorterState = 0;
        resetIntakeBallCount();
    }

    public void sorterGoToState(int state, boolean wiggle) {
        int wiggleCount = 200, count = 0;
        double wigglePos = 0.0, wiggleMag = 0.0025;

        if (state == 0) {
            resetSorter();
        } else if (state == 1) {
            sorterServo.setPosition((sorterPos1));
            setSorterState(1);
        } else if (state == 2) {
            sorterServo.setPosition((sorterPos2));
            setSorterState(2);
        } else if (state == 3) {
            sorterServo.setPosition((sorterPos3));
            setSorterState(3);
        } else if (state == 4) {
            sorterServo.setPosition((sorterPos4));
            setSorterState(4);
        } else if (state == 5) {
            sorterServo.setPosition((sorterPos5));
            setSorterState(5);
        } else if (state == 6) {
            sorterServo.setPosition((sorterPos6));
            setSorterState(6);
        } else if (state == 7) {
            sorterServo.setPosition((sorterPos6));
            setSorterState(7);
        }

        if (wiggle) {
            wigglePos = sorterServo.getPosition();
            while (count < wiggleCount) {
                sorterServo.setPosition((wigglePos - wiggleMag));
                sorterServo.setPosition((wigglePos + wiggleMag));
                count++;
            }
            sorterServo.setPosition(wigglePos);
        }

    }

    public boolean getDoorState() {
        return doorOpen;
    }

    public void setDoorState(boolean state) {
        doorOpen = state;
    }

    public void openDoor() {
        door.setPosition(doorOpenPos);
        setDoorState(true);

    }

    public void closeDoor() {
        door.setPosition(doorClosedPos);
        setDoorState(false);
    }

    public double getShooterBlockerPosition() {
        return shooterBlocker.getPosition();
    }

    public void setShooterBlockerPosition(double position) {
        shooterBlocker.setPosition(position);
    }

    public void blockShooter() {
        setShooterBlockerPosition(shooterClosePos);

    }

    public void unblockShooter() {
        setShooterBlockerPosition(shooterOpenPos);
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
        boolean ballDetected = false;
        readsorterBottomColorSensorRGB();
        if (sorterBottomColorSensorRGB[0] > 100 || sorterBottomColorSensorRGB[1] > 100 || sorterBottomColorSensorRGB[2] > 100) {
            ballDetected = true;
        }
        return ballDetected;
    }

    public void setRGBIndicatorTo(double color) {
        rgbIndicator.setPosition(color);
    }

    public void shootBall(String powerLevel) {
        spinShooter("start", powerLevel);

        while (intakeBallCount > 0) {
            openDoor();
            unblockShooter();
            sorterGoToState(getSorterState() + 1, true);
            decrementIntakeBallCount();
        }
        // Shoot one extra (to handle the case with just 2 balls in the sorter
        sorterGoToState(getSorterState() + 1, true);

        // sorter is guaranteed to be empty, so reset
        spinShooter("stop", powerLevel);
        closeDoor();
        resetSorter();
        blockShooter();
        //spinIntake("intake");
    }
}
