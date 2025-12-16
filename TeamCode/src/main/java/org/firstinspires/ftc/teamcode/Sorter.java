package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import android.util.Log;



public class Sorter {

    //Hardware
    private Servo sorterServo;
    private Servo doorServo;
    private ColorSensor detector;

    private Servo indicatorLight;

    //State
    private ArrayList<Object> state;

    //Constructor
    public Sorter(HardwareMap map){

        sorterServo = map.get(Servo.class, "sorterServo");
        doorServo = map.get(Servo.class, "door");
        detector = map.get(ColorSensor.class, "sorterBottomColorSensor");
        indicatorLight = map.get(Servo.class, "rgbIndicator");

        sorterServo.setPosition(0.115); //Change to Specify Offset

        //State: {Position of SorterServo, Ball Color Stack, Closed/Open Door}; Ball Color Stack in Order: Closest to Shoot --> Farthest to Shoot
        state = new ArrayList<Object>();
        state.add(sorterServo.getPosition()); // state.get/set(0)
        state.add(new String[] {"", "", ""}); // state.get/set(1)
        state.add("Closed"); // state.get/set(2)

        sorterServo.setDirection(Servo.Direction.FORWARD);
        // sorterServo.setPosition(0);

        indicatorLight.scaleRange(0.0, 1.0);

    }

    public void shift(int numberPositions){
        //Update sorterServo Position to reflect shift
        state.set(0, (double) state.get(0) + .07 * numberPositions);

        //Initialize temporary storage for shuffling
        String temp = "";
        String[] tempArray = {"", "", ""};

        //Shuffle
        tempArray = (String[])state.get(1);
        //Shuffle counterclockwise for positive "numberPositions" when looking into intake
        if(numberPositions > 0){
            for(int i = 0; i < numberPositions; i++){
                temp = tempArray[0];
                tempArray[0] = tempArray[1];
                tempArray[1] = tempArray[2];
                tempArray[2] = temp;
            }
        }
        //Shuffle clockwise for negative "numberPositions" when looking into intake
        else{
            for(int i = 0; i < Math.abs(numberPositions); i++){
                temp = tempArray[2];
                tempArray[2] = tempArray[1];
                tempArray[1] = tempArray[0];
                tempArray[0] = temp;
            }
        }
        state.set(1, tempArray);
    }

    public void detect(){
        //Note: 30 Samples Used for Statistical Significance of Mean by Central Limit Th. of Statistics

        boolean artifactPresent = false;

        //Determine if Artifact Present
        double numberPositiveDetects = 0;
        for(int i = 0; i < 30; i++){
            if(detector.red() > 150 || detector.green() > 150 || detector.blue() > 150){
                numberPositiveDetects++;
            }
        }
        artifactPresent = numberPositiveDetects > 15;
        if(artifactPresent){
            String[] tempArray = (String []) state.get(1);
            tempArray[2] = "Ball";
            state.set(1, tempArray);
        }

//        //Classify Artifact Color
//        if(artifactPresent){
//            double accumulatedGreen = 0;
//            for(int i = 0; i < 30; i++){
//                accumulatedGreen+=detector.green();
//                Log.d("Red Reading: ", "" + i + ") " + detector.red());
//                Log.d("Green Reading: ", "" + i + ") " + detector.green());
//                Log.d("Blue Reading: ", "" + i + ") " + detector.blue());
//
//            }
//            if(accumulatedGreen / 30 > 800){
//                String[] tempArray = (String []) state.get(1);
//                tempArray[2] = "Green";
//                state.set(1, tempArray);
//            }
//            else{
//                String[] tempArray = (String []) state.get(1);
//                tempArray[2] = "Purple";
//                state.set(1, tempArray);
//            }
//        }
    }

    //Open/Close Sorter Door Based on Input
    public void door(String input){
        if(input.equals("Open")){
            state.set(2, "Opened");
        }
        else{
            state.set(2, "Closed");
        }
    }
    public void registerShot(){
        String[] tempArray = (String[]) state.get(1);
        tempArray[0] = "";
        state.set(1, tempArray);
    }

    public boolean needsReset(){
        return (double) state.get(0) > (1 - .07 * 3);
    }

    public boolean isEmpty(){
        String[] tempArray = (String[]) state.get(1);
        return tempArray[0].equals("") && tempArray[1].equals("") && tempArray[2].equals("");
    }

    public boolean isFull(){
        String[] tempArray = (String[]) state.get(1);
        return !tempArray[0].equals("") && !tempArray[1].equals("") && !tempArray[2].equals("");
    }

    public boolean hasGreen(){
        String[] tempArray = (String[]) state.get(1);
        return tempArray[0].equals("Green") || tempArray[1].equals("Green") || tempArray[2].equals("Green");
    }

    public boolean hasPurple(){
        String[] tempArray = (String[]) state.get(1);
        return tempArray[0].equals("Purple") || tempArray[1].equals("Purple") || tempArray[2].equals("Purple");
    }

    public String[] getArtifactStack(){
        return (String[]) state.get(1);
    }

    //Access sorterServo position
    public double getPosition(){
        return sorterServo.getPosition();
    }

    public boolean hasDoorOpened(){
        if (doorServo.getPosition() < 0.2) {
            // door has opened enough
            return true;
        }
        else {
            // door hasn't opened enough
            return false;
        }
    }

    public boolean hasDoorClosed(){
        if (doorServo.getPosition() > 0.55) {
            // door has closed enough
            return true;
        }
        else {
            // door hasn't closed enough
            return false;
        }
    }

    //Enforce State
    public void update(){
        sorterServo.setPosition((double) state.get(0));

        if(state.get(2).equals("Opened")){
            doorServo.setPosition(0.15);
        }
        else{
            doorServo.setPosition(.6);
        }
    }
}
