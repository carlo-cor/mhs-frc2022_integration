package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;

public class Hang {
    //MASTER
    /////////////////////////////////////////////
    //                                         //
    //                VARIABLES                //
    //                                         //
    ///////////////////////////////////////////// 
    
    //ELEVATOR
    private HangElevator elevator;

    //PIVOT
    private HangPivot pivot;

    //WEIGHT ADJUSTER
    private WeightAdjuster weightAdjuster; 

    //COUNTERS AND OTHER VARIABLES
    private int setUpMidCount = 0; 
    private int setUpHighCount = 0;
    private int setUpHighGrabCount = 0;
    private Timer timer;

    /////////////////////////////////////////////
    //                                         //
    //              CONSTRUCTOR                //
    //                                         //
    /////////////////////////////////////////////

    public Hang (HangPivot Pivot, HangElevator Elevator, WeightAdjuster newWeightAdjuster){
        elevator = Elevator;
        pivot = Pivot;
        timer = new Timer();
        weightAdjuster = newWeightAdjuster; 
    }

    /////////////////////////////////////////////
    //                                         //
    //               ENUMERATIONS              //
    //                                         //
    /////////////////////////////////////////////

    //PIVOT ENUMERATIONS
    private enum hangStates{
        MIDHANG, HIGHHANG, HIGHHANGGRAB, TESTING, NOTHING
    }
    
    private hangStates hangMode = hangStates.NOTHING; 

    public void setMidHang() {
        hangMode = hangStates.MIDHANG; 
    }

    public void setHighHang() {
        hangMode = hangStates.HIGHHANG; 
    }

    public void setHighHangGrab(){
        hangMode = hangStates.HIGHHANGGRAB;
    }

    public void setTesting() {
        hangMode = hangStates.TESTING; 
    }

    public void setNothing() {
        hangMode = hangStates.NOTHING; 
    }

    /////////////////////////////////////////////
    //                                         //
    //                 METHODS                 //
    //                                         //
    /////////////////////////////////////////////

    public void resetCounters(){
        setUpMidCount = 0;
        setUpHighCount = 0; 
        setUpHighGrabCount = 0;
        timer.reset();
    }    

    private void testing(){}

    public void elevatorWeightUp() {
        if (elevator.topLimitTouched()){        //if the top limit is touched, stop elevator
            elevator.setElevatorStop();                                          
            weightAdjuster.setWeightUp();
        } 

        else {
            elevator.setElevatorExtend();       //extend elevator otherwise
        }
    }

    public void elevatorWeightDown(){
        if (weightAdjuster.beforeDownLim()) {       //if the weight is up, set it down until its in the right "down" position
        weightAdjuster.setWeightDown();
        } else {
        weightAdjuster.setWeightStop();     //once the weight is down, retract elevator
        elevator.setElevatorRetract();
        }
    }

    private void midHangGrab() {
        switch(setUpMidCount) {

            case 0:
            //resets encoder
            pivot.resetEnc();
            elevator.encoderReset();
            setUpMidCount++;
            break;

            case 1: 
            //pivot outward (to set up angle for mid rung grab)
            if ((pivot.backLimitTouched() || pivot.afterOutwardEnc())) {      //if the back limit of pivot is touched OR back enc. limit is reached, STOP
                pivot.setStop();
                setUpMidCount++; 
            } else {                                                            //else, pivot outward
                pivot.setPivOutward(); 
            }
            break; 

            case 2: 
            //elevator extend (all the way to the top)
            if (elevator.topLimitTouched()) {      //if the top limit of elevator is touched, STOP
                elevator.setElevatorStop();
                setUpMidCount++; 
            } 
            
            else {
                elevator.setElevatorExtend();
            }
            break; 

            case 3: 
            //add a delay in between, to allow drivers to choose when to retract
            timer.start(); 
            if (timer.get() >= 5) {     //if timer > 5, stop the timer
                timer.stop(); 
                setUpMidCount++; 
            }
            break;  

            case 4: 
            // elevator retract (pulls all the way up)
            timer.reset(); 
            if (elevator.bottomLimitTouched()) {   // if bottom limit is touched
                elevator.setElevatorStop();   
                setUpMidCount++;                                           // stop
            } 
            else {
                elevator.setElevatorRetract();                                  // retract at normal speed
            }
            break; 

            case 5: 
            // pivot to mid (to place pivot hook above mid rung)
            if(pivot.beforeMiddleEnc()){       //if middle encounter count is reached, stop
                pivot.setStop();       
                setUpMidCount++;
            }
            else{
                pivot.setPivInward();       //else pivot inward
            }
            break; 

            case 6:
            // elevator extends (to secure pivot hook)
            if(!elevator.belowBottomEncoderLimit()){             //if elevator reaches small enc limit, stop
                elevator.setElevatorStop();
                setUpMidCount++;
            }
            else{
                elevator.setElevatorExtendSlow();               // else extend slow
            }
            break;  

            case 7: 
            timer.reset();  //resets timer
            break; 
        }
    }

    private void highHangSetup(){
        switch(setUpHighCount){
            /*
            case 0: 
            // extend elevator (to a certain encoder extent)
            if (elevator.aboveTopEncoderLimitReached()) {  //VALUE SUBJECT TO CHANGE    // if top limit or small encoder limit isn't reached 
                elevator.setElevatorStop(); 
                setUpHighCount++;                                            // extend at a normal speed 
            } else {
                elevator.setElevatorExtend();                                             // else stop
            }
            break; 
            */

            case 0: 
            //pivot inwards 
            if (pivot.beforeInwardEnc() || pivot.frontLimitTouched()){           // if neither inward limit is reached 
                pivot.setStop();
                setUpHighCount++;                                                // pivot inward 
            }
            else{
                pivot.setPivInward();                                                    // else stop 
            }
            break; 

            case 1: 
            //elevator extend 
            if (elevator.topLimitTouched()) {                                    // if neither top limit is reached 
                elevator.setElevatorStop();                                          // extend at normal speed 
                setUpHighCount++; 
            } 
            else {
                if(!elevator.aboveTopEncoderLimit()){                                      // else if close to top limit 
                    elevator.setElevatorExtendSlow();                                 // extend slowly 
                }
                else{ 
                    elevator.setElevatorExtend();                                       //else stop
                }
            }
            break; 

            case 2: 
            setUpHighGrabCount = 0; 
            break; 

            }
        }


    private void highHangGrab(){        //FIND A WAY TO RESET THE COUNTER FOR THESE CASES (LAST RESORT: ANOTHER BUTTOn)
        switch(setUpHighGrabCount){
        /* 
            case 0:
            //retract elevator until pivotable enc is reached 
            if(elevator.pivotableEncoderReached()){  // if elevator enc is higher than pivotable enc stop pivot and retract slow
                pivot.setStop();
                setUpHighGrabCount++; 
            }
            else {
                elevator.setElevatorRetractSlow();
            }
            break; 
            case 1: 
            // retract and pivot outward 
            if(pivot.outwardEncReached() && elevator.bottomLimitTouched()){         //if outward enc is reached AND bottom limit is touched
                pivot.setStop();                                                    //set pivot and elevator stop
                elevator.setElevatorStop();
                elevator.encoderReset();
                setUpHighGrabCount++;
            }
            else if(!pivot.outwardEncReached()){                                    //else if pivot outward enc isn't reached 
                pivot.setPivOutward();                                              //pivot outward 
            }
            else if(!elevator.bottomLimitTouched()){                                 
                if(!elevator.belowBottomEncoderLimit()){                             //else if bottom limit isn't touched and bottom enc limit isn't reached
                    elevator.setElevatorRetract();                                  //retract elevator at normal speed
                }
                else{                                                               //else if bottom limit isn't touched and bottom enc limit is reached
                    elevator.setElevatorRetractSlow();                              //retract slow
                }
            }
            // end position should be: elevator on high rung, pivot fully outwards not on rung 
            break;
            */

            case 0:
            if(elevator.bottomLimitTouched()){      //retract elevator until the bottom limit has been touched
                elevator.setElevatorStop();
                setUpHighGrabCount++;
            }

            else{
                elevatorWeightDown();
            }
            break;

            case 1:
            if(!pivot.beforeMiddleEnc()){      //pivot outward until the pivot hook is nearby the rung (not on rung) 
                pivot.setStop();
                setUpHighGrabCount++;
            }

            else{
                pivot.setPivOutward();
            }
            break;

            case 2:
            if(!elevator.belowBottomEncoderLimit()){        //extend elevator until the bottom encoder limit 
                elevator.setElevatorStop();
                setUpHighGrabCount++;
            }

            else{
                elevator.setElevatorExtend();
            }
            break;

            case 3:
            if(pivot.afterOutwardEnc()){        //pivot outward to allow the hook to go under the bar
                pivot.setStop();
                setUpHighGrabCount++;
            }

            else{
                pivot.setPivOutward();
            }
            break;

            case 4:
            if(elevator.bottomLimitTouched()){      //retract elevator all the way
                elevator.setElevatorStop();
                setUpHighGrabCount++;
            }

            else{
                elevator.setElevatorRetract();
            }
            break;

            case 5:
            timer.start();
            setUpHighGrabCount++;
            break;

            case 6:                     //DELAY WAS ADDED FOR TESTING PURPOSES
            if(timer.get() > 3){
                timer.stop();
                setUpHighGrabCount++;
            }
            break;

            case 7: 
            // pivot to mid 
            if (pivot.beforeMiddleEnc()) {         //if middle enc is reached 
                pivot.setStop();                    //stop pivot
                setUpHighGrabCount++;
            }
            else {
                pivot.setPivInward();               //else pivot inward 
            }
            break;

            case 8:     
            if(!elevator.belowBottomEncoderLimit()){    //if top enc limit(small extend limit) is reached 
                elevator.setElevatorStop();           //stop elevator
                setUpHighGrabCount++;
            }
            else{
                elevator.setElevatorExtendSlow();     //else extend elevator slow
            }
            break; 

            case 9:
            if(weightAdjuster.afterHomeLim() || weightAdjuster.beforeHomeLim()){
                weightAdjuster.setWeightHome();
            } 

            else{
                weightAdjuster.setWeightStop();
                setUpHighGrabCount++;
            }
            break;
           
            case 10:
            timer.reset();
            setUpHighGrabCount++; 
            break;

            case 11: 
            setUpHighCount = 0; 
            break; 
        } 
    }

    private void stop(){        //STOPS ELEVATOR AND PIVOT
        elevator.setElevatorStop();
        pivot.setStop();
    }

    /////////////////////////////////////////////
    //                                         //
    //                   RUN                   //
    //                                         //
    /////////////////////////////////////////////

    
    public void run(){
        //SMART DASHBOARD DISPLAYS
        SmartDashboard.putNumber("MID HANG COUNTER", setUpMidCount); 
        SmartDashboard.putNumber("HIGH HANG COUNTER", setUpHighCount);
        SmartDashboard.putNumber("HIGH HANG GRAB COUNTER", setUpHighGrabCount);
        SmartDashboard.putString("HANG STATE", hangMode.toString());
        SmartDashboard.putNumber("TIMER", timer.get()); 
        SmartDashboard.putBoolean("BELOW BOTTOM ENCODER:", elevator.belowBottomEncoderLimit());
        SmartDashboard.putBoolean("ABOVE TOP ENCODER:", elevator.aboveTopEncoderLimit());

        switch(hangMode){
            case MIDHANG:
            midHangGrab();
            break;

            case HIGHHANG:
            highHangSetup();
            break;

            case HIGHHANGGRAB:
            highHangGrab();
            break;

            case TESTING:
            testing();
            break;

            case NOTHING:
            stop();
            break;

        }

        weightAdjuster.run();
        pivot.run(); 
        elevator.run();

    }
}