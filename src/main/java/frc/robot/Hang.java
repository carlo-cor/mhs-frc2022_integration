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

    public Hang (HangPivot Pivot, HangElevator Elevator){
        elevator = Elevator;
        pivot = Pivot;
        timer = new Timer();
    }

    /////////////////////////////////////////////
    //                                         //
    //               ENUMERATIONS              //
    //                                         //
    /////////////////////////////////////////////

    //PIVOT ENUMERATIONS
    private enum hangStates{
        MIDHANG, HIGHHANG, HIGHHANGGRAB, PIVOTMANUAL, ELEVATORMANUAL, TESTING, NOTHING
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

    public void setPivotManual() {
        hangMode = hangStates.PIVOTMANUAL; 
    }

    public void setElevatorManual() {
        hangMode = hangStates.ELEVATORMANUAL; 
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
    }    

    private void testing(){}

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
            if ((pivot.backLimitTouched() || pivot.outwardEncReached())) {      //if the back limit of pivot is touched OR back enc. limit is reached, STOP
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
            } else {
                if (!elevator.aboveTopEncoderLimitReached()) {                                      //else if top encoder isnt reached, extend at normal rate
                    elevator.setElevatorExtend();
                } else {
                    elevator.setElevatorExtendSlow();                                         // else extend slow
                }
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
                if(!elevator.belowBottomEncoderLimitReached()) {                                    // else if close to bottom limit 
                    elevator.setElevatorRetract();                                  // retract at normal speed
                } else {
                    elevator.setElevatorRetractSlow();                                      // else retract slowly 
                }
            }
            break; 

            case 5: 
            // pivot to mid (to place pivot hook above mid rung)
            if(pivot.middleEncReached()){       //if middle encounter count is reached, stop
                pivot.setStop();       
                setUpMidCount++;
            }
            else{
                pivot.setPivInward();       //else pivot inward
            }
            break; 

            //QUESTIONABLE IF THE DRIVERS WANT IT, BUT WE HAVE TO TALK WITH THEM :)
            case 6:
            // elevator extends (to secure pivot hook)
            if(!elevator.belowBottomEncoderLimitReached()){             //if elevator reaches small enc limit, stop
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
            case 0: 
            // extend elevator (to a certain encoder extent)
            if (elevator.aboveTopEncoderLimitReached()) {  //VALUE SUBJECT TO CHANGE    // if top limit or small encoder limit isn't reached 
                elevator.setElevatorStop(); 
                setUpHighCount++;                                            // extend at a normal speed 
            } else {
                elevator.setElevatorExtend();                                             // else stop
            }
            break; 

            case 1: 
            //pivot inwards 
            if (pivot.inwardEncReached() || pivot.frontLimitTouched()){           // if neither inward limit is reached 
                pivot.setStop();
                setUpHighCount++;                                                // pivot inward 
            }
            else{
                pivot.setPivInward();                                                    // else stop 
            }
            break; 

            case 2: 
            //elevator extend 
            if (elevator.topLimitTouched()) {                                    // if neither top limit is reached 
                elevator.setElevatorStop();                                          // extend at normal speed 
                setUpHighCount++; 
            } 
            else {
                if(!elevator.aboveTopEncoderLimitReached()){                                      // else if close to top limit 
                    elevator.setElevatorExtendSlow();                                 // extend slowly 
                }
                else{ 
                    elevator.setElevatorExtend();                                       //else stop
                }
            }
            break; 

            case 3: 
            setUpHighGrabCount = 0; 
            break; 

            }
        }


    private void highHangGrab(){
        switch(setUpHighGrabCount){
            
            case 0:
            //retract elevator until pivotable enc is reached 
            if(elevator.pivotableEncoderReached()){  // if elevator enc is higher than pivotable enc stop pivot and retract slow
                pivot.setStop();
                elevator.setElevatorRetractSlow();
                setUpHighGrabCount++; 
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
                if(!elevator.belowBottomEncoderLimitReached()){                             //else if bottom limit isn't touched and bottom enc limit isn't reached
                    elevator.setElevatorRetract();                                  //retract elevator at normal speed
                }
                else{                                                               //else if bottom limit isn't touched and bottom enc limit is reached
                    elevator.setElevatorRetractSlow();                              //retract slow
                }
            }
            // end position should be: elevator on high rung, pivot fully outwards not on rung 
            break;

            case 2:
            timer.start();
            setUpHighGrabCount++;
            break;

            case 3:
            if(timer.get() > 3){
                timer.stop();
                setUpHighGrabCount++;
            }
            break;

            case 4: 
            // pivot to mid 
            if (pivot.middleEncReached()) {         //if middle enc is reached 
                pivot.setStop();                    //stop pivot
                setUpHighGrabCount++;
            }
            else {
                pivot.setPivInward();               //else pivot inward 
            }
            break;

            case 5:     //WOAH WOAH WOAH
            if(!elevator.belowBottomEncoderLimitReached()){    //if top enc limit(small extend limit) is reached
                elevator.setElevatorStop();           //stop elevator
                setUpHighGrabCount++;
            }
            else{
                elevator.setElevatorExtendSlow();     //else extend elevator slow
            }
            break; 

            case 6:
            timer.reset();
            setUpHighGrabCount++; 
            break;

            case 7: 
            setUpHighCount = 0; 
            break; 

        }



    }
    

    public void manualPivot(double pivSpeed){
        pivot.setTesting();     //SETS PIVOT STATE TO TESTING
        pivot.manualPivot(pivSpeed);
    }

    public void manualPivotButton(boolean buttonIn, boolean buttonOut){
        pivot.setTesting();     //SETS PIVOT STATE TO TESTING

        if(buttonIn){
            pivot.pivotInward();        //PIVOT INWARD WHEN GIVEN BUTTON IS PRESSED
        }
        else if(buttonOut){
            pivot.pivotOutward();       //PIVOT OUTWARD WHEN GIVEN BUTTON IS PRESSED
        }
        else{
            pivot.setStop();
        }
    }

    public void manualElevator(double elevSpeed){
        elevator.setElevatorTest();     //SETS PIVOT STATE TO TESTING
        elevator.manualElev(elevSpeed);
    }

    public void manualElevatorButton(boolean buttonExtend, boolean buttonRetract) {
        elevator.setElevatorTest();     //SETS ELEVATOR STATE TO TESTING
       
        if (buttonExtend) {     
            elevator.setElevatorExtend();       //EXTENDS WHEN GIVEN BUTTON IS PRESSED
        } else if (buttonRetract) {
            elevator.setElevatorRetract();      //RETRACTS WHEN GIVEN BUTTON IS PRESSED
        } else {
            elevator.setElevatorStop();     
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
        SmartDashboard.putBoolean("BELOW BOTTOM ENCODER:", elevator.belowBottomEncoderLimitReached());
        SmartDashboard.putBoolean("ABOVE TOP ENCODER:", elevator.aboveTopEncoderLimitReached());

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

            case PIVOTMANUAL:
            testing();
            break;

            case ELEVATORMANUAL:
            testing();
            break;

            case TESTING:
            testing();
            break;

            case NOTHING:
            stop();
            break;

        }

        pivot.run(); 
        elevator.run();

    }
}