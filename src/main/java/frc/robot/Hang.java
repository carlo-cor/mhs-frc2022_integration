package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;

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
    private TalonEncoder pivotEncoder;

    //PID
    private PIDController pivotPID;

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

    public Hang (HangPivot Pivot, HangElevator Elevator, WeightAdjuster newWeightAdjuster, TalonEncoder pivotEnc){
        elevator = Elevator;
        pivot = Pivot;
        timer = new Timer();
        weightAdjuster = newWeightAdjuster; 
        pivotPID = new PIDController(0, 0, 0);
        pivotEncoder = pivotEnc;
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
        timer.stop();
        timer.reset();
    }    

    private void testing(){}

    public void elevatorWeightUp() {
        if (elevator.topLimitTouched()){        //if the top limit is touched, stop elevator
            elevator.setElevatorStop();                                          
            weightAdjuster.setWeightUp();
        } 

        else {
            elevator.setElevatorExtendLim();       //extend elevator otherwise
        }
    }

    public void elevatorWeightDown(){
        if (weightAdjuster.beforeDownLim()) {       //if the weight is up, set it down until its in the right "down" position
        weightAdjuster.setWeightDown();
        } else {
        weightAdjuster.setWeightStop();     //once the weight is down, retract elevator
        elevator.setElevatorRetractLim();
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
            if (pivot.afterOutwardEnc()) {      //if the back limit of pivot is touched OR back enc. limit is reached, STOP
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
                timer.start();
                setUpMidCount++; 
            } 
            
            else {
                elevator.setElevatorExtendLim();
            }
            break; 

            case 3: 
            //add a delay in between, to allow drivers to choose when to retract
            if (timer.get() >= 5) {     //if timer > 5, stop the timer
                timer.stop(); 
                setUpMidCount++; 
            }
            break;  

            case 4:                                                             //PID
            if (elevator.belowPivot()) {
                elevator.setElevatorStop();  
                setUpMidCount++;
            } 
            else {
                elevator.setRetract();                                  // retract at normal speed
            }
            break; 

            case 5:     //INWARD PIVOT UNTIL PIVOT LINES UP WITH BAR
            if(pivot.beforeMidRange()){
                pivot.setStop();
                setUpMidCount++;
            } 
            else{
                pivot.setPivInward();
            }
            break;

            case 6:
            // elevator extends (to secure pivot hook)
            if(elevator.abovePivot()){             //if elevator reaches small enc limit, stop
                elevator.setElevatorStop();
                setUpMidCount++;
            }
            else{
                elevator.setExtend();               // else extend slow
            }
            break;  

            case 8: 
            timer.reset();  //resets timer
            break; 
        }   
    }

    private void highHangSetup(){
        switch(setUpHighCount){
           /* 
            case 0:                                             //EXTEND UNTIL ONLY PIVOT IS ON HOOK
            if (elevator.abovePivot()) {
                elevator.setElevatorStop(); 
                pivotPID.reset();
                setUpHighCount++;
            } else {
                elevator.setExtend();
            }
            break; 
*/
            case 0:
            if(pivotEncoder.get() < 800){
                pivotPID.reset();
                pivotPID.setPID(0.0005, 0.00011, 0.00004);
                pivot.setTesting();
                setUpHighCount++;
            }
            else{
                pivot.setPivInward();
            }
            break;

            case 1:                                 //EXTEND UNTIL ELEVATOR IS BEHIND NEXT RUNGUH
            if (elevator.topLimitTouched()) {                                   
                elevator.setElevatorStop();        
                pivot.setStop();                               
                setUpHighCount++; 
            } 
            else {
                elevator.setElevatorExtendLim();
                double pivotOutput = pivotPID.calculate(pivotEncoder.get(), 600);
                pivot.manualPivot(pivotOutput);
            }
            break; 

            case 2: 
            setUpHighGrabCount = 0; 
            break; 

            }
        }


    private void highHangGrab(){        //STARTING: ELEVATOR SLIGHTLY RETRACTED AND ON HIGHER BAR, PIVOT STILL ON LOWER
        switch(setUpHighGrabCount){
            case 0:                                     //RETRACT FULLY SO PIVOT CAN LET GO
            if(elevator.bottomLimitTouched()){
                elevator.setElevatorStop();
                setUpHighGrabCount++;
            }

            else{
                elevator.setElevatorRetractLim();
            }
            break;

            case 1:                                         //PIVOT OUTWARD SO PIVOT LETS GO
            if(pivot.pivotUnhooked()){ 
                pivot.setStop();
                setUpHighGrabCount++;
            }

            else{
                pivot.setPivOutward();
            }
            break;

            case 2:
            if(elevator.abovePivotHigh()){        //EXTEND UNTIL PIVOT CAN FIT UNDER RUNG
                elevator.setElevatorStop();
                setUpHighGrabCount++;
            }
            else{
                elevator.setExtend();;
            }
            break;

            case 3:                             //PIVOT OUTWARD UNTIL BEHIND RUNG
            if(pivot.afterOutwardEnc()){        
                pivot.setStop();
                setUpHighGrabCount++;
            }

            else{
                pivot.setPivOutward();
            }
            break;

            case 4:                                     //FULL RETRACT SO PIVOT CAN GET ON
            if(elevator.bottomLimitTouched()){
                elevator.setElevatorStop();
                setUpHighGrabCount++;
            }

            else{
                elevator.setElevatorRetractLim();
            }
            break;
                                                            //MANUAL DRIVE PIVOT TO LINE UP WITH RUNG
            case 5: 
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

        weightAdjuster.run();
        pivot.run(); 
        elevator.run();

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

    }
}