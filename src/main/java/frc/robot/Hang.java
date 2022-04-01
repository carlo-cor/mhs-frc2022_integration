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

    //INTAKE
    private Intake intake;

    //PID
    private PIDController pivotPID;

    // ADJUSTER
    private WeightAdjuster weightAdjuster; 

    //COUNTERS AND OTHER VARIABLES
    private int setUpMidCount = 0; 
    private int setUpHighCount = 0;
    private int setUpHighGrabCount = 0;
    private Timer timer;

    //ELEVATOR SPEEDS

    /////////////////////////////////////////////
    //                                         //
    //              CONSTRUCTOR                //
    //                                         //
    /////////////////////////////////////////////

    public Hang (HangPivot Pivot, HangElevator Elevator, Intake Intake, WeightAdjuster newWeightAdjuster, TalonEncoder pivotEnc){
        elevator = Elevator;
        pivot = Pivot;
        intake = Intake;
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
        MIDHANG, HIGHHANG, HIGHHANGGRAB, RESETPOS, TESTING, NOTHING
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

    public void setResetPos(){
        hangMode = hangStates.RESETPOS;
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

    private void resetPosition(){   //RESETS ENCODER WHEN PIVOT AND ELEVATOR TOUCHES THEIR RESPECTIVE LIMITS
        if(pivot.frontLimitTouched() && elevator.bottomLimitTouched()){
            pivot.setStop();
            pivot.resetEnc();
            elevator.setElevatorStop();
            elevator.encoderReset();
        }

        else{
            pivot.setPivInwardLim();
            elevator.setRetractLimFast();
        }
    }
    
    private void midHangGrab() {        //STARTING: ON THE FLOOR, BEHIND THE MID RUNG
        switch(setUpMidCount) {

            case 0:                     
            //RESET ENCODERS FOR HANG ELEVATOR, PIVOT, AND INTAKE
            if(pivot.frontLimitTouched() && elevator.bottomLimitTouched() && intake.armIsDown()){
                pivot.setStop();
                pivot.resetEnc();
                intake.setArmStopMode();
                elevator.setElevatorStop();
                elevator.encoderReset();
                weightAdjuster.resetEncoder();
                weightAdjuster.resetTimer();
                setUpMidCount++;
            }
    
            else{      
                pivot.setPivInwardLim();
                elevator.setRetractLimFast();
                intake.setExtend();                  
            }                                      
            break;

 /*           case 0:
            if(pivot.frontLimitTouched() && elevator.bottomLimitTouched() && intake.armIsDown()){
                pivot.setStop();
                pivot.resetEnc();
                intake.setArmStopMode();
                elevator.setElevatorStop();
                elevator.encoderReset();
                weightAdjuster.resetEncoder();
                weightAdjuster.resetTimer();
                setUpMidCount++;
            }

            else{
                if(pivot.frontLimitTouched()){
                    pivot.setStop();
                    pivot.resetEnc();
                }
                else{
                    pivot.setPivInwardLim();
                }

                if(elevator.bottomLimitTouched()){
                    elevator.setElevatorStop();
                    elevator.encoderReset();
                }
                else{
                    elevator.setRetractLimFast();
                }

                if(intake.armIsDown()){
                    intake.setArmStopMode();
                }

                else{
                    intake.setExtend();
                }
            }
            break; */

            case 1:                     //EXTEND ELEV AND PIVOT FOR SETUP POS.
            if(elevator.topLimitTouched() && pivot.afterOutwardEnc() && !intake.belowRetract() && !weightAdjuster.beforeDownLim()){
                pivot.setStop();
                elevator.setElevatorStop();
                intake.setArmStopMode();
                weightAdjuster.setWeightStop();
                timer.start();
                setUpMidCount++;
            }
            else{
                elevator.setExtendLimFast();
                weightAdjuster.setWeightDown();
                if(pivot.afterOutwardEnc()){
                    pivot.setStop();
                }
                else{
                    pivot.setPivOutwardLim();
                }

                if(intake.extInsidePerimeter()){
                    intake.setArmStopMode();
                }
                else{
                    intake.setRetract();
                }
            }
            break;

            case 2:                      //DELAY FOR DRIVERS TO DRIVE TO RUNG
            if (timer.get() >= 1.5) {     
                timer.stop();   //after five seconds, move on to the next case
                setUpMidCount++; 
            }
            break;  

            case 3:                         //RETRACT ELEVATOR UNTIL PIVOT IS ABOVE RUNG                                              
            if (elevator.belowPivot()) {
                elevator.setElevatorStop();  
                setUpMidCount++;
            } 
            else {
                elevator.setRetractLimFast();                                  
            }
            break; 

            case 4:                         //INWARD PIVOT UNTIL PIVOT LINES UP WITH RUNG
            if(pivot.beforeMidRange()){
                pivot.setStop();
                timer.reset();
                timer.start();
                setUpMidCount++;
            } 
            else{
                pivot.setPivInward();
            }
            break;

            case 5:
            if (timer.get() >= 0.5) {     
                timer.stop();   //after five seconds, move on to the next case
                setUpMidCount++; 
            }
            break; 

            case 6:                         //EXTEND ELEVATOR UNTIL PIVOT IS ON THE RUNG
            if(elevator.abovePivot()){            
                elevator.setElevatorStop();
                setUpMidCount++;
            }
            else{
                elevator.setExtendLimSlow();               
            }
            break;  

            case 7:                         //RESETS TIMER FOR DELAY
            timer.reset();  
            break; 
        }   
    }

    private void highHangSetup(){
        switch(setUpHighCount){         //STARTING: ELEVATOR IS SLIGHTLY ABOVE THE RUNG, PIVOT IS ON RUNG

            case 0:                                 
            //PIVOT INWARD UNTIL IT IS ANGLED BELOW THE BAR
            if(pivotEncoder.get() < 1400){
                pivotPID.reset();
                pivotPID.setPID(0.0005, 0.00011, 0.00004);      //SETS PID VALUES
                pivot.setTesting();
                setUpHighCount++;
            }
            else{
                pivot.setPivInwardLim();        //USED TO BE PIVINWARD
            }
            break;

            case 1:                                 
            //EXTEND UNTIL ELEVATOR IS BEHIND NEXT RUNG, PIVOT SHOULD ACTIVELY BE HOLDING ITSELF WITH A PID
            if (elevator.topLimitTouched()) {                                   
                elevator.setElevatorStop();        
                pivot.setStop();                               
                setUpHighCount++; 
            } 
            else {
                elevator.setExtendLimFast();
                double pivotOutput = pivotPID.calculate(pivotEncoder.get(), 900);
                pivot.manualPivot(pivotOutput);
            }
            break; 

            case 2: 
            //RESET NEXT METHOD TO BE ABLE TO DO SEQUENTIALLY
            setUpHighGrabCount = 0; 
            break; 

            }
        }


    private void highHangGrab(){        //STARTING: ELEVATOR SLIGHTLY RETRACTED AND ON HIGHER BAR, PIVOT STILL ON LOWER
        switch(setUpHighGrabCount){
            case 0:                                     
            //RETRACT FULLY SO PIVOT CAN LET GO
            if(elevator.bottomLimitTouched()){
                elevator.setElevatorStop();
                //pivot.setStop();        
                setUpHighGrabCount++;
            }
            else{
                /*if(elevator.startPivotingInward()){
                    if(pivot.frontLimitTouched()){
                        pivot.setStop();
                    }
                    else{
                        pivot.setTesting();
                        pivot.manualPivot(-0.1);
                    }
                }*/
                elevator.setRetractLimFast();
            }
            break;

            case 1:                                         
            //PIVOT OUTWARD UNTIL THE PIVOT IS OFF THE LOWER RUNG
            if(pivot.pivotUnhooked()){ 
                pivot.setStop();
                setUpHighGrabCount++;
            }

            else{
                pivot.setPivOutwardLim();
            }
            break;

            case 2:
            //EXTEND UNTIL PIVOT CAN FIT UNDER RUNG
            if(elevator.abovePivotHigh()){        
                elevator.setElevatorStop();
                setUpHighGrabCount++;
            }
            else{
                elevator.setExtendLimSlow();
            }
            break;

            case 3:                             
            //PIVOT OUTWARD UNTIL BEHIND RUNG
            if(pivot.afterOutwardEnc()){        
                pivot.setStop();
                setUpHighGrabCount++;
            }

            else{
                pivot.setPivOutwardLim();
            }
            break;

            case 4: 
            //RESET COUNTER FOR PREVIOUS METHOD
            setUpHighCount = 0; 
            setUpHighGrabCount++;
            break; 

            case 5:                                     
            //RETRACT ELEVATOR UNTIL BOTTOM LIMIT, ALLOWING DRIVER TO PUT PIVOT ON RUNG
            if(elevator.bottomLimitTouched()){
                elevator.setElevatorStop();
            }

            else{
                elevator.setRetractLimFast();
            }
            break;
                                                            
        } 
    }

    private void stop(){        //STOPS ELEVATOR AND PIVOT
        elevator.setElevatorStop();
        pivot.setStop();
        weightAdjuster.setWeightStop(); 
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
        intake.intakeRun();
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

            case RESETPOS:
            resetPosition();
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