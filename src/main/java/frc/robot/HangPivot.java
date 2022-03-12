package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Thread.State;

import com.kauailabs.navx.frc.AHRS;

public class HangPivot {
    
    /////////////////////////////////////////////
    //                                         //
    //                VARIABLES                //
    //                                         //
    /////////////////////////////////////////////

    //FROM HOME POSITION, OUTWARD IS POSITIVE
    
    //775 MOTOR
    private MotorController hangPivot; 
    private TalonEncoder pivotEncoder;       

    //  LIMIT SWITCHES  //
    private DigitalInput frontSwitch;   
    private DigitalInput backSwitch;

    //  GYRO (NAVX)  //
    private AHRS navX;

    //  VARIABLES [SUBJECT TO CHANGE]  //
    //MAX IS 7124.0
    private final double inwardPivotPos = 0.20 * 7124.0;    //VALUE FOR INWARD PIVOT (USED IN HIGH HANG SETUP OF HANG CODE)    
    private final double outwardPivotPos = 0.80 * 7124.0;   //VALUE FOR OUTWARD PIVOT (USED IN MID HANG SETUP OF HANG CODE)            //7097, 7121, 7154
    private final double midPivotPos = 4027;       //VALUE FOR PERPENDICULAR POSITION (USED TO SECURE PIVOT ON RUNGS)        //OG VALUE: 3959.33
    private final double inwardPivotSpeed = -0.32;  //USED TO BE -0.30     
    private final double outwardPivotSpeed = 0.32;
    private final double unhooked = 1850;
    
    /////////////////////////////////////////////
    //                                         //
    //              CONSTRUCTOR                //
    //                                         //
    /////////////////////////////////////////////
    
    public HangPivot (MotorController pivotMotor, TalonEncoder hangPivotEncoder, AHRS gyro, DigitalInput frontLimitSwitch, DigitalInput backLimitSwitch){  
        hangPivot = pivotMotor;
        pivotEncoder = hangPivotEncoder;
        frontSwitch = frontLimitSwitch;
        backSwitch = backLimitSwitch;
        navX = gyro;
    }

    /////////////////////////////////////////////
    //                                         //
    //               ENUMERATION               //
    //                                         //
    /////////////////////////////////////////////

    private enum States{
        PIVOTINWARDLIM, PIVOTINWARD, PIVOTOUTWARDLIM, PIVOTOUTWARD, PIVOTMID, STOP, TESTING;
    }

    //  SETTING STATES  //
    public States pivotState = States.STOP;

    public void setPivInwardLim(){
        pivotState = States.PIVOTINWARDLIM;
    }

    public void setPivOutwardLim(){
        pivotState = States.PIVOTOUTWARDLIM;
    }

    public void setPivInward(){
        pivotState = States.PIVOTINWARD;
    }

    public void setPivOutward(){
        pivotState = States.PIVOTOUTWARD;
    }

    public void setPivMid(){
        pivotState = States.PIVOTMID;
    }

    public void setTesting(){
        pivotState = States.TESTING;
    }

    public void setStop(){
        pivotState = States.STOP;
    }
    
    /////////////////////////////////////////////
    //                                         //
    //                 CHECKS                  //
    //                                         //
    /////////////////////////////////////////////

    public boolean backLimitTouched(){     //RETURNS VALUE OF BACK LIMIT SWITCH
        return backSwitch.get();
    }

    public boolean frontLimitTouched(){    //RETURNS VALUE OF FRONT LIMIT SWITCH
        return frontSwitch.get();
    }

    public boolean afterOutwardEnc(){      //RETURNS TRUE IF POSITION IS GREATER THAN PIVOT
        return Math.abs(pivotEncoder.get()) > outwardPivotPos - 1000;   //-1000 added to decrease how far back it goes
    }

    public boolean beforeInwardEnc(){       //RETURNS TRUE IF POSITION IS LESS THAN PIVOT
        return Math.abs(pivotEncoder.get()) < inwardPivotPos;
    }

    public boolean inMidRange() {     //RETURNS TRUE IF PIVOT IS PERPENDICULAR TO FLOOR, COMING FROM AN OUTWARD POSITION
        return pivotEncoder.get() > (midPivotPos - 75) && pivotEncoder.get() < (midPivotPos + 75); 
    }

    public boolean beforeMidRange(){        //RETURNS TRUE IF PIVOT IS BEFORE PARALEEL TO ELEVATOR
        return pivotEncoder.get() < (midPivotPos - 75);
    }

    public boolean afterMidRange(){         //RETURNS TRUE IF PIVOT IS AFTER PARALLEL TO ELEVATOR
        return pivotEncoder.get() > (midPivotPos + 75);
    }
                                                                                            // E < W < J = 3
    public boolean pivotUnhooked(){
        return pivotEncoder.get() > unhooked && pivotEncoder.get() < midPivotPos;
    }
    public double getEncoder(){
        return pivotEncoder.get();
    }

    /////////////////////////////////////////////
    //                                         //
    //                 METHODS                 //
    //                                         //
    /////////////////////////////////////////////

    public void resetEnc(){     //RESETS ENCODERS FOR THE PIVOT MOTOR
        pivotEncoder.reset();
    }

    private void pivotInwardToMid(){
        if(frontLimitTouched()){
            hangPivot.set(0);
        }

        else{
            if(!inMidRange()){
                hangPivot.set(inwardPivotSpeed);
            }

            else{
                hangPivot.set(0);
            }
        }
    }

    private void pivotOutwardLim(){    //PIVOTS OUTWARD FOR A CERTAIN AMOUNT OF ENCODER COUNTS [INWARD = TOWARDS ROBOT BASE, OUTWARD = TOWARDS ROBOT PERIMETER]
        if(backLimitTouched()){
            hangPivot.set(0);
        }

        else{
            hangPivot.set(outwardPivotSpeed);
        }
    }

    private void pivotInwardLim(){     //PIVOTS INWARD FOR A CERTAIN AMOUNT OF ENCODER COUNTS
        if(frontLimitTouched()){   //IF THE FRONT LIMIT IS NOT TOUCHED
            hangPivot.set(0);
            pivotEncoder.reset();
        }

        else{
            hangPivot.set(inwardPivotSpeed);
        }
    }

    private void pivotOutward(){      //MANUALLY PIVOT OUTWARD
        hangPivot.set(outwardPivotSpeed);
    }

    private void pivotInward(){       //MANUALLY PIVOT INWARD
        hangPivot.set(inwardPivotSpeed);       
    }

    public void manualPivot(double pivotSpeed){     //MANUALLY MOVE THE PIVOT MOTOR WITH JOYSTICK
        hangPivot.set(pivotSpeed);
    }

    private void stopPivot(){       //STOPS HANG PIVOT
        hangPivot.set(0);
    }

    private void testing(){

    }
    

    /////////////////////////////////////////////
    //                                         //
    //                   RUN                   //
    //                                         //
    /////////////////////////////////////////////

    public void run(){      //RUN METHOD WITH SMART DASHBOARD DISPLAYS AND STATE SWITCHES
        
        SmartDashboard.putNumber("PIVOT SPEED", hangPivot.get());
        SmartDashboard.putString("HANG PIVOT STATE", pivotState.toString());
        SmartDashboard.putBoolean("BACK LIMIT", backSwitch.get());
        SmartDashboard.putBoolean("FRONT LIMIT", frontSwitch.get());
        SmartDashboard.putNumber("PIVOT ENCODER", pivotEncoder.get());
        
                //  SmartDashboard.putNumber("NAVX PITCH", navX.getPitch());

        switch(pivotState){
            
            case TESTING:
            testing();
            break;

            case PIVOTOUTWARDLIM:
            pivotOutwardLim();
            break;

            case PIVOTINWARDLIM:
            pivotInwardLim();
            break;

            case PIVOTOUTWARD:
            pivotOutward();
            break;

            case PIVOTINWARD:
            pivotInward();
            break;

            case PIVOTMID:
            pivotInwardToMid();
            break;

            case STOP:
            stopPivot();
            break;

        }
    }
}