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
    //MAX IS 6013.0
    private final double inwardPivotPos = 1500.0;    //VALUE FOR INWARD PIVOT (USED IN HIGH HANG SETUP OF HANG CODE)    
    private final double outwardPivotPos = 4200.0;   //VALUE FOR OUTWARD PIVOT (USED IN MID HANG SETUP OF HANG CODE)
    private final double midPivotPos = 3000.0;       //VALUE FOR PERPENDICULAR POSITION (USED TO SECURE PIVOT ON RUNGS)
    private final double inwardPivotSpeed = -0.10;       
    private final double outwardPivotSpeed = 0.10;
    
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
        PIVOTINWARD, PIVOTOUTWARD, PIVOTMID, STOP, TESTING;
    }

    //  SETTING STATES  //
    public States pivotState = States.STOP;

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
        return Math.abs(pivotEncoder.get()) > outwardPivotPos;
    }

    public boolean beforeInwardEnc(){       //RETURNS TRUE IF POSITION IS LESS THAN PIVOT
        return Math.abs(pivotEncoder.get()) < inwardPivotPos;
    }

    public boolean beforeMiddleEnc() {     //RETURNS TRUE IF PIVOT IS PERPENDICULAR TO FLOOR, COMING FROM AN OUTWARD POSITION
        return pivotEncoder.get() < midPivotPos; 
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
            if(!beforeMiddleEnc()){
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
            if(!afterOutwardEnc()){
                hangPivot.set(outwardPivotSpeed);
            }

            else{
                hangPivot.set(0);
            }
        }
    }

    private void pivotInwardLim(){     //PIVOTS INWARD FOR A CERTAIN AMOUNT OF ENCODER COUNTS
        if(frontLimitTouched()){   //IF THE FRONT LIMIT IS NOT TOUCHED
            hangPivot.set(0);
            pivotEncoder.reset();
        }

        else{
            if(!beforeInwardEnc()){    //IF THE INWARD ENC LIMIT IS NOT REACHED, PIVOT INWARD
                hangPivot.set(inwardPivotSpeed);
            }

            else{   //STOP IF POSITION IS REACHED
                hangPivot.set(0);
            }
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
        
        SmartDashboard.putNumber("MOTOR SPEED", hangPivot.get());
        SmartDashboard.putString("HANG PIVOT STATE", pivotState.toString());
        SmartDashboard.putBoolean("BACK LIMIT", backSwitch.get());
        SmartDashboard.putBoolean("FRONT LIMIT", frontSwitch.get());
        SmartDashboard.putNumber("PIVOT ENCODER", pivotEncoder.get());
        
                //  SmartDashboard.putNumber("NAVX PITCH", navX.getPitch());

        switch(pivotState){
            
            case TESTING:
            testing();
            break;

            case PIVOTOUTWARD:
            pivotOutwardLim();
            break;

            case PIVOTINWARD:
            pivotInwardLim();
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