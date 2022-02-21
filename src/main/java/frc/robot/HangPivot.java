package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

public class HangPivot {
    
    /////////////////////////////////////////////
    //                                         //
    //                VARIABLES                //
    //                                         //
    /////////////////////////////////////////////

    //775 MOTOR
    private MotorController hangPivot; 
    private TalonEncoder pivotEncoder;       

    //  LIMIT SWITCHES  //
    private DigitalInput frontSwitch;   
    private DigitalInput backSwitch;

    //  GYRO (NAVX)  //
    private AHRS navX;

    //  VARIABLES [SUBJECT TO CHANGE]  //
    private final double inwardPivotPos = 150.0;      
    private final double outwardPivotPos = 200.0;
    private final double midPivotPos = 175.0; 
    private final double inwardPivotSpeed = 0.25;
    private final double outwardPivotSpeed = -0.25;
    //private final double grabbingHighPivotPos = 1400.0; 

    
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
        PIVOTINWARD, PIVOTOUTWARD, STOP, TESTING;
    }

    //  SETTING STATES  //
    public States pivotState = States.STOP;

    public void setPivInward(){
        pivotState = States.PIVOTINWARD;
    }

    public void setPivOutward(){
        pivotState = States.PIVOTOUTWARD;
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
    //DIRECTIONS ARE NOT FINAL
    public boolean backLimitTouched(){     //RETURNS VALUE OF BACK LIMIT SWITCH
        return !backSwitch.get();
    }

    public boolean frontLimitTouched(){    //RETURNS VALUE OF FRONT LIMIT SWITCH
        return !frontSwitch.get();
    }

    public boolean outwardEncReached(){      //RETURNS TRUE IF POSITION IS GREATER THAN PIVOT
        return Math.abs(pivotEncoder.get()) > outwardPivotPos;
    }

    public boolean inwardEncReached(){       //RETURNS TRUE IF POSITION IS LESS THAN PIVOT
        return Math.abs(pivotEncoder.get()) < inwardPivotPos;
    }
/*
    public boolean grabbingHigh(){      //CHECKS IF PIVOT ENCODER REACHED HIGH BAR
        return pivotEncoder.get() < grabbingHighPivotPos; 
    }
*/
    public boolean middleEncReached() {     //CHECKS IF PIVOT IS PERPENDICULAR TO FLOOR
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

    public void pivotOutwardLim(){    //PIVOTS OUTWARD FOR A CERTAIN AMOUNT OF ENCODER COUNTS [INWARD = TOWARDS ROBOT BASE, OUTWARD = TOWARDS ROBOT PERIMETER]
        if(backLimitTouched()){
            hangPivot.set(0);
        }

        else{
            if(!outwardEncReached()){
                hangPivot.set(outwardPivotSpeed);
            }

            else{
                hangPivot.set(0);
            }
        }
    }

    public void pivotInwardLim(){     //PIVOTS INWARD FOR A CERTAIN AMOUNT OF ENCODER COUNTS
        if(frontLimitTouched()){   //IF THE FRONT LIMIT IS NOT TOUCHED
            hangPivot.set(0);
        }

        else{
            if(!inwardEncReached()){    //IF THE PIVOT ENCODER IS LESS THAN ITS POSITION, PIVOT INWARD
                hangPivot.set(inwardPivotSpeed);
            }

            else{   //STOP IF POSITION IS REACHED
                hangPivot.set(0);
            }
        }
    }

    public void pivotOutward(){      //MANUALLY PIVOT OUTWARD
        hangPivot.set(outwardPivotSpeed);
    }

    public void pivotInward(){       //MANUALLY PIVOT INWARD
        hangPivot.set(inwardPivotSpeed);       
    }

    public void manualPivot(double pivotSpeed){     //MANUALLY MOVE THE PIVOT MOTOR WITH JOYSTICK
        hangPivot.set(pivotSpeed);
    }

    public void stopPivot(){       //STOPS HANG PIVOT
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
        SmartDashboard.putNumber("NAVX PITCH", navX.getPitch());

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

            case STOP:
            stopPivot();
            break;

        }
    }
}