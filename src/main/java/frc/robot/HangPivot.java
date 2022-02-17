package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;

public class HangPivot {
    
    /////////////////////////////////////////////
    //                                         //
    //                VARIABLES                //
    //                                         //
    /////////////////////////////////////////////

    //  MOTOR (775) & VERSA-PLANETARY ENCODER  //
    //private MotorController hangPivot;
    //private MotorController hangPivotTwo;   //OLD ROBOT TEST
    private MotorControllerGroup hangPivot;
    private RelativeEncoder pivotEncoder;       

    //  LIMIT SWITCHES  //
    private DigitalInput frontSwitch;   
    private DigitalInput backSwitch;

    //  GYRO (NAVX)  //
    private AHRS navX;

    //  VARIABLES [SUBJECT TO CHANGE]  //
    private final double inwardPivotPos = 60.0;      
    private final double outwardPivotPos = 100.0;
    private final double midPivotPos = 80.0; 
    private final double inwardPivotSpeed = 0.20;
    private final double outwardPivotSpeed = -0.20;
    private final double grabbingHighPivotPos = 1400.0; 

    
    /////////////////////////////////////////////
    //                                         //
    //              CONSTRUCTOR                //
    //                                         //
    /////////////////////////////////////////////
    
    public HangPivot (MotorController pivotMotor, MotorController pivotMotorTwo, RelativeEncoder hangPivotEncoder, AHRS gyro, DigitalInput frontLimitSwitch, DigitalInput backLimitSwitch){  
        //hangPivot = pivotMotor;
        //hangPivotTwo = pivotMotorTwo;
        hangPivot = new MotorControllerGroup(pivotMotor, pivotMotorTwo);
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
        return Math.abs(pivotEncoder.getPosition()) > outwardPivotPos;
    }

    public boolean inwardEncReached(){       //RETURNS TRUE IF POSITION IS LESS THAN PIVOT
        return Math.abs(pivotEncoder.getPosition()) < inwardPivotPos;
    }

    public boolean isGrabbingHigh(){      //CHECKS IF PIVOT ENCODER REACHED HIGH BAR
        return pivotEncoder.getPosition() < grabbingHighPivotPos; 
    }

    public boolean middleEncReached() {     //CHECKS IF PIVOT IS PERPENDICULAR TO FLOOR
        return Math.abs(pivotEncoder.getPosition()) < midPivotPos; 
    }

    /////////////////////////////////////////////
    //                                         //
    //                 METHODS                 //
    //                                         //
    /////////////////////////////////////////////

    public void resetEnc(){     //RESETS ENCODERS FOR THE PIVOT MOTOR
        pivotEncoder.setPosition(0);
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
        SmartDashboard.putBoolean(" PIVOT BACK LIMIT", backSwitch.get());
        SmartDashboard.putBoolean("PIVOT FRONT LIMIT", frontSwitch.get());
        SmartDashboard.putBoolean("PIVOT MID LIMIT", middleEncReached()); 
        SmartDashboard.putNumber("PIVOT ENCODER", pivotEncoder.getPosition());
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