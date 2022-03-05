package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//UP IS NEGATIVE!!

//FALCON 500 (1)

public class HangElevator{
   
    //MOTORS
    private MotorController elevatorMotor;

    //ENCODERS
    private TalonFXSensorCollection elevatorEncoder;

    //SENSORS
    private DigitalInput limitTop;          //346374.0, 348279, 348412, 351491   == 348639    
    private DigitalInput limitBot;          //0

    //VALUES
    private double closeTopLimit = 0.75* 348639;                  //close to top limit switch enc. value         
    private double closeBotLimit = 0.25 * 348639;                 //close to bottom limit switch enc. value
    private double extendSpeed = 0.40;                         //counter-clockwise to extend (-speed)
    private double slowExtendSpeed = 0.20;
    private double retractSpeed = -0.40;                         //clockwise to retract (+speed)
    private double slowRetractSpeed = -0.20;
    private double pivotableEnc = 170000; 
    private double equalToPivot = 57227.6;                            //encoder count for elevator to be same height as pivot
//58593, , 55551, 55390, 59987, 56617
    //CONSTRUCTOR
    public HangElevator(MotorController elevMotor, DigitalInput limitSwitchTop, DigitalInput limitSwitchBottom, TalonFXSensorCollection elevEncoder){
        elevatorMotor = elevMotor;
        limitTop = limitSwitchTop;
        limitBot = limitSwitchBottom;
        elevatorEncoder = elevEncoder;
    }
    
    //ENUMERATIONS/STATES
    private enum elevatorState{
        EXTEND, RETRACT, EXTENDSLOW, RETRACTSLOW, STOP, TESTING;
    }
    
    private elevatorState runState = elevatorState.STOP;        //default state     

    public void setElevatorExtend(){
        runState = elevatorState.EXTEND;
    }

    public void setElevatorRetract(){
        runState = elevatorState.RETRACT;
    }

    public void setElevatorExtendSlow(){
        runState = elevatorState.EXTENDSLOW;
    }

    public void setElevatorRetractSlow(){
        runState = elevatorState.RETRACTSLOW;
    }
    public void setElevatorStop(){
        runState = elevatorState.STOP;
    }

    public void setElevatorTest(){
        runState = elevatorState.TESTING;
    }

    //CHECKS
    public boolean topLimitTouched(){                                                       //return true if top limit switch is pressed
        return limitTop.get();
    }

    public boolean bottomLimitTouched(){                                                    //return true if bottom limit switch is pressed
        return limitBot.get(); 
    }
    
    public boolean aboveTopEncoderLimit(){                                                //return true if past top encoder check
        return Math.abs(elevatorEncoder.getIntegratedSensorPosition()) > closeTopLimit;
    }
    
    public boolean belowBottomEncoderLimit(){                                                //return true if past bottom encoder check
        return Math.abs(elevatorEncoder.getIntegratedSensorPosition()) < closeBotLimit;
    }

    public boolean pivotableEncoderReached(){
        return Math.abs(elevatorEncoder.getIntegratedSensorPosition()) < pivotableEnc; 
    }

    public boolean elevatorEqualToPivot(){
        return Math.abs(elevatorEncoder.getIntegratedSensorAbsolutePosition()) == equalToPivot;
    }

    //STOP
    public void stop(){                                                                     //stop elevator motor
        elevatorMotor.set(0);
    }

    public void empty(){

    }

    //TESTING
    public void testing(double speed){
        elevatorMotor.set(speed);
    }

    //MANUALS
    private void elevExtend(){                                                               //automatically set to extend speed value
        elevatorMotor.set(extendSpeed);
    }                
     
    private void elevRetract(){                                                              //automatically set to retract speed value
        elevatorMotor.set(retractSpeed);
    }

    private void elevExtendSlow(){                                                           //automatically set to extend slow speed value
        elevatorMotor.set(slowExtendSpeed);
    }
    
    private void elevRetractSlow(){                                                          //automatically set to retract slow speed value
        elevatorMotor.set(slowRetractSpeed);
    }

    public void manualElev(double speed){                                                   //if not at either limits, move to an inputted speed
        //if(!topLimitTouched() || !bottomLimitTouched())
        elevatorMotor.set(speed);
    }

    public void encoderReset(){                                                             //reset elevator encoder value
        elevatorEncoder.setIntegratedSensorPosition(0,0);
    }
    

    //EXTEND
    public void extendToTopLimit(){
        if(topLimitTouched()){                                                              //if at top limit
            elevatorMotor.set(0);                                                           //stop extending
        }
        else{
            if(aboveTopEncoderLimit()){                                                   //not at top limit but close to
                elevatorMotor.set(slowExtendSpeed);                                         //extend slow
            }
            else{
                elevatorMotor.set(extendSpeed);                                             //if not close to top limit, extend fast
            }
        }
        }

    //RETRACT
    public void retractToBottomLimit(){
        if(bottomLimitTouched()){                                                           //if at bottom limit
            elevatorMotor.set(0);                                                           //stop retracting
            elevatorEncoder.setIntegratedSensorPosition(0, 0);                       //reset encoder (bottom limit should be 0 position)
        }
        else{
            if(belowBottomEncoderLimit()){                                                   //if not at bottom limit but close to
                elevatorMotor.set(slowRetractSpeed);
            }
            else{                                                                           //if not at or close to bottom limit
                elevatorMotor.set(retractSpeed);                                            //retract fast
            }
        }
    }

    //RUN
    public void run(){
        SmartDashboard.putNumber("ElevatorEncoder Absolute Value:", Math.abs(elevatorEncoder.getIntegratedSensorPosition()));       //print abs val of elev enc
        SmartDashboard.putNumber("ElevatorEncoder Real Value:", elevatorEncoder.getIntegratedSensorPosition());                     //actual val of elev enc
        SmartDashboard.putBoolean("Elevator Top Limit:", limitTop.get());                                                           //top limit switch
        SmartDashboard.putBoolean("Elevator Bottom Limit:", limitBot.get());                                                        //bot limit switch
        SmartDashboard.putNumber("Elevator Arm Speed:", elevatorMotor.get());                                                       //speed of the elev
        SmartDashboard.putString("Elevator Run State:", runState.toString());                                                       //enum of the elev
        SmartDashboard.putBoolean("Above Top Encoder Limit:", aboveTopEncoderLimit());
        SmartDashboard.putBoolean("Below Bottom Encoder Limit:", belowBottomEncoderLimit());
        SmartDashboard.putBoolean("Pivotable Encoder Reached", pivotableEncoderReached());
        switch(runState){
            
            case STOP:
            stop();
            break;

            case EXTEND:
            extendToTopLimit();
            break;

            case RETRACT:
            retractToBottomLimit();
            break;

            case EXTENDSLOW:
            elevExtendSlow();
            break;

            case RETRACTSLOW:
            elevRetractSlow();
            break;
            
            case TESTING:
            empty();
            break;

        }
        
    }
    
}