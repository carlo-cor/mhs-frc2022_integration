package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//FALCON 500 (1)

public class HangElevator{
   
    //MOTORS
    private MotorController elevatorMotor;
    //775 pro 

    //ENCODERS
    private TalonEncoder elevatorEncoder;

    //SENSORS
    private DigitalInput limitTop;          //4000                    
    private DigitalInput limitBot;          //-1200

    //VALUES
    private double closeTopLimit = 0.50* 2094;                  //close to top limit switch enc. value         
    private double closeBotLimit = 600;                         //close to bottom limit switch enc. value
    private double pivotableEnc = 1600;                           //encoder that needs to be reached for the pivot to come off previous rung
    private double extendSpeed = 0.40;                          //counter-clockwise to extend (-speed)
    private double slowExtendSpeed = 0.3;
    private double retractSpeed = -0.40;                         //clockwise to retract (+speed)
    private double slowRetractSpeed = -0.3;
    

    //CONSTRUCTOR
    public HangElevator(MotorController elevMotor, DigitalInput limitSwitchTop, DigitalInput limitSwitchBottom, TalonEncoder elevEncoder){
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
        return !limitTop.get();
    }

    public boolean bottomLimitTouched(){                                                    //return true if bottom limit switch is pressed
        return !limitBot.get(); 
    }
    
    public boolean aboveTopEncoderLimit(){                                                //return true if past top encoder check
        return elevatorEncoder.get() > closeTopLimit;
    }
    
    public boolean belowBottomEncoderLimit(){                                                //return true if past bottom encoder check
        return elevatorEncoder.get() < closeBotLimit;
    }

    public boolean pivotableEncoderReached(){                                               //return true if retracted enough for pivot to come off
        return elevatorEncoder.get() > pivotableEnc;
    }

    //STOP
    public void stop(){                                                                     //stop elevator motor
        elevatorMotor.set(0);
    }

    //TESTING
    public void testing(){}

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
        if(!topLimitTouched() || !bottomLimitTouched())
        elevatorMotor.set(speed);
    }

    public void encoderReset(){                                                             //reset elevator encoder value
        elevatorEncoder.reset();
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
            elevatorEncoder.reset();                              //reset encoder (bottom limit should be 0 position)
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
        SmartDashboard.putNumber("ElevatorEncoder:", elevatorEncoder.get());
        SmartDashboard.putBoolean("Elevator Top Limit:", limitTop.get());
        SmartDashboard.putBoolean("Elevator Bottom Limit:", limitBot.get());
        SmartDashboard.putNumber("Elevator Arm Speed:", elevatorMotor.get());
        SmartDashboard.putString("Elevator Run State:", runState.toString());
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
            testing();
            break;

            default:
            stop();
            break;
        }
        
        
    }
    
}