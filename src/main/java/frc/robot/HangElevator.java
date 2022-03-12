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
    private double closeTopLimit = 0.9* 348639;                  //close to top limit switch enc. value         
    private double closeBotLimit = 0.1 * 348639;                 //close to bottom limit switch enc. value
    private double extendSpeed = 0.55;                         //counter-clockwise to extend (-speed)
    private double slowExtendSpeed = 0.20;
    private double retractSpeed = -0.55;                         //clockwise to retract (+speed)
    private double slowRetractSpeed = -0.20;
    private double pivotableEnc = 170000; 
    public double equalToPivot = 82451.3333;                            //encoder count for elevator to be same height as pivot       //82586, 82530, 82238
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
        EXTENDLIM, RETRACTLIM, EXTEND, RETRACT, STOP, TESTING;
    }
    
    private elevatorState runState = elevatorState.STOP;        //default state     

    public void setElevatorExtendLim(){
        runState = elevatorState.EXTENDLIM;
    }

    public void setElevatorRetractLim(){
        runState = elevatorState.RETRACTLIM;
    }

    public void setExtend(){          
        runState = elevatorState.EXTEND;
    }

    public void setRetract(){
        runState = elevatorState.RETRACT;
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
        return elevatorEncoder.getIntegratedSensorPosition() > closeTopLimit;
    }
    
    public boolean belowBottomEncoderLimit(){                                                //return true if past bottom encoder check
        return elevatorEncoder.getIntegratedSensorPosition() < closeBotLimit;
    }

    public boolean pivotableEncoderReached(){
        return elevatorEncoder.getIntegratedSensorPosition() < pivotableEnc; 
    }

    public boolean abovePivot(){
        return elevatorEncoder.getIntegratedSensorPosition() >= (equalToPivot + 30000);
    }

    public boolean abovePivotHigh(){
        return elevatorEncoder.getIntegratedSensorPosition() >= (equalToPivot + 25000);         //from 50,000
    }

    public boolean belowPivot(){
        return elevatorEncoder.getIntegratedSensorPosition() <= (equalToPivot - 30000);
    }

    public boolean equalToPivotRange() {
        return (elevatorEncoder.getIntegratedSensorPosition() <= (equalToPivot + 30000)) && (elevatorEncoder.getIntegratedSensorPosition() >= (equalToPivot - 15000)); 
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
    public void elevExtend(){                                                               //automatically set to extend speed value
        elevatorMotor.set(extendSpeed);
    }                
     
    public void elevRetract(){                                                              //automatically set to retract speed value
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
        SmartDashboard.putBoolean("Above Pivot", abovePivot());
        SmartDashboard.putBoolean("Below Pivot", belowPivot());

        switch(runState){
            
            case STOP:
            stop();
            break;

            case EXTENDLIM:
            extendToTopLimit();
            break;

            case RETRACTLIM:
            retractToBottomLimit();
            break;

            case EXTEND:
            elevExtend();
            break;

            case RETRACT:
            elevRetract();
            break;
            
            case TESTING:
            empty();
            break;

        }
        
    }
    
}