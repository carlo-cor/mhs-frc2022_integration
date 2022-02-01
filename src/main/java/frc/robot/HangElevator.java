package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//FALCON 500 (1)

public class HangElevator{
   
    //MOTORS
    private MotorController elevatorMotor;

    //ENCODERS
    private TalonFXSensorCollection elevatorEncoder;

    //SENSORS
    private DigitalInput limitTop;          //4000                    
    private DigitalInput limitBot;          //-1200

    //VALUES
    private double closeTopLimit = 0.50* 2094;                   //encoder value, when close to the top limit switch, start to slow down         
    private double closeBotLimit = 600;                   // -- bottom switch --

    //CONSTRUCTOR
    public HangElevator(MotorController elevMotor, DigitalInput limitSwitchTop, DigitalInput limitSwitchBottom, TalonFXSensorCollection elevEncoder){
        elevatorMotor = elevMotor;
        limitTop = limitSwitchTop;
        limitBot = limitSwitchBottom;
        elevatorEncoder = elevEncoder;
    }
    
    //ENUMERATIONS/STATES
    private enum elevatorState{
        EXTEND, RETRACT, STOP, TESTING;
    }
    
    private elevatorState runState = elevatorState.STOP;        

    public void elevatorExtend(){
        runState = elevatorState.EXTEND;
    }

    public void elevatorRetract(){
        runState = elevatorState.RETRACT;
    }

    public void elevatorStop(){
        runState = elevatorState.STOP;
    }

    public void elevatorTest(){
        runState = elevatorState.TESTING;
    }

    //STOP
    private void stop(){
        elevatorMotor.set(0);
    }

    //TESTING
    public void test(double JoystickY){
        elevatorMotor.set(JoystickY);
    }

    public void encoderReset(){
        elevatorEncoder.setIntegratedSensorPosition(0, 0);
    }
    
    //EXTEND
    private void extend(){
        if(limitTop.get()){                                                            //if not at top limit
            if(elevatorEncoder.getIntegratedSensorPosition() < closeTopLimit){              //and not close to limit
                elevatorMotor.set(0.40);                                                          //extend fast
            }
            else{                                                                           //if close to limit
                elevatorMotor.set(0.30);                                                          //extend slow
            }
        }
        else{                                                                           //until at top limit
            elevatorStop();                                                             //stop extension
        }
    }

    //RETRACT
    private void retract(){
        if(limitBot.get()){
            if(elevatorEncoder.getIntegratedSensorPosition() > closeBotLimit){
                elevatorMotor.set(-0.40);
            }
            else{
                elevatorMotor.set(-0.30);
            }
        }
        else{
            elevatorMotor.set(0);
            elevatorEncoder.setIntegratedSensorPosition(0, 0);

        }
    }

    //RUN
    public void run(){
        SmartDashboard.putNumber("ElevatorEncoder:", elevatorEncoder.getIntegratedSensorPosition());
        SmartDashboard.putBoolean("Elevator Top Limit:", !limitTop.get());
        SmartDashboard.putBoolean("Elevator Bottom Limit:", !limitBot.get());
        SmartDashboard.putNumber("Elevator Arm Speed:", elevatorMotor.get());
        SmartDashboard.putString("Elevator Run State:", runState.toString());
        switch(runState){
            
            case STOP:
            stop();
            break;

            case EXTEND:
            extend();
            break;

            case RETRACT:
            retract();
            break;
            
            case TESTING:
            break;

            default:
            stop();
            break;
        }   
    }   
}