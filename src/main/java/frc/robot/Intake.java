package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput; 
public class Intake {
    private MotorController intakeMotor; // the intake motor
    private DigitalInput holdSwitch; // limit switch
    private double intakeSpeed = 0.2; // the speed of the intake motor
    public Intake(MotorController newIntakeMotor, DigitalInput newHoldSwitch){
        intakeMotor = newIntakeMotor;
        holdSwitch = newHoldSwitch;
    }

    public enum state{ // states of the intake
        INTAKING, OUTTAKING, FEEDING, TESTING, OVERRIDE, STOP

    }

    public state mode = state.STOP; 
    public void setIntakeMode(){ // sets mode to intaking
        mode = state.INTAKING;
    }
    public void setOutakeMode(){ // sets mode to outtake
        mode = state.OUTTAKING;
    }
    public void setFeedingMode(){ // sets mode to feeding mode
        mode = state.FEEDING;
    }

    public void setOverrideMode(){
        mode = state.OVERRIDE;
    }

    public void setTestingMode(){ // sets mode to testing mode
        mode = state.TESTING;
    }
    public void setStopMode(){ // sets mode to stop
        mode = state.STOP;
    }
    
    public boolean cargoCheck(){ //checks the limit switch if it is being triggered or not
        return holdSwitch.get();
    }
    public void intake(double speed){ //method for the motor intaking
        intakeMotor.set(-speed);
    }
    public void output(double speed){ //output or outtaking
        intakeMotor.set(speed);
    }
    public void stopMotor(){ // stops motor
        intakeMotor.set(0);
    }
    private void motorCheckIntake(){ //intakes cargo and holds it when switch is being triggered
        if (cargoCheck()){
            stopMotor();
        }
        else{
            intake(intakeSpeed);
        }
    }
    private void feeding(){ // feeds the ball into the shooter
        if (cargoCheck()){
            intake(intakeSpeed);
        }
        else{
            stopMotor();
        }
    }
    public void displayMethod(){
        SmartDashboard.putBoolean("Limit switch", cargoCheck()); // displays if the limit switch is being triggered
        SmartDashboard.putString("Mode", mode.toString()); // displays the current state of the intake
    }
    public void run(){
        switch(mode){
            case INTAKING: // sets intake to intaking stage
            motorCheckIntake();
            break; 
            case OUTTAKING: // sets intake to outtaking stage
            output(intakeSpeed);
            break;
            case FEEDING: // sets intake to feeding stage
            feeding();
            break;

            case OVERRIDE: // overrides sensor
            intake(intakeSpeed);
            break;

            case STOP: // sets motor to stop stage
            stopMotor();
            break;
            case TESTING: //sets to testing stage
            break;
        }
    }
}