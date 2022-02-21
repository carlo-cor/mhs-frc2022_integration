package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput; 
import edu.wpi.first.wpilibj.Timer;

public class Intake {
    private MotorController intakeMotor; // the intake motor
    private DigitalInput holdSwitch; // limit switch
    private double outtakeSpeed = 0.5; // test the speed out
    private double intakeSpeed = 0.4; // the speed of the intake motor
    private Timer timer; //timer for intake
    private double holdDelay = 0.55; // test the delay time

    public Intake(MotorController newIntakeMotor, DigitalInput newHoldSwitch, Timer newTimer){
        intakeMotor = newIntakeMotor;
        holdSwitch = newHoldSwitch;
        timer = newTimer;
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

    public void setIntakeSpeed(double speed){ //method for the motor intaking
        intakeMotor.set(-speed);
    }

    public void setOuttakeSpeed(double speed){ //output or outtaking
        intakeMotor.set(speed);
    }

    public void stopMotor(){ // stops motor
        intakeMotor.set(0);
    }

    private void intaking(){ //intakes cargo and holds it when switch is being triggered
        if (!cargoCheck()){
            timer.start();
            if (timer.get() > holdDelay){
                timer.stop();
                stopMotor();
            }
            else{
                setIntakeSpeed(intakeSpeed);
            }
        }
        else{
            timer.reset();
            timer.stop();
            setIntakeSpeed(intakeSpeed);
        }
    }

    private void feeding(){ // feeds the ball into the shooter
        if (!cargoCheck()){
            setIntakeSpeed(intakeSpeed);
        }
        else{
            stopMotor();
        }
    }

    public void displayMethod(){
        SmartDashboard.putBoolean("Limit switch", cargoCheck()); // displays if the limit switch is being triggered
        SmartDashboard.putString("Mode", mode.toString()); // displays the current state of the intake
        SmartDashboard.putNumber("Timer", timer.get());
    }

    public void run(){
        displayMethod();
        switch(mode){
            case INTAKING: // sets intake to intaking stage
            intaking();
            break; 

            case OUTTAKING: // sets intake to outtaking stage
            setOuttakeSpeed(outtakeSpeed);
            break;

            case FEEDING: // sets intake to feeding stage
            feeding();
            break;

            case OVERRIDE: // overrides sensor
            setIntakeSpeed(intakeSpeed);
            break;

            case STOP: // sets motor to stop stage
            stopMotor();
            break;

            case TESTING: //sets to testing stage
            break;
        }
    }
}