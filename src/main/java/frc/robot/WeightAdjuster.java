package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WeightAdjuster {

    // ANGLING FOR HIGHER BARS REQUIRES FOR WEIGHTSHIFTER TO GO INWARD. BEFORE SHIFTING WEIGHT (FOR THE ANGLE), MAKE SURE ELEV IS FULLY EXTENDED
    // PULLING OFF FROM LOWER BARS REQUIRES FOR WEIGHTSHIFTER TO GO OUTWARD

    //ASSUME GOING UP IS POSITIVE

    private MotorController weightAdjuster;
    private SingleChannelEncoder weightEncoder;

    public double weightSpeedUp = 0.20;               //speed going up
    public double weightSpeedDown = -0.20;            //speed going down

    private double weightMaxUp = 100;            //encoder count for the most up it can be
    private double weightMaxDown = -100;          //encoder count for the farthest down it can be

    public WeightAdjuster(MotorController WeightShifter, SingleChannelEncoder shifterEnc){
        weightAdjuster = WeightShifter;
        weightEncoder = shifterEnc;
    }

    private enum States{
        UP, DOWN, HOME, TESTING, STOP;
    }

    public States weightShifterState = States.STOP;

    public void setWeightHome(){
        weightShifterState = States.HOME;
    }

    public void setWeightUp(){
        weightShifterState = States.UP;
    }

    public void setWeightDown(){
        weightShifterState = States.DOWN;
    }

    public void setWeightStop(){
        weightShifterState = States.STOP;
    }

    public void setWeightTest(){
        weightShifterState = States.TESTING;
    }

    //BOOLS
    public boolean beforeUpLim() {
        return weightEncoder.get() <= weightMaxUp; 
    }

    public boolean beforeDownLim() {
        return weightEncoder.get() >= weightMaxDown; 
    }

    public boolean beforeHomeLim() {
        return weightEncoder.get() > 1; 
    }

    public boolean afterHomeLim() {
        return weightEncoder.get() < -1; 
    }

    //METHODS
    private void weightUp(){        
        if(beforeUpLim()){     //when the encoder is less than the encoder limit (max UP position), go up
            weightAdjuster.set(weightSpeedUp);
        }

        else{           
            weightAdjuster.set(0);
        }
    }

    private void weightDown(){
        if(beforeDownLim()){       //when the encoder is more than the encoder limit (max DOWN position), go down
        weightAdjuster.set(weightSpeedDown);
        }

        else{
            weightAdjuster.set(0);
        }
    }

    private void weightHome(){
        if(afterHomeLim()){      //when the weight is down, adjust the weight inwards
            weightAdjuster.set(weightSpeedUp);
        }

        else if(beforeHomeLim()){      //when the weight is up, adjust the weight outwards
            weightAdjuster.set(weightSpeedDown);
        }

        else{
            weightAdjuster.set(0);
        }
    }

    public void weightReset() {
        weightEncoder.reset();
    }

    public void manualUp() {
        weightAdjuster.set(weightSpeedUp); 
    }

    public void manualDown() {
        weightAdjuster.set(weightSpeedDown); 
    }

    private void stop(){
        weightAdjuster.set(0);
    }

    public void manualWeight(double speed){     //manually control the weight adjuster
        weightAdjuster.set(speed);
    }

    public void testing (){
        //EMPTY CODE FOR TESTING
    }

    public void run(){

        SmartDashboard.putNumber("TALON ENCODER", weightEncoder.get());
        SmartDashboard.putNumber("SPEED", weightAdjuster.get()); 
        SmartDashboard.putBoolean("BEFORE UP LIM", beforeUpLim());
        SmartDashboard.putBoolean("BEFORE DOWN LIM", beforeDownLim());
        SmartDashboard.putBoolean("BEFORE HOME LIM", beforeHomeLim()); 
        SmartDashboard.putBoolean("AFTER HOME LIM", afterHomeLim()); 
        SmartDashboard.putString("WEIGHT STATE", weightShifterState.toString()); 
        
        switch(weightShifterState){

            case UP:
            weightUp();
            break;

            case DOWN:
            weightDown();
            break;

            case HOME:
            weightHome();
            break;

            case TESTING:
            testing();
            break;

            case STOP:
            stop();
            break;

        }

    }
}