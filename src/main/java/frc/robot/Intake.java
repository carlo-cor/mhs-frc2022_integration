package frc.robot;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput; 
import edu.wpi.first.wpilibj.Timer;

public class Intake {

    //MOTORS:
    private MotorController intakeBar;          // motor for the inner bar of the intake
    private MotorController intakeExt;          // Brings the intake rollers down/up
    private MotorController outerRollers;       // motor for the rollers

    //SENSORS:
    private SingleChannelEncoder intakeExtEnc;  // Encoder for the intake extension
    private DigitalInput intakeSensor;          // beam break sensor
    private DigitalInput armLimit;
    private Timer timer;                        // timer for intake

    //SENSOR VALUES:
    private double holdDelay = 0.08;             // the delay time (TEST) 
    private double extEncUp = -190;              // encoder for the extension going up (TEST) //-197 //-193 //-197'
    private double extEncMidWay = -90;
    private double extEncDown = 190;            // encoder for the extension going down(TEST) //192 //199 //186
    private double insideRobotPerimeter = -140; // Encoder count for bringing the arm extension to the point where it is inside the robot perimeter

    //SPEEDS:
    private double intakeSpeed = 1;           // the speed of the intake motor
    private double feedingSpeed = 1;          // the speed of the motor when feeding
    private double outtakeSpeed = 1;          // the speed of the motor outtaking
    
    private double intakeExtSpeed = 1;        // speed for intake extension (TEST)
    private double intakeRetractSpeed = 0.55;  // speed for intake extensin when it retracts 
    private double outerRollerSpeed = 1;      // the speed of the outerRoller motor (TEST)

    //COUNTERS:
    private double extCounter = 0; 
    private int counter;

    public Intake(MotorController newIntakeBar, MotorController newIntakeExt, MotorController newOuterRollers, SingleChannelEncoder enc, DigitalInput newIntakeSensor, DigitalInput newArmLimit, Timer newTimer){
        intakeBar = newIntakeBar;
        intakeExt = newIntakeExt;
        intakeExtEnc = enc;
        outerRollers = newOuterRollers;
        intakeSensor = newIntakeSensor;
        armLimit = newArmLimit;
        timer = newTimer;
    }

    public enum intakeState{ //states of the intake
        INTAKING, OUTTAKING, FEEDING, TESTING, OVERRIDE, STOP
    }

    public intakeState intakeMode = intakeState.STOP;

    public enum armState{
        MIDWAY, RETRACT, EXTEND, TESTING, STOP
    }
    
    public armState armMode = armState.STOP;

    public void setIntakeMode(){    //sets mode to intaking
        intakeMode = intakeState.INTAKING;
    }

    public void setOutakeMode(){    //sets mode to outtake
        intakeMode = intakeState.OUTTAKING;
    }

    public void setFeedingMode(){   //sets mode to feeding mode
        intakeMode = intakeState.FEEDING;
    }

    public void setOverrideMode(){  // sets mode to override mode 
        intakeMode = intakeState.OVERRIDE;      // override intakes without the use of the sensor
    }

    public void setIntakeTestingMode(){   // sets mode to testing mode
        intakeMode = intakeState.TESTING;
    }

    public void setIntakeStopMode(){      // sets mode to stop
        intakeMode = intakeState.STOP;
    }

    public void setArmTestingMode(){
        armMode = armState.TESTING;
    }
    
    public void setArmStopMode(){
        armMode = armState.STOP;
    }

    public void setRetract(){
        armMode = armState.RETRACT;       //sets mode to when the extension is up
    }

    public void setExtend(){
        armMode = armState.EXTEND;        //sets mode to when the extension is down
    }
    
    public void setMidway(){
        armMode = armState.MIDWAY;
    }

    public boolean cargoCheck(){    //checks if the beam is being broken or not
        return intakeSensor.get();
    }

    public boolean armIsDown(){
        return armLimit.get();      //checks if intake arm is down
    }

    public boolean atMidway(){
        return intakeExtEnc.get() <= extEncMidWay;
    }

    public boolean extInsidePerimeter(){
        return intakeExtEnc.get() <= insideRobotPerimeter;
    }

    public void setBarSpeed(double barSpeed){
        intakeBar.set(barSpeed);
    }

    public void setRollerSpeed(double rollerSpeed){
        outerRollers.set(rollerSpeed);
    }

    //method for the motor intaking
    public void setIntakeSpeed(double speedForBar, double speedForRollers){     
        intakeBar.set(-speedForBar);
        outerRollers.set(-speedForRollers); // test motor
    }

    //output or outtaking
    public void setOuttakeSpeed(double speedForBar, double speedForRollers){ 
        intakeBar.set(speedForBar);
        outerRollers.set(speedForRollers); // test motor
    }

    //stops motor
    private void stopBarAndRolllers(){ 
        intakeBar.set(0);
        outerRollers.set(0);
    }
    
    //stops the intake extension motor
    private void stopIntakeExt(){
        intakeExt.set(0);
    }

    //retracts the intake up
    private void retract(double speedForIntakeExt){
        if (intakeExtEnc.get() > extEncUp){
            intakeExt.set(-speedForIntakeExt);
        }
        else{
            stopIntakeExt();
        }
    }

    //extends the intake down
    private void extend(double speedForIntakeExt){
        if (!armIsDown()){
            intakeExt.set(speedForIntakeExt);
        }

        else{
            stopIntakeExt();
            intakeExtEnc.reset();
        }
    }

    private void midway(double speedForIntakeExt){
        if(intakeExtEnc.get() > extEncMidWay){
            intakeExt.set(-speedForIntakeExt);
        }

        else{
            stopIntakeExt();
        }
    }

    //manually moves the intake extension motor
    public void manualIntakeExt(double speedForManualIntakeExt){
        intakeExt.set(speedForManualIntakeExt);
    }

    //intakes cargo and holds it when switch is being triggered
    private void intaking(){ 
        if (!cargoCheck()){
            timer.start();
            if (timer.get() > holdDelay){
                timer.stop();
                stopBarAndRolllers();
            }
            else{
                setIntakeSpeed(intakeSpeed, outerRollerSpeed);
            }
        }
        else{
            timer.reset();
            timer.stop();
            setIntakeSpeed(intakeSpeed, outerRollerSpeed);
        }
    }

    // feeds the ball into the shooter
    private void feeding(){ 
        if(!cargoCheck()){
            midway(intakeRetractSpeed);
            setIntakeSpeed(feedingSpeed, 0);
        }
        else{
            stopBarAndRolllers();
        }
    }

    public void resetEnc(){
        intakeExtEnc.reset();
    }

    //displays sensor values and intake state
    public void displayMethod(){
        
        SmartDashboard.putBoolean("Intake Sensor", cargoCheck());   // displays if the sensor is being triggered
        SmartDashboard.putString(" Intake mode", armMode.toString());          // displays the current state of the intake
        SmartDashboard.putString("Arm mode", armMode.toString());
        SmartDashboard.putNumber("Timer", timer.get());             // displays the time to the timer
        SmartDashboard.putNumber("Encoder for intake extension", intakeExtEnc.get());    // displays the encoder count
        SmartDashboard.putNumber("Speed for extension", intakeExt.get());         // displays the speed of the intake extension 
        SmartDashboard.putNumber("Extension counter", extCounter);  //
        SmartDashboard.putNumber("Case statement counter", counter);
        SmartDashboard.putBoolean("Arm is down", armIsDown());
    }

    public void intakeRun(){
        displayMethod();
        switch(intakeMode){

            case INTAKING:
            intaking();
            break;

            case OUTTAKING:
            setOuttakeSpeed(outtakeSpeed, outerRollerSpeed);
            break;

            case FEEDING:
            feeding();
            break;

            case OVERRIDE:
            setIntakeSpeed(intakeSpeed, outerRollerSpeed);
            break;

            case TESTING:
            break;

            case STOP:
            stopBarAndRolllers();
            break;

        }

        switch (armMode){

            case RETRACT:
            retract(intakeRetractSpeed);
            break;

            case EXTEND:
            extend(intakeExtSpeed);
            break;

            case MIDWAY:
            midway(intakeRetractSpeed);
            break;

            case STOP:
            stopIntakeExt();
            break;

            case TESTING:
            break;
        }
    }

}