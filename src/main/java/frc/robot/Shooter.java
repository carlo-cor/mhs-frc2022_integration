package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

public class Shooter {

    //MOTOR VARIABLES
    private WPI_TalonFX shooterMotor;           //needed to import a TalonFX instead of MotorController in order to use its integrated sensors

    //CLASS VARIABLES
    private Limelight limelight;
    private PIDController shooterPID;
    private Drive drive;

    //CONSTANTS (USING TEST VALUES FOR NOW)
    private final double cameraHeight = 1.7916667;              //height of the limelight from the ground
    private final double ballLaunchHeight = 1.5;                //height the ball will be launched 
    private final double targetHeight = 4.1666667;              //height of the target(upper hub)

    private final double cameraAngleDegrees = 0;                //angle of the limelight
    private final double shooterAngleDegrees = 70.0;            //initial angle the ball will be shot at
    private final double flywheelRadiusFeet = 0.2083333;        //radius of the flywheel in FEET

    private final double cameraToBallDistance = -0.2916667;     //offset between ball and limelight
    private final double minimumShootingDistance = 5;           //minimum distance ball has to be to clear the rim
    private final double maximumTrackingDistance = 13;          //maximum distance limelight can track accurately

    private final double lowHubSpeed = -0.4;                       //set speed to shoot in low hub
    private final double launchPadSpeed = 1;                      //ideal RPM to shoot into the upper hub from the LaunchPad


    //CONSTRUCTOR
    public Shooter(Limelight newlimelight, WPI_TalonFX newShooterMotor, Drive newDrive){    
        limelight = newlimelight;
        shooterMotor = newShooterMotor;
        drive = newDrive;
        shooterPID = new PIDController(0, 0, 0);   
    }

    //STATES: the shooter will either shoot with a set speed from the launch pad, adjust its RPM according to the limelight, shoot in the low hub, or not shoot at all
    private enum state{
        STOP, LOWHUB, UPPERHUB, LAUNCHPAD, TESTING
    }

    private state shooterState = state.STOP; //default state will be set to STOP

    public void setStop(){
        shooterState = state.STOP;
    }

    public void setLowHubShoot(){
        shooterState = state.LOWHUB;
    }

    public void setUpperHubShoot(){
        shooterState = state.UPPERHUB;
    }

    public void setLaunchPadShoot(){
        shooterState = state.LAUNCHPAD;
    }

    public void setTesting(){
        shooterState = state.TESTING;
    }

    public void setSpeedManual(double input){
        shooterMotor.set(input);
    }

    //Method to calculate the distance from the BALL to the CENTER of the upper hub
    private double getDistance(){
        double heightDiff = targetHeight - cameraHeight;
        double cameraAngleRad = Math.toRadians(cameraAngleDegrees);
        double angleDiffRad = Math.toRadians(limelight.getYOffset());

        if(limelight.checkTargetSeen()){
            return cameraToBallDistance + (heightDiff/Math.tan(cameraAngleRad + angleDiffRad)) + 2;
        }
        else{
            return -1;
        }
    }

    //Method to calculate the ideal initial velocity of the ball given a distance
    private double getIdealVelocity(double distance){
        double heightDiff = targetHeight - ballLaunchHeight;
        double shooterAngleRad = Math.toRadians(shooterAngleDegrees);
        
        if(distance > 0){
            return Math.sqrt((16*distance*distance)/((Math.sin(shooterAngleRad) * Math.cos(shooterAngleRad) * distance) - (Math.pow(Math.cos(shooterAngleRad), 2) * heightDiff))); 
        }

        else{
            return 0;
        }
    }

    //Method to calculate the ideal RPM given a velocity 
    private double getIdealRPM(double velocity){
        return (30 * velocity)/(Math.PI * flywheelRadiusFeet);
    }

    //Converts the Sensor 'Velocity' to RPM
    private double getActualRPM(){
        double rawVelocity = shooterMotor.getSelectedSensorVelocity();
        return (rawVelocity * 600)/2048;
    }

    //Checks if the RPM is within the +- 1 foot error margin  (used only for the upper hub method & autonomous)
    public boolean checkIfRPMWithinRange(){
        double actualRPM = Math.abs(getActualRPM());
        double upperLimit = getIdealRPM(getIdealVelocity(getDistance() + 1));
        double lowerLimit = getIdealRPM(getIdealVelocity(getDistance() - 1));

        return (actualRPM < upperLimit && actualRPM > lowerLimit);
    }

    //Checks if the robot is able to clear the upper hub & if the limelight reading is accurate 
    private boolean checkIfWithinShootingDistance(){
        return getDistance() > minimumShootingDistance && getDistance() < maximumTrackingDistance;
    }

    private void align(){
        double kP = 0;
        double minCommand = 0;
        double error = limelight.getXOffset();
        double adjustSpeed;

        if(error > 0){
            adjustSpeed = (kP * error) + minCommand;
        }
        else if(error < 0){
            adjustSpeed = (kP * error) - minCommand;
        }
        else{
            adjustSpeed = 0;
        }

        drive.arcadeRun(adjustSpeed, 0);
    }

    //Method to display values and booleans
    public void displayValues(){

        //Measured and calculated values
        SmartDashboard.putNumber("Ball Distance from Center of Upper Hub", getDistance());
        SmartDashboard.putNumber("Limelight Distance from Rim", getDistance() - cameraToBallDistance - 2);
        SmartDashboard.putNumber("Ideal Velocity", getIdealVelocity(getDistance()));
        SmartDashboard.putNumber("Ideal RPM", getIdealRPM(getIdealVelocity(getDistance())));
        SmartDashboard.putNumber("Actual RPM", getActualRPM());  
        SmartDashboard.putNumber("Raw Sensor Velocity", shooterMotor.getSelectedSensorVelocity());

        //Booleans
        SmartDashboard.putBoolean("Low Hub Shoot", lowHubShoot());
        SmartDashboard.putBoolean("Upper Hub Shoot", upperHubShoot());
        SmartDashboard.putBoolean("Launch Pad Shoot", launchPadShoot());

        //PID Values
        SmartDashboard.putNumber("Get Error", shooterPID.getPositionError());
        SmartDashboard.putNumber("P-term", shooterPID.getP());
        SmartDashboard.putNumber("I-Term", shooterPID.getI());
        SmartDashboard.putNumber("D-Term", shooterPID.getD());
    }

    //stops the shooter and resets the PID
    private void stop(){
        shooterPID.setP(0);
        shooterPID.setI(0);
        shooterPID.setD(0);
        shooterPID.reset();
        shooterMotor.stopMotor();
    }

    //Method to shoot in the low hub
    private boolean lowHubShoot(){
        double lowerLimit = 0;          //minimum RPM to make it in
        double upperLimit = 0;          //maximum RPM to make it in 
    
        shooterMotor.set(lowHubSpeed);   
        
        return Math.abs(getActualRPM()) > lowerLimit && Math.abs(getActualRPM()) < upperLimit;

        //NEVERMIND, THIS WILL BECOME MANUAL
        /*if(getActualRPM() > lowerLimit && getActualRPM() < upperLimit){     //if the shooter is fast enough then shoot
            intake.setFeedingMode();
        }*/

    }

    //Method to shoot in the upper hub using limelight
    private boolean upperHubShoot(){  
        
        limelight.setTrackingMode();
        
        shooterPID.setP(0);
        shooterPID.setI(0);
        shooterPID.setD(0);
        double setSpeed = shooterPID.calculate(getActualRPM(), getIdealRPM(getIdealVelocity(getDistance())));   //PID used to keep speed ideal 

        align();
        
        if(checkIfWithinShootingDistance()){    //if in shooting range, rev the shooter
            shooterMotor.set(setSpeed);                                                        
        }
        else{
            shooterMotor.stopMotor();           //if not in shooting range, DO NOT SHOOT
        }

        return checkIfRPMWithinRange();

        //NEVERMIND, THIS WILL BECOME MANUAL
        /*if(checkIfRPMWithinRange()){            //if the shooter is fast enough, then feed ball in and SHOOT     
            intake.setFeedingMode();
        }*/
        
    }

    //Method to shoot in the upper hub from the launch pad
    private boolean launchPadShoot(){    

        limelight.setTrackingMode();
                                        //PID to keep speed ideal
        double lowerLimit = 0;          //minimum RPM to make it in
        double upperLimit = 0;          //maximum RPM to make it in

        align();
    
        shooterMotor.set(launchPadSpeed);      
        
        return Math.abs(getActualRPM()) > lowerLimit && Math.abs(getActualRPM()) < upperLimit;

        //NEVERMIND, THIS WILL BECOME MANUAL
        /*if(getActualRPM() > lowerLimit && getActualRPM() < upperLimit){     //if shooter is fast enough, then feed ball in and SHOOT 
            intake.setFeedingMode();
        }*/
        
    }

    public void run(){
        switch(shooterState){
            case STOP:
                stop();
            break;
            case LOWHUB:
                lowHubShoot();
            break;
            case UPPERHUB:
                upperHubShoot();
            break;
            case LAUNCHPAD:
                launchPadShoot();
            break;
            case TESTING:
            break;
        }

        limelight.run();

    }

}