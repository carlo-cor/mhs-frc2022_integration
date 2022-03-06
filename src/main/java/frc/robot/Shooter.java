package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter{

    //MOTOR VARIABLES
    private WPI_TalonFX shooterMotor;           //needed to import a TalonFX instead of MotorController in order to use its integrated sensors

    //CLASS VARIABLES
    private Limelight limelight;
    private Drive drive;

    //OTHER VARIABLES
    private double upperLimit;
    private double lowerLimit;
    private double setSpeed = 0;
    private double turnSpeed = 0;
    private double rangeSpeed = 0;

    //CONSTANTS (USING TEST VALUES FOR NOW)
    private final double cameraHeight = 1.79166667;             //height of the limelight from the ground
    private final double targetHeight = 3.64583333;//8.66666667;             //height of the target(upper hub)
    private final double cameraAngleDegrees = 0;                //angle of the limelight to the horizontal
    
    private final double minimumShootingDistance = 10;           //minimum distance ball has to be to clear the rim
    private final double maximumTrackingDistance = 17;          //maximum distance limelight can track accurately
    private final double cameraToBumperDistance = 0.9375;

    private final double lowHubSpeed = -0.4;                    //set speed to shoot in low hub
    private final double launchPadSpeed = -1.0;                 //set speed to shoot into the upper hub from the LaunchPad


    //CONSTRUCTOR
    public Shooter(Limelight newlimelight, WPI_TalonFX newShooterMotor, Drive newDrive){    
        limelight = newlimelight;
        shooterMotor = newShooterMotor;
        drive = newDrive; 
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

    public void setManual(double input){
        shooterMotor.set(input);
    }

    //Checks if the shooter is spinning fast enough
    public boolean checkRPM(){
        double actualRPM = Math.abs(getActualRPM());
        return actualRPM > lowerLimit && actualRPM < upperLimit;
    }

    //Checks if the robot is able to clear the upper hub & if the limelight reading is accurate 
    public boolean checkIfWithinShootingDistance(){
        return getDistance() > minimumShootingDistance && getDistance() < maximumTrackingDistance;
    }

    public boolean checkAligned(){
        return limelight.getXOffset() < 0.5 && limelight.getXOffset() > -0.5;
    }

    //Displays values and booleans
    public void displayValues(){
        SmartDashboard.putNumber("Distance (ft.)", getDistance());
        SmartDashboard.putNumber("Actual RPM", getActualRPM());  
        SmartDashboard.putNumber("Set Speed:", setSpeed);
        SmartDashboard.putNumber("Upper Limit:", upperLimit);
        SmartDashboard.putNumber("Lower Limit:", lowerLimit);

        SmartDashboard.putBoolean("In Range?", checkIfWithinShootingDistance());
        SmartDashboard.putBoolean("Aligned?", checkAligned());
        SmartDashboard.putBoolean("SHOOT!", checkRPM());
    }

    //Calculates the distance from the BUMPER to the CENTER OF THE HUB
    private double getDistance(){
        double heightDiff = targetHeight - cameraHeight;
        double cameraAngleRad = Math.toRadians(cameraAngleDegrees);
        double angleDiffRad = Math.toRadians(limelight.getYOffset());

        if(limelight.checkTargetSeen()){
            return (heightDiff/Math.tan(cameraAngleRad + angleDiffRad) - cameraToBumperDistance + 2);
        }
        else{
            return -1;
        }
    }

    //Converts the Sensor 'Velocity' to RPM (just for more convenience)
    private double getActualRPM(){
        double rawVelocity = shooterMotor.getSelectedSensorVelocity();
        return (rawVelocity * 600)/2048;
    }

    //Aligns the robot with the upper hub
    private void align(){
        double kP = 0.01;
        double minCommand = 0.2;
        double error = limelight.getXOffset();

        limelight.setTrackingMode();

        if(error > 0.5){
            turnSpeed = (kP * error) + minCommand;
        }
        else if(error < -0.5){
            turnSpeed = (kP * error) - minCommand;
        }
        else{
            turnSpeed = 0;
        }
    }

    private void getInRange(){

        limelight.setTrackingMode();

        if(getDistance() < minimumShootingDistance + 1){
            rangeSpeed = 0.4;
        }
        else if(getDistance() > maximumTrackingDistance - 1){
            rangeSpeed = -0.4;
        }
        else{
            rangeSpeed = 0;
        }
    }


    //stops the shooter
    private void stop(){
        shooterMotor.stopMotor();
        setSpeed = 0;
        upperLimit = -1;
        lowerLimit = -1;
    }

    //Method to shoot in the low hub
    private void lowHubShoot(){
        setSpeed = lowHubSpeed; 
        upperLimit = 2600;
        lowerLimit = 2200;
        shooterMotor.set(-setSpeed); 
    }

    //Method to shoot in the upper hub using limelight
    private void upperHubShoot(){ 
        limelight.setTrackingMode();

        align();
        getInRange();
        drive.arcadeRun(turnSpeed, rangeSpeed);

        if(getDistance() > minimumShootingDistance && getDistance() < 11){
            setSpeed = -0.692;
            upperLimit = 0;
            lowerLimit = 0;
        }
        else if(getDistance() > 11 && getDistance() < 12){
            setSpeed = -0.725;
            upperLimit = 0;
            lowerLimit = 0;
        }
        else if(getDistance() > 12 && getDistance() < 13){
            setSpeed = -0.7618;
            upperLimit = 0;
            lowerLimit = 0;
        }
        else if(getDistance() > 13 && getDistance() < 14){
            setSpeed = -0.804;
            upperLimit = 0;
            lowerLimit = 0;
        }
        else if(getDistance() > 14 && getDistance() < 15){
            setSpeed = -0.852;
            upperLimit = 0;
            lowerLimit = 0;
        }
        else if(getDistance() > 15 && getDistance() < 16){
            setSpeed = -0.906;
            upperLimit = 0;
            lowerLimit = 0;
        }
        else if(getDistance() > 16 && getDistance() < maximumTrackingDistance){
            setSpeed = -0.968;
            upperLimit = 0;
            lowerLimit = 0;
        }
        else{
            setSpeed = 0;
            upperLimit = -1;
            lowerLimit = -1;
        }

        shooterMotor.set(setSpeed);
    }

    //Method to shoot in the upper hub from the launch pad
    private void launchPadShoot(){    
        limelight.setTrackingMode();

        align();
        drive.arcadeRun(turnSpeed, 0);

        setSpeed = launchPadSpeed;  
        upperLimit = 6400;
        lowerLimit = 6200;   
        shooterMotor.set(setSpeed); 
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