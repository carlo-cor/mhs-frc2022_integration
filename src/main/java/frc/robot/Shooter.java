package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {

    //MOTOR VARIABLES
    private WPI_TalonFX shooterMotor;   //needed to import a TalonFX instead of MotorController in order to use its integrated sensors

    //CLASS VARIABLES
    private Limelight limelight;
    private Intake intake;

    //CONSTANTS
    private final double cameraHeight = 1.7916667;              //TEST VALUES
    private final double ballLaunchHeight = 1.5;  
    private final double targetHeight = 4.1666667;

    private final double cameraAngleDegrees = 0;
    private final double shooterAngleDegrees = 70.0; 
    private final double flywheelRadiusFeet = 0.2083333;

    private final double cameraToBallDistance = -0.2916667;
    private final double minimumShootingDistance = 5;
    private final double maximumTrackingDistance = 13;          //maximum distance limelight can track accurately
    private final double distanceFromLaunchPadtoHub = 20;

    //CONSTRUCTOR: the shooter will use one FALCON 500 Motor for the flywheel and will use the limelight to aim
    public Shooter(Limelight newlimelight, WPI_TalonFX newShooterMotor, Intake newIntake){    
        limelight = newlimelight;
        shooterMotor = newShooterMotor;
        intake = newIntake;
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

    //Checks if the RPM is within the +- 1 foot error margin
    private boolean checkIfRPMWithinRange(){
        double actualRPM = Math.abs(getActualRPM());
        double upperLimit = getIdealRPM(getIdealVelocity(getDistance() + 1));
        double lowerLimit = getIdealRPM(getIdealVelocity(getDistance() - 1));

        return (actualRPM < upperLimit && actualRPM > lowerLimit);
    }

    //Checks if the robot is able to clear the upper hub & if the limelight reading is accurate
    private boolean checkIfWithinShootingDistance(){
        return getDistance() > minimumShootingDistance && getDistance() < maximumTrackingDistance;
    }

    //Method to display sensor and calculated values
    public void displayValues(){
        SmartDashboard.putNumber("Distance from Center of Upper Hub", getDistance());
        SmartDashboard.putNumber("Ideal Velocity", getIdealVelocity(getDistance()) + 2);
        SmartDashboard.putNumber("Ideal RPM", getIdealRPM(getIdealVelocity(getDistance() + 2)));
        SmartDashboard.putNumber("Actual RPM", getActualRPM());  
        SmartDashboard.putNumber("Raw Sensor Velocity", shooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putBoolean("Within RPM Range?", checkIfRPMWithinRange());
        SmartDashboard.putBoolean("Within Shooting Distance?", checkIfWithinShootingDistance());
    }

    //stops the shooter
    private void stop(){
        shooterMotor.stopMotor();
    }

    private void lowHubShoot(){
        double setSpeed = 0;               //set values later
        double upperLimit = 0;
        double lowerLimit = 0;

        shooterMotor.set(setSpeed);        //set to a value later

        if(getActualRPM() < upperLimit && getActualRPM() > lowerLimit){
            intake.setFeedingMode();
        }
        else{
            intake.setStopMode();
        }

    }

    //Checks if the shooting distance is optimal, revs the shooter to a desired RPM, and feeds the ball in when it reaches the desired RPM
    private void upperHubShoot(){
        //double setSpeed = (getIdealRPM(getIdealVelocity(getDistance() + 2)) * 2048)/600;      
        double setSpeed = (getIdealRPM(getIdealVelocity(getDistance() + 2)) + 230) / 6640 ;     //multiplies the desired rpm by a proportion to get the desired percent output
        
        if(checkIfWithinShootingDistance()){
            shooterMotor.set(-setSpeed);                                                        //Will use some sort of PID later, just using a proportion for now
        }
        else{
            shooterMotor.stopMotor();
        }

        if(checkIfRPMWithinRange()){
            intake.setFeedingMode();
        }
        else{
            intake.setStopMode();
        }
        
    }

    //Sets the shooter to a set RPM to shoot from the Launch Pad, feeds the ball in when reaching a certain RPM
    private void launchPadShoot(){
        //double setSpeed = (getIdealRPM(getIdealVelocity(distanceFromLaunchPadtoHub)) * 2048)/ 600;    
        double setSpeed = (getIdealRPM(getIdealVelocity(distanceFromLaunchPadtoHub)) + 300) / 6600 ;    //multiplies the desired rpm by a proportion to get the desired percent output
        double upperLimit = getIdealRPM(getIdealVelocity(distanceFromLaunchPadtoHub + 1));
        double lowerLimit = getIdealRPM(getIdealVelocity(distanceFromLaunchPadtoHub - 1));
    
        shooterMotor.set(-setSpeed);                                                                    //Will use some sort of PID later, just using a proportion for now

        if(Math.abs(getActualRPM()) > lowerLimit && Math.abs(getActualRPM()) < upperLimit){
            intake.setFeedingMode();
        }
        else{
            intake.setStopMode();
        }
        
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

    }

}