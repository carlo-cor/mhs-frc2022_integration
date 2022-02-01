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
    private final double cameraHeight = 0;
    private final double ballLaunchHeight = 0;  
    private final double targetHeight = 0;

    private final double cameraAngleDegrees = 0;
    private final double shooterAngleDegrees = 0;
    private final double flywheelRadiusFeet = 0;

    private final double minimumShootingDistance = 0;
    private final double maximumTrackingDistance = 0;   //maximum distance limelight can track accurately
    private final double distanceFromLaunchPadtoHub = 0;

    //CONSTRUCTOR: the shooter will use one FALCON 500 Motor for the flywheel and will use the limelight to aim
    public Shooter(Limelight newlimelight, WPI_TalonFX newShooterMotor, Intake newIntake){    
        limelight = newlimelight;
        shooterMotor = newShooterMotor;
        intake = newIntake;
    }

    //STATES: the shooter will either shoot with a set speed from the launch pad, adjust its RPM according to the limelight, or not shoot at all 
    private enum state{
        STOP, LOWHUBSHOOT, ADJUSTANDSHOOT, LAUNCHPADSHOOT
    }

    private state shooterState = state.STOP; //default state will be set to STOP

    public void setStop(){
        shooterState = state.STOP;
    }

    public void setLowHubShoot(){
        shooterState = state.LOWHUBSHOOT;
    }

    public void setAutoShoot(){
        shooterState = state.ADJUSTANDSHOOT;
    }

    public void setLaunchPadShoot(){
        shooterState = state.LAUNCHPADSHOOT;
    }

    //Method to calculate the distance from the ball to the rim of the upper hub
    private double getDistance(){
        double heightDiff = targetHeight - cameraHeight;
        double cameraAngleRad = Math.toRadians(cameraAngleDegrees);
        double angleDiffRad = Math.toRadians(limelight.getYOffset());

        if(limelight.checkTargetSeen()){
            return (heightDiff/Math.tan(cameraAngleRad + angleDiffRad));
        }
        else{
            return -1;
        }
    }

    //Method to calculate the ideal initial velocity of a ball given a distance
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
        double actualRPM = getActualRPM();
        double upperLimit = getIdealRPM(getIdealVelocity(getDistance() + 3));
        double lowerLimit = getIdealRPM(getIdealVelocity(getDistance() + 1));

        return (actualRPM < upperLimit && actualRPM > lowerLimit);
    }

    //Checks if the robot is able to clear the upper hub & if the limelight reading is accurate
    private boolean checkIfWithinShootingDistance(){
        return getDistance() > minimumShootingDistance && getDistance() < maximumTrackingDistance;
    }

    //Method to display sensor and calculated values
    public void displayValues(){
        SmartDashboard.putNumber("Distance from Rim", getDistance());
        SmartDashboard.putNumber("Ideal Velocity", getIdealVelocity(getDistance()) + 2);
        SmartDashboard.putNumber("Ideal RPM", getIdealRPM(getIdealVelocity(getDistance() + 2)));
        SmartDashboard.putNumber("Raw Sensor Velocity", shooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Actual RPM", getActualRPM());
    }

    //stops the shooter
    private void stop(){
        shooterMotor.stopMotor();
    }

    private void lowHubShoot(){
        shooterMotor.set(0);        //set to a value later
    }

    //shoots the ball if within a certain distance and if the RPM is good enough
    private void adjustShoot(){
        double setSpeed = (getIdealRPM(getIdealVelocity(getDistance() + 2)) * 2048)/600;
        
        if(checkIfRPMWithinRange() && checkIfWithinShootingDistance()){
            intake.setFeedingMode();
        }
        else{
            shooterMotor.set(ControlMode.Velocity, setSpeed);
        }
    }

    private void launchPadShoot(){
        double setSpeed = getIdealRPM(getIdealVelocity(distanceFromLaunchPadtoHub));
        double upperLimit = getIdealRPM(getIdealVelocity(distanceFromLaunchPadtoHub + 1));
        double lowerLimit = getIdealRPM(getIdealVelocity(distanceFromLaunchPadtoHub - 1));
    
        if(getActualRPM() > lowerLimit && getActualRPM() < upperLimit){
            intake.setFeedingMode();
        }
        else{
            shooterMotor.set(ControlMode.Velocity, setSpeed);
        }
    }

    public void run(){
        switch(shooterState){
            case STOP:
                stop();
            break;
            case LOWHUBSHOOT:
                lowHubShoot();
            break;
            case ADJUSTANDSHOOT:
                adjustShoot();
            break;
            case LAUNCHPADSHOOT:
                launchPadShoot();
            break;
        }

    }

}