package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//(TEST)
import edu.wpi.first.math.controller.PIDController;

public class Shooter{

    //MOTOR VARIABLES
    private WPI_TalonFX shooterMotor;                  

    //CLASS VARIABLES
    private Limelight limelight;
    private PIDController pid;

    //METHOD VARIABLES
    public double alignSpeed = 0;
    public double getInRangeSpeed = 0;

    //CONSTANTS
    private final double cameraHeight = 1.79166667;                 //height of the limelight from the ground
    private final double targetHeight = 8.46875;                    //height of the target(upper hub) from floor to the bottom of reflective tape
    private final double cameraAngleDegrees = 28.125028259;         //angle of the limelight to the horizontal
    
    private final double minimumShootingDistance = 8;               //minimum distance ball has to be to make it in the upper hub
    private final double maximumTrackingDistance = 15;              //maximum distance limelight can track accurately
    private final double cameraToBumperDistance = 1.458333;         //distance from camera to bumper

    private final double rpmToSpeed = 0.000155;
    private final double lowHubRPM = 2500;
    private final double launchPadRPM = 6350;

    //CONSTRUCTOR
    public Shooter(Limelight newlimelight, WPI_TalonFX newShooterMotor){    
        limelight = newlimelight;
        shooterMotor = newShooterMotor;

        pid = new PIDController(0.0002, 0.00002, 0);   

        //PLAY AROUND WITH LATER (PID TUNING IS STILL UNFINISHED!!!) (old = -0.5, 0.5)
        pid.setIntegratorRange(-1, 1);
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

    //*set the state to TESTING before using this method*
    public void setManual(double input){
        shooterMotor.set(input);
    }
    
    //Displays values and booleans
    public void displayValues(){
        
        SmartDashboard.putNumber("Distance (ft.)", getDistance());
        SmartDashboard.putBoolean("SHOOT!", checkRPM());
        SmartDashboard.putBoolean("In Range?", checkInRange());
        SmartDashboard.putBoolean("Aligned?", checkAligned());
        SmartDashboard.putBoolean("Target Seen?", limelight.checkTargetSeen());
        
        //USED ONLY FOR TESTING:
        /*
        SmartDashboard.putNumber("Actual RPM", getActualRPM());
        SmartDashboard.putNumber("Ideal RPM", getIdealRPM());
        SmartDashboard.putNumber("Position Error", pid.getPositionError());
        SmartDashboard.putNumber("Velocity Error", pid.getVelocityError());
        SmartDashboard.putNumber("Setpoint", pid.getSetpoint());
        SmartDashboard.putNumber("Shooter Output:", shooterMotor.get());
        SmartDashboard.putNumber("Align Speed", alignSpeed);
        SmartDashboard.putNumber("Range Speed", getInRangeSpeed);
        */

        //PID TESTING
        /*
        SmartDashboard.putNumber("kP", SmartDashboard.getNumber("kP", 0));
        SmartDashboard.putNumber("kI", SmartDashboard.getNumber("kI", 0));
        SmartDashboard.putNumber("kD", SmartDashboard.getNumber("kD", 0));
        pid.setP(SmartDashboard.getNumber("kP", 0));
        pid.setI(SmartDashboard.getNumber("kI", 0));
        pid.setD(SmartDashboard.getNumber("kD", 0));
        */
    }

    //Checks if the shooter is spinning fast enough
    public boolean checkRPM(){
        if(shooterState == state.STOP){
            return false;
        }
        //DELETE LATER
        else if(shooterState == state.TESTING){
            return true;
        }
        //
        else{
            return pid.atSetpoint();
        }
    }
    
    //Check if the shooter is aligned with the hub
    public boolean checkAligned(){
        return limelight.getXOffset() < 0.5 && limelight.getXOffset() > -0.5 && limelight.checkTargetSeen();
    }

    //Checks if the robot is able to clear the upper hub & if the limelight reading is accurate 
    public boolean checkInRange(){
        return getDistance() > minimumShootingDistance && getDistance() < maximumTrackingDistance;
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

    //Converts distance to rpm (TEST)
    private double getIdealRPM(){
        if(getDistance() < 0){
            return 0;
        }
        else{
            /*TRIAL 1 DATA (EXPONENTIAL): 
            return 223.744 * Math.pow(1.13965, getDistance()) + 4026.4;
            */
            //TRIAL 2 DATA (QUADRATIC):
            return 6.45688 * Math.pow(getDistance(), 2) + 13.3939 * getDistance() + 4073.73 - 420.69;
        }
    }

    //Returns the required motor speed according to a given rpm (TEST)
    private double setSpeed(double rpm){

        double speed;
        double baseSpeed = rpm * rpmToSpeed;

        pid.setSetpoint(rpm);
        speed = baseSpeed + pid.calculate(getActualRPM());

        if(speed > 1.0){
            return 1.0;
        }
        else{
            return speed;
        }
    }

    //Aligns the robot with the upper hub
    private void align(){
        double kP = 0.0125;
        double minCommand = 0.25;
        double error = limelight.getXOffset();

        limelight.setTrackingMode();

        if(error > 1){
            alignSpeed = (kP * error) + minCommand;
        }
        else if(error < -1){
            alignSpeed = (kP * error) - minCommand;
        }
        else{
            alignSpeed = 0;
        }
    }

    //Gets the robot to an optimal shooting range for the upper hub as long as limelight sees target (still need to test)
    private void getInRange(){
        double kP = 0.025;
        double minCommand = 0.4;
        double error = 0;

        limelight.setTrackingMode();

        //if TOO CLOSE to the hub, move BACKWARDS
        if(getDistance() > 0 && getDistance() < minimumShootingDistance + 0.5){ 
            error = minimumShootingDistance - getDistance(); 
            getInRangeSpeed = (kP * error) + minCommand;    //0.45
        }
        //if TOO FAR to the hub, move FORWARD
        else if(getDistance() > maximumTrackingDistance - 0.5){ 
            error = maximumTrackingDistance - getDistance();
            getInRangeSpeed = (kP * error) - minCommand;
        }
        else{
            getInRangeSpeed = 0;
        }     
    }

    //Stops the shooter
    private void stop(){
        pid.reset();
        shooterMotor.stopMotor();
        alignSpeed = 0;
        getInRangeSpeed = 0;
    }

    //Method to shoot in the low hub
    private void lowHubShoot(){
        pid.setTolerance(80);

        alignSpeed = 0;
        getInRangeSpeed = 0;

        shooterMotor.set(setSpeed(lowHubRPM));
    }

    //Method to shoot in the upper hub using limelight (STILL NEED TO GET VALUES FOR REAL ROBOT)
    private void upperHubShoot(){
        limelight.setTrackingMode();

        pid.setTolerance(70);

        align();
        getInRange();

        if(getDistance() > minimumShootingDistance && getDistance() < maximumTrackingDistance){
            shooterMotor.set(setSpeed(getIdealRPM()));
        }
        else{
            shooterMotor.stopMotor();
        }
    }

    //Method to shoot in the upper hub from the launch pad
    private void launchPadShoot(){    
        limelight.setTrackingMode();

        pid.setTolerance(80);

        align();
        getInRangeSpeed = 0;

        shooterMotor.set(setSpeed(launchPadRPM));
    }

    private void testing(){
        align();
        getInRange();
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
               testing();
            break;
        }

        limelight.run();

    }

}