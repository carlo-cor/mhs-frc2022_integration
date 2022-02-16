package frc.robot;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

public class Autonomous {

    //SENSOR VARIABLES:
    private RelativeEncoder encoder;
    private AHRS gyro;

    //CLASS VARIABLES:
    private Drive drive;
    private Shooter shooter;
    private Intake intake;

    //COUNTER VARIABLES:
    private int oneBallCounter = 0;
    private int twoBallCounter = 0;
    private int threeBallCounter = 0;

    //CONSTANTS:
    private final double encCountsPerFoot = 3.15872823333;
    
    public Autonomous(Drive newDrive, Shooter newShooter, Intake newIntake, RelativeEncoder newEncoder, AHRS newGyro){
        drive = newDrive;       
        shooter = newShooter;
        intake = newIntake;
        encoder = newEncoder;
        gyro = newGyro;
    }

    private enum routines{
        NOTHING, ONEBALL, TWOBALL, THREEBALL
    }

    private routines routineState = routines.NOTHING;

    public void setNothing(){
        routineState = routines.NOTHING;
    }

    public void setOneBall(){
        routineState = routines.ONEBALL;
    }

    public void setTwoBall(){
        routineState = routines.TWOBALL;
    }

    public void setThreeBall(){
        routineState = routineState.THREEBALL;
    }

    public void display(){
        SmartDashboard.putNumber("One Ball Counter", oneBallCounter);
        SmartDashboard.putNumber("Two Ball Counter", twoBallCounter);
        SmartDashboard.putNumber("Three Ball Counter", threeBallCounter);
        SmartDashboard.putNumber("Encoder Counts", encoder.getPosition());
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
    }

    public void reset(){
        encoder.setPosition(0);
        gyro.reset();
        oneBallCounter = 0;
        twoBallCounter = 0;
        threeBallCounter = 0;
    }

    private double convertFeetToEncoderCounts(double feet){
        return feet * encCountsPerFoot;
    }

    private void nothing(){

    }

    private void oneBall(){
        switch(oneBallCounter){
            case 0:     //rev shooter
                if(/*shooter.checkIfRPMWithinRange()*/true){
                    shooter.setSpeedManual(-0.4);
                    oneBallCounter++;
                }
                else{
                    //shooter.setUpperHubShoot();
                }
            break;

            case 1:     //shoot ball if rpm is within range
                if(/*!intake.cargoCheck()*/true){
                    intake.setFeedingMode();
                    //shooter.setStop();
                    //intake.setStopMode();
                    oneBallCounter++;
                }

                else{
                    //intake.setFeedingMode();
                }
            break;

            case 2:     //Taxi off the tarmac
                if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(4)){
                    drive.tankRun(0, 0);
                    shooter.setSpeedManual(0);
                    intake.setStopMode();
                    oneBallCounter++;
                }
                else{
                    drive.tankRun(0.4, -0.37);
                }
        }
    }

    private void twoBall(){
        switch(twoBallCounter){
            case 0:     //rev the shooter                         
                if(/*shooter.checkIfRPMWithinRange()*/true){
                    shooter.setSpeedManual(-0.4);
                    twoBallCounter++;
                }
                else{
                    //shooter.setUpperHubShoot();
                }      
            break;

            case 1:     //shoot the ball
                if(/*!intake.cargoCheck()*/true){
                    intake.setFeedingMode();
                    //shooter.setStop();
                    //intake.setStopMode();
                    twoBallCounter++;
                }
                else{
                    //intake.setFeedingMode();
                }

            break;
            
            case 2:     //turn around to face the cargo ball
                if(gyro.getYaw() > 160.0f && gyro.getYaw() < 200.0f ){
                    encoder.setPosition(0);
                    drive.tankRun(0, 0);
                    shooter.setSpeedManual(0);
                    intake.setStopMode();   
                    twoBallCounter++;
                }
                else{
                    drive.tankRun(0.6, 0.57);
                }
            break;

            case 3:     //intake the ball
                if(/*intake.cargoCheck()*/ Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(4)){
                    drive.tankRun(0, 0);
                    intake.setStopMode();
                    twoBallCounter++;
                }
                else{
                    //intake.setIntakeMode();
                    intake.setFeedingMode();
                    drive.tankRun(-0.4, 0.37);
                }
            break;

            case 4:     //turn back to face the upper hub
                if(gyro.getYaw() < 10 && gyro.getYaw() > -10){
                    drive.tankRun(0, 0);
                    twoBallCounter++;
                }
                else{
                    drive.tankRun(0.6, 0.57);
                }
            break; 

            case 5:     //rev the shooter
                if(/*shooter.checkIfRPMWithinRange()*/true){
                    shooter.setSpeedManual(-0.4);
                    twoBallCounter++;
                }
                else{
                    //shooter.setUpperHubShoot();
                }
            break;

            case 6:     //shoot the ball
                if(/*intake.cargoCheck()*/true){
                    intake.setFeedingMode();
                    //shooter.setStop();
                    //intake.setStopMode();
                    twoBallCounter++;
                }
                else{
                    //intake.setFeedingMode();
                }
            break;
        }
    }

    private void threeBall(){
        switch(threeBallCounter){
            case 0:
            //????????????????????????????
            break;
        }
    }

    public void run(){
        switch(routineState){
            case NOTHING:
                nothing();
            break;

            case ONEBALL:
                oneBall();
            break;

            case TWOBALL:
                twoBall();
            break;

            case THREEBALL:
                threeBall();
            break;

        }

        //shooter.run();
        intake.run();
    }
}