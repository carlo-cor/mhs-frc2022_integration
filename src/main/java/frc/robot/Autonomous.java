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

            case 0:     //taxi off tarmac
                if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(3.33333)){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    oneBallCounter++;
                }
                else{
                    drive.tankRun(-0.5, -0.47);
                }
            break;

            case 1:     //rev shooter
                if(shooter.checkRPM()){
                    oneBallCounter++;
                }
                else{
                    shooter.setUpperHubShoot();
                }
            break;

            case 2:     //shoot ball if rpm is within range
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setStopMode();
                    oneBallCounter++;
                }

                else{
                    intake.setFeedingMode();
                }
            break;

        }
    }

    private void twoBall(){
        switch(twoBallCounter){

            case 0:     //taxi off tarmac
                if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(6)){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    twoBallCounter++;
                }
                else{
                    drive.tankRun(-0.45, -0.42);
                }
            break;

            case 1:     //rev the shooter                         
                if(shooter.checkRPM()){
                    twoBallCounter++;
                }
                else{
                    shooter.setUpperHubShoot();
                }      
            break;
                
            case 2:     //shoot the ball
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setStopMode();
                    twoBallCounter++;
                }
                else{
                    intake.setFeedingMode();
                }
            break;
            
            case 3:     //turn to face the cargo ball
                if(gyro.getYaw() > 35f && gyro.getYaw() < 45f ){
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    twoBallCounter++;
                }
                else{
                    drive.tankRun(0.55, -0.52);
                }
            break;

            case 4:     //intake the ball
                if(!intake.cargoCheck()){
                    drive.tankRun(0, 0);
                    intake.setStopMode();
                    twoBallCounter++;
                }
                else{
                    intake.setIntakeMode();
                    drive.tankRun(0.4, 0.37);
                }
            break;

            case 5:     //back up to initial position
                if(encoder.getPosition() <= 1 && encoder.getPosition() >= -1){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    twoBallCounter++;
                }
                else{
                    drive.tankRun(-0.5, -0.47);
                }
            break;

            case 6:     //turn back to face the upper hub
                if(gyro.getYaw() < 5 && gyro.getYaw() > -5){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    twoBallCounter++;
                }
                else{
                    drive.tankRun(-0.6, 0.57);
                }
            break; 

            case 7:     //rev the shooter
                if(shooter.checkRPM()){
                    twoBallCounter++;
                }
                else{
                    shooter.setUpperHubShoot();
                }
            break;

            case 8:     //shoot the ball
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setStopMode();
                    twoBallCounter++;
                }
                else{
                    intake.setFeedingMode();
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

        shooter.run();
        intake.run();
    }
}