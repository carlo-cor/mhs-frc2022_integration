package frc.robot;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;

public class Autonomous {

    //SENSOR VARIABLES:
    private RelativeEncoder encoder;
    private AHRS gyro;

    //CLASS VARIABLES:
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Timer timer;

    //COUNTER VARIABLES:
    private int oneBallCounter = 0;
    private int twoBallCounter = 0;
    private int threeBallHighCounter = 0;
    private int threeBallHighLowCounter = 0;

    //CONSTANTS:
    private final double encCountsPerFoot = 11.1029532;
    
    public Autonomous(Drive newDrive, Shooter newShooter, Intake newIntake, RelativeEncoder newEncoder, AHRS newGyro){
        drive = newDrive;       
        shooter = newShooter;
        intake = newIntake;
        encoder = newEncoder;
        gyro = newGyro;
        timer = new Timer();
    }

    private enum routines{
        NOTHING, ONEBALL, TWOBALL, THREEBALLHIGH, THREEBALLHIGHLOW
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

    public void setThreeBallHigh(){
        routineState = routines.THREEBALLHIGH;
    }

    public void setThreeBallHighLow(){
        routineState = routines.THREEBALLHIGHLOW;
    }


    public void display(){
        SmartDashboard.putNumber("One Ball Counter", oneBallCounter);
        SmartDashboard.putNumber("Two Ball Counter", twoBallCounter);
        SmartDashboard.putNumber("Three Ball High Counter", threeBallHighCounter);
        SmartDashboard.putNumber("Three Ball High Low Counter", threeBallHighLowCounter);
        SmartDashboard.putNumber("Encoder Counts", encoder.getPosition());
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
        SmartDashboard.putBoolean("CheckRPM", shooter.checkRPM());
    }

    public void reset(){
        encoder.setPosition(0);
        gyro.reset();
        oneBallCounter = 0;
        twoBallCounter = 0;
        threeBallHighCounter = 0;
        threeBallHighLowCounter = 0;
    }

    private double convertFeetToEncoderCounts(double feet){
        return feet * encCountsPerFoot;
    }

    private void nothing(){

    }

    private void oneBall(){
        switch(oneBallCounter){

            case 0:     //taxi off tarmac
                if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(4)){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    oneBallCounter++;
                }
                else{
                    drive.tankRun(-0.6, -0.6);
                }
            break;

            case 1:     //rev shooter
                if(shooter.checkRPM()){
                    oneBallCounter++;
                }
                else{
                    shooter.setLowHubShoot();
                    //sooter.setUpperHubShoot
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
                if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(6.5)){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    twoBallCounter++;
                }
                else{
                    drive.tankRun(-0.6, -0.6);
                }
            break;

            case 1:     //rev the shooter                         
                if(shooter.checkRPM()){
                    twoBallCounter++;
                }
                else{
                    //shooter.setUpperHubShoot();
                    shooter.setLowHubShoot();
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
                if(gyro.getYaw() > 48f && gyro.getYaw() < 53f ){
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    twoBallCounter++;
                }
                else{
                    drive.tankRun(0.5, -0.5);
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
                    drive.tankRun(0.5, 0.5);
                }
            break;

            case 5:     //back up to initial position
                if(encoder.getPosition() <= 1 && encoder.getPosition() >= -1){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    twoBallCounter++;
                }
                else{
                    drive.tankRun(-0.5, -0.5);
                    twoBallCounter++;
                }
            break;

            case 6:     //turn back to face the upper hub
                if(gyro.getYaw() < 5 && gyro.getYaw() > -5){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    twoBallCounter++;
                }
                else{
                    drive.tankRun(-0.5, 0.5);
                }
            break; 

            case 7:     //rev the shooter
                if(shooter.checkRPM()){
                    twoBallCounter++;
                }
                else{
                    //shooter.setUpperHubShoot();
                    shooter.setLowHubShoot();
                }
            break;

            case 8:
                if(timer.get() >= 1){
                    timer.stop();
                    timer.reset();
                    twoBallCounter++;
                }
                else{
                    timer.start();
                    intake.setOverrideMode();
                }
            case 9:     //shoot the ball
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

    private void threeLowHighBall(){        //TWO BALL HIGH, ONE BALL LOW
        switch(threeBallHighLowCounter){   
            case 0: 
            if (encoder.getPosition() <= -13.6) {
                drive.tankRun(0, 0); 
                encoder.setPosition(0); 
                threeBallHighLowCounter++; 
            } else {
                drive.tankRun(-0.60, -0.57); 
            }
            break; 

            case 1:
            if(shooter.checkRPM()){                 //shoot preload into low hub
                threeBallHighLowCounter++;
            }

            else{
                shooter.setLowHubShoot();
            }
            break;

            case 2:
            if(intake.cargoCheck()){
                intake.setStopMode();
                shooter.setStop();
                threeBallHighLowCounter++;
            }

            else{
                intake.setFeedingMode();
            }
            break;

            case 3:                                                 //turn right to ball by the wall
            if(gyro.getYaw() < 146 && gyro.getYaw() > 140){
                drive.tankRun(0, 0);
                encoder.setPosition(0);
                threeBallHighLowCounter++;
            }

            else{
                drive.tankRun(0.45, -0.45);
            }
            break;

            case 4:                                                   //forward and intake 2nd ball
            if(!intake.cargoCheck() || encoder.getPosition() > 71.5){       
                drive.tankRun(0, 0);
                encoder.setPosition(0);
                threeBallHighLowCounter++;
            }
            else{
                intake.setIntakeMode();
                drive.tankRun(0.65, 0.65);
            }
            break;

            case 5:
            if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(1.5)){           //back up a foot and a half so you dont hit the wall when you turn around
                drive.tankRun(0, 0);
                encoder.setPosition(0);
                gyro.reset();
                threeBallHighLowCounter++;
            }
            else{
                drive.tankRun(-0.53, -0.50);
            }
            break;                                                                  //reset gyro here and we should know what the gyro needs to be for the third ball

            case 6:
            if(gyro.getYaw() > -143f && gyro.getYaw() < -137){                              //turn left to face hub
                drive.tankRun(0, 0);   
                encoder.setPosition(0);
                threeBallHighLowCounter++;
            }
            else{
                drive.tankRun(-0.55, 0.52);
            }
            break;

            case 7:                                                              //rev the shooter                         
            if(shooter.checkRPM()){
                threeBallHighLowCounter++;
            }
            else{
                //shooter.setUpperHubShoot();
                shooter.setLowHubShoot();
            }      
            break;
            
            case 8:                                                                              //shoot the 2nd ball
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setStopMode();
                    threeBallHighLowCounter++;
                }
                else{
                    intake.setFeedingMode();
                }
            break;                                                                                  //gyro at -7.6

            case 9:                                                                                 //gyro to -100
            if(gyro.getYaw() > 117f && gyro.getYaw() < 123f){                              //turn left to face third ball   [CONSIDER MAKING THIS FASTER!]
                drive.tankRun(0, 0);   
                encoder.setPosition(0);
                threeBallHighLowCounter++;
            }
            else{
                drive.tankRun(-0.40, 0.37);
            }
            break;

            case 10:                                                                 //foward and intake 3rd ball
            if(!intake.cargoCheck() || encoder.getPosition() >= 98){                                       //start: 2 - end: 94
                drive.tankRun(0, 0);
                threeBallHighLowCounter++;
            }
            else{
                intake.setIntakeMode();
                drive.tankRun(0.70, 0.70);
            }
            break;

            case 11:
            if(gyro.getYaw() > -143f && gyro.getYaw() < -137f){                              //turn right to face hub
                drive.tankRun(0, 0);   
                intake.setStopMode();
                encoder.setPosition(0);
                threeBallHighLowCounter++;
            }
            else{
                drive.tankRun(0.6, -0.6);
            }
            break;

            case 12:                                                              //rev the shooter                         
            if(shooter.checkRPM()){
                threeBallHighLowCounter++;
            }
            else{
                //shooter.setUpperHubShoot();
                shooter.setLowHubShoot();
            }      
            break;
            
            case 13:                                                                              //shoot the 3rd ball
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setStopMode();
                    threeBallHighLowCounter++;
                }
                else{
                    intake.setFeedingMode();
                }
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

            case THREEBALLHIGH:
                twoBall();
            break;

            case THREEBALLHIGHLOW:
                threeLowHighBall();
            break;

        }

        shooter.run();
        intake.run();
    }
}