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
    private Limelight limelight;

    //COUNTER VARIABLES:
    private int oneBallCounter = 0;
    private int twoBallCounter = 0;
    private int threeBallHighCounter = 0;
    private int threeBallLowCounter = 0;

    //CONSTANTS:
    private final double encCountsPerFoot = 11.1029532;
    
    public Autonomous(Drive newDrive, Shooter newShooter, Intake newIntake, RelativeEncoder newEncoder, AHRS newGyro, Limelight newLimelight){
        drive = newDrive;       
        shooter = newShooter;
        intake = newIntake;
        encoder = newEncoder;
        gyro = newGyro;
        limelight = newLimelight;
        timer = new Timer();
    }

    private enum routines{
        NOTHING, ONEBALL, TWOBALL, THREEBALLHIGH, THREEBALLLOW
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

    public void setThreeBallLow(){
        routineState = routines.THREEBALLLOW;
    }


    public void display(){
        //SENSOR VALUES:
        SmartDashboard.putNumber("Encoder Counts", encoder.getPosition());
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());

        //ROUTINE COUNTERS:
        SmartDashboard.putNumber("One Ball Counter", oneBallCounter);
        SmartDashboard.putNumber("Two Ball Counter", twoBallCounter);
        SmartDashboard.putNumber("Three Ball High Counter", threeBallHighCounter);
        SmartDashboard.putNumber("Three Ball High Low Counter", threeBallLowCounter);
    }

    public void reset(){
        limelight.setTrackingMode();
        encoder.setPosition(0);
        gyro.reset();
    
        oneBallCounter = 0;
        twoBallCounter = 0;
        threeBallHighCounter = 0;
        threeBallLowCounter = 0;
    }

    //Converts feet into encoder counts
    private double convertFeetToEncoderCounts(double feet){
        return feet * encCountsPerFoot;
    }

    //Routine to do nothing
    private void nothing(){

    }

    //Routine to taxi and shoot into the upper hub
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
                    //shooter.setUpperHubShoot
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

    //Routine to taxi, shoot preload into upper hub, intake ball to the right, and shoot that ball into the upper hub
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

            case 4:     //intake the ball (keeps on moving forward until ball is in or if it travels a certain distance)
                if(!intake.cargoCheck() || encoder.getPosition() >= convertFeetToEncoderCounts(5)){
                    drive.tankRun(0, 0);
                    intake.setStopMode();
                    twoBallCounter++;
                }
                else{
                    intake.setIntakeMode();
                    drive.tankRun(0.5, 0.5);
                }
            break;

            case 5:     //back up to initial position (SKIPPED FOR NOW)
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

            case 8: //In case the sensor isn't triggered, feed the ball in regardless
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
        switch(threeBallLowCounter){   
            case 0: 
                if (encoder.getPosition() <= -13.6) {
                    drive.tankRun(0, 0); 
                    encoder.setPosition(0); 
                    threeBallLowCounter++; 
                } else {
                    drive.tankRun(-0.6, -0.6); 
                }
            break; 

            case 1:
                if(shooter.checkRPM()){                 //shoot preload into low hub
                    threeBallLowCounter++;
                }

                else{
                    shooter.setLowHubShoot();
                }
            break;

            case 2:
                if(intake.cargoCheck()){
                    intake.setStopMode();
                    shooter.setStop();
                    threeBallLowCounter++;
                }

                else{
                    intake.setFeedingMode();
                }
            break;

            case 3:                                                 //turn right to ball by the wall
                if(gyro.getYaw() < 146 && gyro.getYaw() > 140){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    threeBallLowCounter++;
                }

                else{
                    drive.tankRun(0.45, -0.45);
                }
            break;

            case 4:                                                   //forward and intake 2nd ball
                if(!intake.cargoCheck() || encoder.getPosition() > 71.5){       
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    threeBallLowCounter++;
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
                    threeBallLowCounter++;
                }
                else{
                    drive.tankRun(-0.53, -0.50);
                }
            break;                                                                  //reset gyro here and we should know what the gyro needs to be for the third ball

            case 6:
                if(gyro.getYaw() > -143f && gyro.getYaw() < -137){                              //turn left to face hub
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    threeBallLowCounter++;
                }
                else{
                    drive.tankRun(-0.55, 0.52);
                }
            break;

            case 7:                                                              //rev the shooter                         
                if(shooter.checkRPM()){
                    threeBallLowCounter++;
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
                    threeBallLowCounter++;
                }
                else{
                    intake.setFeedingMode();
                }
            break;                                                                                  //gyro at -7.6

            case 9:                                                                                 //gyro to -100
                if(gyro.getYaw() > 117f && gyro.getYaw() < 123f){                              //turn left to face third ball   [CONSIDER MAKING THIS FASTER!]
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    threeBallLowCounter++;
                }
                else{
                    drive.tankRun(-0.40, 0.37);
                }
            break;

            case 10:                                                                 //foward and intake 3rd ball
                if(!intake.cargoCheck() || encoder.getPosition() >= 98){                                       //start: 2 - end: 94
                    drive.tankRun(0, 0);
                    threeBallLowCounter++;
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
                    threeBallLowCounter++;
                }
                else{
                    drive.tankRun(0.6, -0.6);
                }
            break;

            case 12:                                                              //rev the shooter                         
                if(shooter.checkRPM()){
                    threeBallLowCounter++;
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
                    threeBallLowCounter++;
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

            break;

            case THREEBALLLOW:
                threeLowHighBall();
            break;

        }

        limelight.run();
        shooter.run();
        intake.run();
    }
}