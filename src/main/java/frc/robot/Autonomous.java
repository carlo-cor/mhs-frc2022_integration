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
    private int twoBallLeftCounter = 0;
    private int twoBallRightCounter = 0;
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
        NOTHING, ONEBALL, TWOBALL_L, TWOBALL_R, THREEBALLLOW, THREEBALLHIGH 
    }

    private routines routineState = routines.NOTHING;

    public void setNothing(){
        routineState = routines.NOTHING;
    }

    public void setOneBall(){
        routineState = routines.ONEBALL;
    }

    public void setTwoBallLeft(){
        routineState = routines.TWOBALL_L;
    }

    public void setTwoBallTRight(){
        routineState = routines.TWOBALL_R;
    }
    
    public void setThreeBallLow(){
        routineState = routines.THREEBALLLOW;
    }

    public void setThreeBallHigh(){
        routineState = routines.THREEBALLHIGH;
    }



    public void display(){
        //SENSOR VALUES:
        SmartDashboard.putNumber("Encoder Counts", encoder.getPosition());
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());

        //ROUTINE COUNTERS:
        SmartDashboard.putNumber("One Ball Counter", oneBallCounter);
        SmartDashboard.putNumber("Two Ball A Counter", twoBallLeftCounter);
        SmartDashboard.putNumber("Two Ball B Counter", twoBallRightCounter);
        SmartDashboard.putNumber("Three Ball High Counter", threeBallHighCounter);
        SmartDashboard.putNumber("Three Ball Low Counter", threeBallLowCounter);
    }

    public void deleteDisplays(){
        SmartDashboard.delete("Encoder Counts");
        SmartDashboard.delete("Gyro Yaw");
        SmartDashboard.delete("One Ball Counter");
        SmartDashboard.delete("Two Ball A Counter");
        SmartDashboard.delete("Two Ball B Counter");
        SmartDashboard.delete("Three Ball High Counter");
        SmartDashboard.delete("Three Ball Low Counter");
    }


    public void reset(){
        limelight.setTrackingMode();
        encoder.setPosition(0);
        gyro.reset();
    
        oneBallCounter = 0;
        twoBallLeftCounter = 0;
        twoBallRightCounter = 0;
        threeBallLowCounter = 0;
        threeBallHighCounter = 0;
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

            case 1:     //rev shooter, bring intake down, and align with upper hub
                if(shooter.checkRPM() && intake.armIsDown() && shooter.checkAligned() && shooter.checkInRange()){
                    drive.arcadeRun(0, 0);
                    encoder.setPosition(0);
                    oneBallCounter++;
                }
                else{
                    //DELETE LATER:
                    shooter.setTesting();
                    shooter.setManual(0.6);
                    //
                    
                    //shooter.setUpperHubShoot();
                    intake.setExtend();
                    drive.arcadeRun(shooter.alignSpeed, shooter.getInRangeSpeed);
                }
            break;

            case 2:     //shoot ball if rpm is within range
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setIntakeStopMode();;
                    oneBallCounter++;
                }

                else{
                    intake.setFeedingMode();
                }
            break;

            case 3:     //bring intake to midway point
                if(intake.atMidway()){
                    intake.setArmStopMode();
                    oneBallCounter++;
                }
                else{
                    intake.setMidway();
                }
        }
    }

    //Routine to taxi, shoot preload into upper hub, intake ball to the right, and shoot that ball into the upper hub
    private void twoBallLeft(){
        switch(twoBallLeftCounter){ 

            case 0:     //taxi off tarmac
                if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(7)){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    twoBallLeftCounter++;
                }
                else{
                    drive.tankRun(-0.6, -0.6);
                }
            break;

            case 1:     //rev the shooter & bring arm down                         
                if(shooter.checkRPM() && intake.armIsDown() && shooter.checkAligned() && shooter.checkInRange()){
                    drive.arcadeRun(0, 0);
                    encoder.setPosition(0);
                    twoBallLeftCounter++;
                }
                else{
                    //DELETE LATER:
                    shooter.setTesting();
                    shooter.setManual(0.6);
                    //
                    
                    //shooter.setUpperHubShoot();
                    intake.setExtend();
                    drive.arcadeRun(shooter.alignSpeed, shooter.getInRangeSpeed);
                }      
            break;
                
            case 2:     //shoot the ball when rpm is within range
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setIntakeStopMode();
                    twoBallLeftCounter++;
                }
                else{
                    intake.setFeedingMode();
                }
            break;
            
            case 3:     //turn to face the cargo ball
                if(gyro.getYaw() > 48f && gyro.getYaw() < 53f ){
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    twoBallLeftCounter++;
                }
                else{
                    if(gyro.getYaw() < 48){
                        drive.tankRun(0.5, -0.5);
                    }
                    else if(gyro.getYaw() > 53){
                        drive.tankRun(-0.5, 0.5);
                    }
                }
            break;

            case 4:     //intake the ball (keeps on moving forward until ball is in or if it travels a certain distance)
                if(!intake.cargoCheck() || encoder.getPosition() >= convertFeetToEncoderCounts(5)){
                    drive.tankRun(0, 0);
                    intake.setIntakeStopMode();
                    twoBallLeftCounter++;
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
                    twoBallLeftCounter++;
                }
                else{
                    drive.tankRun(-0.5, -0.5);
                    twoBallLeftCounter++;
                }
            break;

            case 6:     //turn back to face the upper hub
                if(gyro.getYaw() < 3 && gyro.getYaw() > -3){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    twoBallLeftCounter++;
                }
                else{
                    if(gyro.getYaw() > 3){
                        drive.tankRun(-0.5, 0.5);
                    }
                    else if(gyro.getYaw() < -3){
                        drive.tankRun(0.5, -0.5);
                    }
                }
            break; 

            case 7:     //rev the shooter
                if(shooter.checkRPM() && shooter.checkAligned() && shooter.checkInRange()){
                    drive.arcadeRun(0, 0);
                    encoder.setPosition(0);
                    twoBallLeftCounter++;
                }
                else{
                    //DELETE LATER:
                    shooter.setTesting();
                    shooter.setManual(0.6);
                    //

                    //shooter.setUpperHubShoot();
                    drive.arcadeRun(shooter.alignSpeed, shooter.getInRangeSpeed);
                }
            break;

            case 8: //In case the sensor isn't triggered, feed the ball in regardless for a short time
                if(timer.get() >= 1){
                    timer.stop();
                    timer.reset();
                    twoBallLeftCounter++;
                }
                else{
                    timer.start();
                    intake.setOverrideMode();
                }
            break;

            case 9:     //shoot the ball
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setIntakeStopMode();
                    twoBallLeftCounter++;
                }
                else{
                    intake.setFeedingMode();   
                }
            break;

            case 10:
                if(intake.atMidway()){
                    intake.setArmStopMode();
                    twoBallLeftCounter++;
                }

                else{
                    intake.setMidway();
                }

        }
    }

    private void twoBallRight(){           //USES PORTION OF THE THREE BALL *HIGH* METHOD
        switch(twoBallRightCounter){
            
        }
    }

    private void threeBallLow(){        //TWO BALL HIGH, ONE BALL LOW
        switch(threeBallLowCounter){ 

            case 0: 
                if (Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(1.225)) {
                    drive.tankRun(0, 0); 
                    encoder.setPosition(0); 
                    threeBallLowCounter++; 
                } else {
                    drive.tankRun(-0.6, -0.6); 
                }
            break; 

            case 1:
                if(shooter.checkRPM() && intake.armIsDown()){                 //shoot preload into low hub
                    threeBallLowCounter++;
                }

                else{
                    //DELETE LATER:
                    //shooter.setTesting();
                    //shooter.setManual(0.6);
                    //

                    intake.setExtend();
                    shooter.setLowHubShoot();
                }
            break;

            case 2:
                if(intake.cargoCheck()){
                    intake.setIntakeStopMode();
                    shooter.setStop();
                    threeBallLowCounter++;
                }

                else{
                    intake.setFeedingMode();
                }
            break;

            case 3:                                                 //turn right to ball by the wall
                if(gyro.getYaw() < 142 && gyro.getYaw() > 139){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    threeBallLowCounter++;
                }

                else{
                    //drive.tankRun(0.55, -0.55);
                    
                    if(gyro.getYaw() > 142){
                        drive.tankRun(-0.35, 0.35);
                    }
                    else if(gyro.getYaw() < 139){
                        drive.tankRun(0.55, -0.55);
                    }
                    
                }
            break;

            case 4:                                                   //forward and intake 2nd ball
                if(!intake.cargoCheck() || Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(6.25)){       
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    gyro.reset();
                    threeBallLowCounter++;
                }
                else{
                    intake.setIntakeMode();
                    drive.tankRun(0.65, 0.65);
                }
            break;

            case 5:
                if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(0.8)){           //back up a foot and a half so you dont hit the wall when you turn around
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    threeBallLowCounter++;
                }
                else{
                    drive.tankRun(-0.6, -0.6);
                }
            break;                                                                  

            case 6:
                if(gyro.getYaw() > -155f && gyro.getYaw() < -150){                              //turn left to face hub
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    threeBallLowCounter++;
                }
                else{
                    
                    if(gyro.getYaw() > -150){
                        drive.tankRun(-0.75, 0.75);
                    }
                    else if(gyro.getYaw() < -155){
                        drive.tankRun(0.75, -0.75);
                    }
                }
            break;

            case 7:                                                              //rev the shooter                         
                if(shooter.checkRPM() && shooter.checkAligned() && shooter.checkInRange()){
                    drive.arcadeRun(0, 0);
                    encoder.setPosition(0);
                    threeBallLowCounter++;
                }
                else{
                    //DELETE LATER:
                        shooter.setTesting();
                        shooter.setManual(0.6);
                    //

                    //shooter.setUpperHubShoot();
                    drive.arcadeRun(shooter.alignSpeed, 0);
                }      
            break;
            
            case 8:                                                                              //shoot the 2nd ball
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setIntakeStopMode();
                    threeBallLowCounter++;
                }
                else{
                    intake.setFeedingMode();
                }
            break;                                                                                  //gyro at -7.6

            case 9:                                                                                 //gyro to -100
                if(gyro.getYaw() > 112f && gyro.getYaw() < 114f){                              //turn left to face third ball   [CONSIDER MAKING THIS FASTER!]
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    threeBallLowCounter++;
                }
                else{
                    //drive.tankRun(-0.45, 0.45);
                    
                    if(gyro.getYaw() > 114f || gyro.getYaw() < 0){
                        drive.tankRun(-0.45, 0.45);
                    }
                    else if(gyro.getYaw() < 112f){
                        drive.tankRun(0.35, -0.35);
                    }
                    
                }
            break;

            case 10:                                                                 //foward and intake 3rd ball
                if(!intake.cargoCheck() || Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(9)){                                       //start: 2 - end: 94
                    drive.tankRun(0, 0);
                    intake.setIntakeStopMode();
                    threeBallLowCounter++;
                }
                else{
                    intake.setIntakeMode();
                    drive.tankRun(0.9, 0.9);
                }
            break;

            case 11:
                if(gyro.getYaw() > -135f && gyro.getYaw() < -130f){                              //turn right to face hub
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    threeBallLowCounter++;
                }
                else{
                    if(gyro.getYaw() > -130f && gyro.getYaw() < 0){
                        drive.tankRun(-0.75, 0.75);
                    }
                    else if(gyro.getYaw() < -135f || gyro.getYaw() > 0){
                        drive.tankRun(0.75, -0.75);
                    }
                }
            break;

            case 12:                                                              //rev the shooter                         
                if(shooter.checkRPM() && shooter.checkAligned() && shooter.checkInRange()){
                    drive.arcadeRun(0, 0);
                    encoder.setPosition(0);
                    threeBallLowCounter++;
                }
                else{
                    //DELETE LATER
                    shooter.setTesting();
                    shooter.setManual(0.6);
                    //

                    //shooter.setUpperHubShoot();
                    drive.arcadeRun(shooter.alignSpeed, shooter.getInRangeSpeed);
                }      
            break;

            case 13:
                if(timer.get() >= 1){
                    timer.stop();
                    timer.reset();
                    threeBallLowCounter++;
                }
                else{
                    timer.start();
                    intake.setOverrideMode();
                }
            break;

            case 14:                                                                              //shoot the 3rd ball
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setIntakeStopMode();
                    threeBallLowCounter++;
                }
                else{
                    intake.setFeedingMode();
                }
            break;

            case 15:
                if(intake.atMidway()){
                    intake.setArmStopMode();
                    threeBallLowCounter++;
                }
                else{
                    intake.setMidway();
                }
            break;
        }
    }

    private void threeBallHigh(){
        switch(threeBallHighCounter){
            
            case 0: 
                if (Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(0.5)) {
                    drive.tankRun(0, 0); 
                    encoder.setPosition(0); 
                    threeBallHighCounter++; 
                } else {
                    drive.tankRun(-0.6, -0.6); 
                }
            break; 
            

            case 1:
                if(shooter.checkRPM() && intake.armIsDown() && shooter.checkAligned() && shooter.checkInRange()){                 //shoot preload into high hub
                    drive.arcadeRun(0, 0);
                    encoder.setPosition(0);
                    threeBallHighCounter++;
                }

                else{
                    //DELETE LATER:
                    shooter.setTesting();
                    shooter.setManual(0.6);
                    //

                    intake.setExtend();
                    //shooter.setUpperHubShoot();
                    drive.arcadeRun(shooter.alignSpeed, shooter.getInRangeSpeed);
                }
            break;

            case 2:
                if(intake.cargoCheck()){
                    intake.setIntakeStopMode();
                    shooter.setStop();
                    threeBallHighCounter++;
                }

                else{
                    intake.setFeedingMode();
                }
                   
            break;

            case 3: //turn right to ball by the wall
                 if(gyro.getYaw() > 178 && gyro.getYaw() < -178){
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    threeBallHighCounter++;
                }

                else{
                    if(gyro.getYaw() < 178 && gyro.getYaw() > 0){
                        drive.tankRun(0.6, -0.6);
                    }
                    else if(gyro.getYaw() > -178 && gyro.getYaw() < 0){
                        drive.tankRun(-0.6, 0.6);
                    }
                }                                                 
            break;

            
            case 4:                                                   //forward and intake 2nd ball
                if(!intake.cargoCheck() || Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(2)){       
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    threeBallHighCounter++;
                }
                else{
                    intake.setIntakeMode();
                    drive.tankRun(0.65, 0.65);
                }
            break;
            /*
            case 5:
                if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(1.5)){           //back up a foot and a half so you dont hit the wall when you turn around
                    drive.tankRun(0, 0);
                    encoder.setPosition(0);
                    gyro.reset();
                    threeBallHighCounter++;
                }
                else{
                    drive.tankRun(-0.6, -0.6);
                }
            break;                                                                  //reset gyro here and we should know what the gyro needs to be for the third ball
            case 6:
                if(gyro.getYaw() > -155f && gyro.getYaw() < -150){                              //turn left to face hub
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    threeBallHighCounter++;
                }
                else{
                    
                    if(gyro.getYaw() > -145){
                        drive.tankRun(-0.6, 0.6);
                    }
                    else if(gyro.getYaw() < -150){
                        drive.tankRun(0.6, -0.6);
                    }
                }
            break;
            case 7:                                                              //rev the shooter                         
                if(shooter.checkRPM() && shooter.checkAligned() && shooter.checkInRange()){
                    drive.arcadeRun(0, 0);
                    encoder.setPosition(0);
                    threeBallHighCounter++;
                }
                else{
                    //DELETE LATER:
                        shooter.setTesting();
                        shooter.setManual(0.6);
                    //
                    //shooter.setUpperHubShoot();
                    drive.arcadeRun(shooter.alignSpeed, shooter.getInRangeSpeed);
                }      
            break;
            
            case 8:                                                                              //shoot the 2nd ball
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setIntakeStopMode();
                    threeBallHighCounter++;
                }
                else{
                    intake.setFeedingMode();
                }
            break;                                                                                  //gyro at -7.6
            case 9:                                                                                 //gyro to -100
                if(gyro.getYaw() > 120f && gyro.getYaw() < 125f){                              //turn left to face third ball   [CONSIDER MAKING THIS FASTER!]
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    threeBallHighCounter++;
                }
                else{
                    if(gyro.getYaw() > 125f || gyro.getYaw() < 0){
                        drive.tankRun(-0.6, 0.6);
                    }
                    else if(gyro.getYaw() < 120){
                        drive.tankRun(0.6, -0.6);
                    }
                }
            break;
            case 10:                                                                 //foward and intake 3rd ball
                if(!intake.cargoCheck() || Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(8.83)){                                       //start: 2 - end: 94
                    drive.tankRun(0, 0);
                    intake.setIntakeStopMode();
                    threeBallHighCounter++;
                }
                else{
                    intake.setIntakeMode();
                    drive.tankRun(0.8, 0.8);
                }
            break;
            case 11:
                if(gyro.getYaw() > -135f && gyro.getYaw() < -130f){                              //turn right to face hub
                    drive.tankRun(0, 0);   
                    encoder.setPosition(0);
                    threeBallHighCounter++;
                }
                else{
                    if(gyro.getYaw() > -130f){
                        drive.tankRun(0.6, -0.6);
                    }
                    else if(gyro.getYaw() < -135f){
                        drive.tankRun(0.6, -0.6);
                    }
                }
            break;
            case 12:                                                              //rev the shooter                         
                if(shooter.checkRPM() && shooter.checkAligned() && shooter.checkInRange()){
                    drive.arcadeRun(0, 0);
                    encoder.setPosition(0);
                    threeBallHighCounter++;
                }
                else{
                    //DELETE LATER
                    shooter.setTesting();
                    shooter.setManual(0.6);
                    //
                    //shooter.setUpperHubShoot();
                    drive.arcadeRun(shooter.alignSpeed, shooter.getInRangeSpeed);
                }      
            break;
            
            case 13:                                                                              //shoot the 3rd ball
                if(intake.cargoCheck()){
                    shooter.setStop();
                    intake.setIntakeStopMode();
                    threeBallHighCounter++;
                }
                else{
                    intake.setFeedingMode();
                }
            break;
            case 14:
                if(intake.atMidway()){
                    intake.setArmStopMode();
                }
                else{
                    intake.setMidway();
                }
            break;
            */
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

            case TWOBALL_L:
                twoBallLeft();
            break;

            case TWOBALL_R:
                twoBallRight();
            break;

            case THREEBALLLOW:
                threeBallLow();
            break;

            case THREEBALLHIGH:
                threeBallHigh();
            break;
        }

        limelight.run();
        shooter.run();
        intake.intakeRun();
    }
}