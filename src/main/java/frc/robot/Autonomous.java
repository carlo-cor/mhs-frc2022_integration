package frc.robot;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
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
    private int threeBallHighCounter = 0;
    private int threeBallHighLowCounter = 0;

    private Timer autoTimer;

    //CONSTANTS:
    private final double encCountsPerFoot = 11.1029532;
    
    public Autonomous(Drive newDrive, Shooter newShooter, Intake newIntake, RelativeEncoder newEncoder, AHRS newGyro){
        drive = newDrive;       
       // shooter = newShooter;
       // intake = newIntake;
        encoder = newEncoder;
        gyro = newGyro;
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
        SmartDashboard.putNumber("Three Ball Counter", threeBallHighCounter);
        SmartDashboard.putNumber("Encoder Counts", encoder.getPosition());
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
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
/*
    private void oneBall(){
        switch(oneBallCounter){
            case 0:     //rev shooter
                if(/*shooter.checkIfRPMWithinRange()true){
                    shooter.setSpeedManual(-0.4);
                    oneBallCounter++;
                }
                else{
                    //shooter.setUpperHubShoot();
                }
            break;

            case 1:     //shoot ball if rpm is within range
                if(/*!intake.cargoCheck()true){
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
                if(/*shooter.checkIfRPMWithinRange()true){
                    shooter.setSpeedManual(-0.4);
                    twoBallCounter++;
                }
                else{
                    //shooter.setUpperHubShoot();
                }      
            break;

            case 1:     //shoot the ball
                if(/*!intake.cargoCheck()true){
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
                if(/*intake.cargoCheck() Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(4)){
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
                if(/*shooter.checkIfRPMWithinRange()true){
                    shooter.setSpeedManual(-0.4);
                    twoBallCounter++;
                }
                else{
                    //shooter.setUpperHubShoot();
                }
            break;

            case 6:     //shoot the ball
                if(/*intake.cargoCheck()true){
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

    */
    private void threeLowHighBall(){        //TWO BALL HIGH, ONE BALL LOW
        switch(threeBallHighLowCounter){   
            case 0: 
            if (encoder.getPosition() <= convertFeetToEncoderCounts(-1.22489934)) {
                drive.tankRun(0, 0); 
                encoder.setPosition(0); 
                threeBallHighLowCounter++; 
            } else {
                drive.tankRun(-0.60, -0.57); 
            }
            break; 

            case 1:
           // if(shooter.checkRPM()){                 //shoot preload into low hub
                threeBallHighLowCounter++;
            //}

            //else{
                //shooter.setLowHubShoot();
            //}
            break;

            case 2:
            //if(intake.cargoCheck()){
                //intake.setStopMode();
                //shooter.setStop();
                threeBallHighLowCounter++;
            //}

            //else{
                //intake.setFeedingMode();
            //}
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
            if(/*!intake.cargoCheck() || */encoder.getPosition() >= convertFeetToEncoderCounts(6/*.43972813*/)){
                drive.tankRun(0, 0);
                encoder.setPosition(0);
                threeBallHighLowCounter++;
            }

            else{
                //intake.setIntakeMode();
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
            //if(shooter.checkRPM()){
                threeBallHighLowCounter++;
            //}
            //else{
                //shooter.setUpperHubShoot();
                //shooter.setLowHubShoot();
            //}      
            break;
            
            case 8:                                                                              //shoot the 2nd ball
                //if(intake.cargoCheck()){
                    //shooter.setStop();
                    //intake.setStopMode();
                    threeBallHighLowCounter++;
                //}
                //else{
                    //intake.setFeedingMode();
                //}
            break;                                                                                  //gyro at -7.6

            case 9:                                                                                 //gyro to -100
            if(gyro.getYaw() > 124f && gyro.getYaw() < 130f){                              //turn left to face third ball   [CONSIDER MAKING THIS FASTER!]
                drive.tankRun(0, 0);   
                encoder.setPosition(0);
                threeBallHighLowCounter++;
            }
            else{
                drive.tankRun(-0.50, 0.47);
            }
            break;

            case 10:                                                                 //foward and intake 3rd ball
            if(/*!intake.cargoCheck() || */encoder.getPosition() >= convertFeetToEncoderCounts(8.82648051)){                                       //start: 2 - end: 94
                drive.tankRun(0, 0);
                threeBallHighLowCounter++;
            }
            else{
                //intake.setIntakeMode();
                drive.tankRun(0.70, 0.70);
            }
            break;

            case 11:
            if(gyro.getYaw() > -143f && gyro.getYaw() < -137f){                              //turn right to face hub
                drive.tankRun(0, 0);   
                //intake.setStopMode();
                encoder.setPosition(0);
                threeBallHighLowCounter++;
            }
            else{
                drive.tankRun(0.6, -0.6);
            }
            break;

            case 12:                                                              //rev the shooter                         
            //if(shooter.checkRPM()){
                threeBallHighLowCounter++;
            //}
            //else{
                //shooter.setUpperHubShoot();
                //shooter.setLowHubShoot();
            //}      
            break;
            
            case 13:                                                                              //shoot the 3rd ball
                //if(intake.cargoCheck()){
                    //shooter.setStop();
                    //intake.setStopMode();
                    threeBallHighLowCounter++;
                //}
                //else{
                    //intake.setFeedingMode();
                //}
            break;


        }
    }

    
    private void threeHighBall(){
     /*   switch(threeBallHighCounter){            // reset navx yaw
            case 0:     //taxi off tarmac
            if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(6)){           //back up 6ish feet from beginning
                drive.tankRun(0, 0);
                encoder.setPosition(0);
                threeBallHighCounter++;
            }
            else{
                drive.tankRun(-0.45, -0.42);
            }
        break;

        case 1:                                                                              //rev the shooter                         
            if(shooter.checkRPM()){
                threeBallHighCounter++;
            }
            else{
                //shooter.setUpperHubShoot();
                shooter.setLowHubShoot();
            }      
        break;
            
        case 2:                                                                              //shoot the ball
            if(intake.cargoCheck()){
                shooter.setStop();
                intake.setStopMode();
                threeBallHighCounter++;
            }
            else{
                intake.setFeedingMode();
            }
        break;
        
        case 3:                                                                          //turn left to face the first cargo ball to pick up
            if(gyro.getYaw() < -35f && gyro.getYaw() > -45f){
                drive.tankRun(0, 0);   
                encoder.setPosition(0);
                threeBallHighCounter++;
            }
            else{
                drive.tankRun(-0.55, 0.52);
            }
        break;

        case 4:                                                                         //forward and intake 2nd ball
            if(!intake.cargoCheck()){
                drive.tankRun(0, 0);
                intake.setStopMode();
                threeBallHighCounter++;
            }
            else{
                intake.setIntakeMode();
                drive.tankRun(0.4, 0.37);
            }
        break;

        case 5:                                                                          //turn right to face back to the hub
        if(gyro.getYaw() > -5f && gyro.getYaw() < 0f){
            drive.tankRun(0, 0);   
            encoder.setPosition(0);
            threeBallHighCounter++;
        }
        else{
            drive.tankRun(0.55, -0.52);
        }
        break;

        case 6:                                                                              //rev the shooter                         
        if(shooter.checkRPM()){
            threeBallHighCounter++;
        }
        else{
            //shooter.setUpperHubShoot();
            shooter.setLowHubShoot();
        }      
        break;
        
        case 7:                                                                              //shoot the ball
            if(intake.cargoCheck()){
                shooter.setStop();
                intake.setStopMode();
                threeBallHighCounter++;
            }
            else{
                intake.setFeedingMode();
            }
        break;

        case 8:
        if(gyro.getYaw() < 65f && gyro.getYaw() > 55f){                                     //turn right to face 3rd ball
            drive.tankRun(0, 0);   
            encoder.setPosition(0);
            threeBallHighCounter++;
        }
        else{
            drive.tankRun(0.55, -0.52);
        }
        break;
    
        case 9:                                                                         //forward and intake 3rd ball
            if(!intake.cargoCheck()){
                drive.tankRun(0, 0);
                intake.setStopMode();
                threeBallHighCounter++;
            }
            else{
                intake.setIntakeMode();
                drive.tankRun(0.4, 0.37);
            }
        break;

        case 10:
            if(Math.abs(encoder.getPosition()) >= convertFeetToEncoderCounts(2)){           //back up 6ish feet from beginning
                drive.tankRun(0, 0);
                encoder.setPosition(0);
                threeBallHighCounter++;
            }
            else{
                drive.tankRun(-0.45, -0.42);
            }
        break;

        case 11:                                                                            //turn left to face the hub
            if(gyro.getYaw() < -35f && gyro.getYaw() > -45f){
                drive.tankRun(0, 0);   
                encoder.setPosition(0);
                threeBallHighCounter++;
            }
            else{
                drive.tankRun(-0.55, 0.52);
            }
        break;

        case 12:                                                                              //rev the shooter                         
        if(shooter.checkRPM()){
            threeBallHighCounter++;
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
                threeBallHighCounter++;
            }
            else{
                intake.setFeedingMode();
            }
        break;
        

            
            rev & shoot;
            turn right 160 degrees to ball by the wall
            forward and intake
            backward so we dont hit the wall
            turn left 160 ish degrees - facing the hub
            shoot!!!!!!!!!!!!!!!!!!!!!!
            turn left 60 degrees - facing othe rball to pick up
            forward and intake!!! - wincy
            turn right 90 degrees - facing hub
            shoot; -wincy perez - 
            
            
        }*/
    } 

    

    public void run(){
        switch(routineState){
            case NOTHING:
                nothing();
            break;

            case ONEBALL:
                //oneBall();
            break;

            case TWOBALL:
                //twoBall();
            break;

            case THREEBALLHIGH:
                threeHighBall();
            break;

            case THREEBALLHIGHLOW:
                threeLowHighBall();

        }

        //shooter.run();
        //intake.run();
    }
}
 