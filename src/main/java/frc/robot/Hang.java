package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

public class Hang {
    
    /////////////////////////////////////////////
    //                                         //
    //                VARIABLES                //
    //                                         //
    ///////////////////////////////////////////// 
    
    //ELEVATOR
    private HangElevator elevator;

    //PIVOT
    private HangPivot pivot;

    //COUNTERS AND OTHER VARIABLES
    public int setUpMidCount = 0;
    private int setUpHighCount = 0; 

    /////////////////////////////////////////////
    //                                         //
    //              CONSTRUCTOR                //
    //                                         //
    /////////////////////////////////////////////

    public Hang (HangPivot Pivot, HangElevator Elevator){
        elevator = Elevator;
        pivot = Pivot;
    }

    /////////////////////////////////////////////
    //                                         //
    //               ENUMERATIONS              //
    //                                         //
    /////////////////////////////////////////////

    //PIVOT ENUMERATIONS
    private enum hangStates{
        MIDHANG, HIGHHANG, PIVOTMANUAL, ELEVATORMANUAL, TESTING, NOTHING
    }
    
    private hangStates hangMode = hangStates.NOTHING; 

    public void setMidHang() {
        hangMode = hangStates.MIDHANG; 
    }

    public void setHighHang() {
        hangMode = hangStates.HIGHHANG; 
    }

    public void setPivotManual() {
        hangMode = hangStates.PIVOTMANUAL; 
    }

    public void setElevatorManual() {
        hangMode = hangStates.ELEVATORMANUAL; 
    }

    public void setTesting() {
        hangMode = hangStates.TESTING; 
    }

    public void setNothing() {
        hangMode = hangStates.NOTHING; 
    }

    /////////////////////////////////////////////
    //                                         //
    //                 METHODS                 //
    //                                         //
    /////////////////////////////////////////////

    public void resetCounters(){
        setUpMidCount = 0;
        setUpHighCount = 0; 
    }    

    private void testing(){

    }

    private void midHangGrab() {
        switch(setUpMidCount) {
            case 0: 
            //pivot outward 
            if ((pivot.backLimitTouched() || pivot.outwardEncReached())) {      //If the back limit of pivot is touched OR back enc. limit is reached, STOP
                pivot.setStop();
                setUpMidCount++; 
            } else {                                                            //Else, pivot outward
                pivot.setPivOutward(); 
            }
            break; 

            case 1: 
            //elevator extend 
            if (elevator.topLimitTouched() || elevator.topEncoderLimitReached()) {      //If the top limit of elevator is touched || enc limit is reached, STOP
                elevator.setElevatorStop();
                setUpMidCount++; 
            } else {
                if (!elevator.topLimitTouched()) {                                      //else if top limit isn't touched but close to top, extend slowly 
                    elevator.setElevatorExtendSlow();
                } else {
                    elevator.setElevatorExtend();                                         // else extend at normal speed 
                }
            }
            break; 

            case 2: 
            // elevator retract 
            if (elevator.bottomLimitTouched() || elevator.botEncoderLimitReached()) {   // if bottom limit is touched || bottom encoder limit is reached, 
                elevator.setElevatorStop();   
                setUpMidCount++;                                           // stop
            } else {
                if(!elevator.bottomLimitTouched()) {                                    // else if close to bottom limit 
                    elevator.setElevatorRetractSlow();                                  // retract slowly 
                } else {
                    elevator.setElevatorRetract();                                      // else retract at normal speed 
                }
            }
            break; 

            case 3: 
            //pivot to mid
            if (!pivot.middleEncReached() ) {                                      // if middle encoder limit isn't reached 
                pivot.setPivInward();                                              // pivot inward 
            } else {
                pivot.setStop();                                                   // else stop
                setUpMidCount++; 
            }
            break; 

            case 4: 
            //elevator extend
            if (!elevator.topLimitTouched()|| !elevator.topEncoderLimitReached()) {  // if neither top limit is reached 
                elevator.setElevatorExtend();                                        // extend at normal speed 
            }
            else {

                if(!elevator.topLimitTouched()){                                    // else if close to top limit 
                    elevator.setElevatorExtendSlow();                               // extend slowly 
                }

                else{
                    elevator.setElevatorStop();                                     // else stop 
                }
                
            }
            break; 
        }
    }

    private void highHangGrab(){
        switch(setUpHighCount){
            case 0: 
            // extend elevator (some)
            if (!elevator.topLimitTouched() && !elevator.topEncoderLimitReached()) {    // if top limit or small encoder limit isn't reached
                elevator.setElevatorExtend();                                           // extend at a normal speed 
            } else {
                elevator.setElevatorStop();                                             // else stop
                setUpHighCount++; 
            }
            break; 

            case 1: 
            //pivot inwards 
            if (!pivot.inwardEncReached() || !pivot.frontLimitTouched()){           // if neither inward limit is reached 
                pivot.setPivInward();                                               // pivot inward 
            }

            else{
                pivot.setStop();                                                    // else stop 
                setUpHighCount++;
            }
            break; 

            case 2: 
            //elevator extend 
            if (!elevator.topLimitTouched() || !elevator.topEncoderLimitReached()) {   // if neither top limit is reached 
                elevator.setElevatorExtend();                                          // extend at normal speed 
            } 
            
            else {
                if(!elevator.topLimitTouched()){                                      // else if close to top limit 
                    elevator.setElevatorExtendSlow();                                 // extend slowly 
                }

                else{
                    elevator.setElevatorStop();                                       //else stop
                    setUpHighCount++;
                }
                
            }
            break; 

            case 3: 
            //pivot outwards
            if (!pivot.backLimitTouched() && pivot.isGrabbingHigh()) {       // if back limit isn't touched or encoder limit for grabbing high rung isnt reached 
                pivot.setPivOutward();                                       // pivot outward 
            } 
            else {
                pivot.setStop();                                            // else stop 
                setUpHighCount++; 
            }
            break; 

            case 4: 
            //elevator retract 
            if (!elevator.bottomLimitTouched() && !elevator.botEncoderLimitReached()) {   // if neither bottom limit is reached 
                elevator.setElevatorRetract();                                            // retract at normal speed 
            }  
            else {
                if(!elevator.bottomLimitTouched()) {                                    // else if close to bottom limit 
                    elevator.setElevatorRetractSlow();                                  // retract slowly 
                } else {
                    elevator.setElevatorRetract();                                      // else retract at normal speed 
                }                                           
            }
            break; 
        }
    }

    public void manualPivot(double pivSpeed){
        pivot.manualPivot(pivSpeed);
    }

    public void manualElevator(double elevSpeed){
        elevator.manualElev(elevSpeed);
    }

    private void stop(){
        elevator.setElevatorStop();
        pivot.setStop();
    }

    /////////////////////////////////////////////
    //                                         //
    //                   RUN                   //
    //                                         //
    /////////////////////////////////////////////

    
    public void run(){
        //SMART DASHBOARD DISPLAYS
        SmartDashboard.putNumber("MID HANG COUNTER", setUpMidCount); 
        SmartDashboard.putNumber("HIGH HANG COUNTER", setUpHighCount);

        switch(hangMode){
            case MIDHANG:
            midHangGrab();
            break;

            case HIGHHANG:
            highHangGrab();
            break;

            case PIVOTMANUAL:
            break;

            case ELEVATORMANUAL:
            break;

            case TESTING:
            testing();
            break;

            case NOTHING:
            stop();
            break;

        }

        pivot.run(); 
        elevator.run();

    }
}