package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable limelight;
    private NetworkTableEntry camModeEntry;
    private NetworkTableEntry ledModeEntry;
    private NetworkTableEntry checkTargetEntry;
    private NetworkTableEntry xOffsetEntry;
    private NetworkTableEntry yOffseEntry;

    //SETS DEFAULT CONFIG TO NULL
    public Limelight(){
        limelight = null;
        camModeEntry = null;
        ledModeEntry = null;
        checkTargetEntry = null;
        xOffsetEntry = null;
        yOffseEntry = null;
    }

    //TWO STATES: TRACKING OR DRIVING
    private enum state{
        DRIVING, TRACKING
    }

    private state cameraState = state.DRIVING;

    public void setDrivingMode(){
        cameraState = state.DRIVING;
    }

    public void setTrackingMode(){
        cameraState = state.TRACKING;
    }

    //GETS THE LIMELIGHT TABLE ITSELF
    private NetworkTable getLimelight(){
        if(limelight == null){
            limelight = NetworkTableInstance.getDefault().getTable("limelight");
        }

        return limelight;
    }

    //GETS THE CAMERA MODE TABLE ENTRY
    private NetworkTableEntry getCamModeEntry(){
        if(getLimelight() == null){
            camModeEntry = null;
        }
        else{
            camModeEntry = getLimelight().getEntry("camMode");
        }

        return camModeEntry;

    }

    //GETS THE LED MODE TABLE ENTRY
    private NetworkTableEntry getLEDModeEntry(){
        if(getLimelight() == null){
            ledModeEntry = null;
        }
        else{
            ledModeEntry = getLimelight().getEntry("ledMode");
        }

        return ledModeEntry;
    }

    //GETS THE TARGET TABLE ENTRY
    private NetworkTableEntry getTargetEntry(){
        if(getLimelight() == null){
            checkTargetEntry = null;
        }
        else{
            checkTargetEntry = getLimelight().getEntry("tv");
        }

        return checkTargetEntry;
    }

    //GETS THE tx TABLE ENTRY
    private NetworkTableEntry getXOffsetEntry(){
        if(getLimelight() == null){
            xOffsetEntry = null;
        }
        else{
            xOffsetEntry = getLimelight().getEntry("tx");
        }

        return xOffsetEntry;
    }

    //GETS THE ty TABLE ENTRY
    private NetworkTableEntry getYOffsetEntry(){
        if(getLimelight() == null){
            yOffseEntry = null;
        }
        else{
            yOffseEntry = getLimelight().getEntry("ty");
        }

        return yOffseEntry;
    }

    //sets the camera settings to be in driving mode (turns off LEDs and increases exposure)
    private void drivingMode(){
        if(getCamModeEntry() != null && getLEDModeEntry() != null){
            getCamModeEntry().setDouble(1);
            getLEDModeEntry().setDouble(1);
        }
    }

    //sets the camera settings to be in tracking mode (turns on LEDs and decreases exposure)
    private void trackingMode(){
        if(getCamModeEntry() != null && getLEDModeEntry() != null){
            getCamModeEntry().setDouble(0);
            getLEDModeEntry().setDouble(3);
        }
    }

    //returns true if a target is seen
    public boolean checkTargetSeen(){
        return getTargetEntry().getDouble(0) == 1;
    }
        
    //returns how off the target is in the 'x' direction relative to the crosshairs (in degrees)
    public double getXOffset(){
        return getXOffsetEntry().getDouble(0);
    }

    //returns how off the target is in the 'y' direction relative to the crosshairs
    public double getYOffset(){
        return getYOffsetEntry().getDouble(0);
    }

    public void run(){
        switch(cameraState){
            case DRIVING:
                drivingMode();
            break;
            case TRACKING:
                trackingMode();
            break;
        }
    }
}