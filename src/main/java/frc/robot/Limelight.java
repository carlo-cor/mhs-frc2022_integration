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

    public Limelight(){
        limelight = null;
        camModeEntry = null;
        ledModeEntry = null;
        checkTargetEntry = null;
        xOffsetEntry = null;
        yOffseEntry = null;
    }

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

    private NetworkTable getLimelight(){
        if(limelight == null){
            limelight = NetworkTableInstance.getDefault().getTable("limelight");
        }

        return limelight;
    }

    private NetworkTableEntry getCamModeEntry(){
        if(getLimelight() == null){
            camModeEntry = null;
        }
        else{
            camModeEntry = getLimelight().getEntry("camMode");
        }

        return camModeEntry;

    }

    private NetworkTableEntry getLEDModeEntry(){
        if(getLimelight() == null){
            ledModeEntry = null;
        }
        else{
            ledModeEntry = getLimelight().getEntry("ledMode");
        }

        return ledModeEntry;
    }

    private NetworkTableEntry getTargetEntry(){
        if(getLimelight() == null){
            checkTargetEntry = null;
        }
        else{
            checkTargetEntry = getLimelight().getEntry("tv");
        }

        return checkTargetEntry;
    }

    private NetworkTableEntry getXOffsetEntry(){
        if(getLimelight() == null){
            xOffsetEntry = null;
        }
        else{
            xOffsetEntry = getLimelight().getEntry("tx");
        }

        return xOffsetEntry;
    }

    private NetworkTableEntry getYOffsetEntry(){
        if(getLimelight() == null){
            yOffseEntry = null;
        }
        else{
            yOffseEntry = getLimelight().getEntry("ty");
        }

        return yOffseEntry;
    }

    private void drivingMode(){
        if(getCamModeEntry() != null && getLEDModeEntry() != null){
            getCamModeEntry().setDouble(1);
            getLEDModeEntry().setDouble(1);
        }
    }

    private void trackingMode(){
        if(getCamModeEntry() != null && getLEDModeEntry() != null){
            getCamModeEntry().setDouble(0);
            getLEDModeEntry().setDouble(3);
        }
    }

    public boolean checkTargetSeen(){
        return getTargetEntry().getDouble(0) == 1;
    }
        
    public double getXOffset(){
        return getXOffsetEntry().getDouble(0);
    }

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