package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive {
    private MotorControllerGroup leftSide; 
    private MotorControllerGroup rightSide; 
    private DifferentialDrive differentialDrive; 

    // uncomment leftBack and rightBack motor controllers 
    public Drive(MotorController leftFront, MotorController leftBack, MotorController rightFront, MotorController rightBack) {
        leftSide = new MotorControllerGroup(leftFront, leftBack); 
        rightSide = new MotorControllerGroup(rightFront, rightBack); 
        differentialDrive = new DifferentialDrive(leftSide, rightSide); 
    }

    private double deadzone(double dJoystickValue) {
        if(Math.abs(dJoystickValue) < 0.2) {
            return 0; 
        }
        return dJoystickValue; 
    }

    public void arcadeDrive(double dSpeed, double dRotation) {
        differentialDrive.arcadeDrive(deadzone(dSpeed), deadzone(-dRotation)); 
    }

    public void tankDrive(double dLeftSpeed, double dRightSpeed) {
        differentialDrive.tankDrive(deadzone(dLeftSpeed), deadzone(-dRightSpeed)); 
    }
}