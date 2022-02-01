package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Shifter {
    private DoubleSolenoid shifter; 

    public Shifter(DoubleSolenoid newShifter) { 
         shifter = newShifter; 
    }

    public void setSpeed() {
        shifter.set(DoubleSolenoid.Value.kForward); 
    }

    public void setPower() {
        shifter.set(DoubleSolenoid.Value.kReverse); 
    }
}