package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.CounterBase;

public class TalonEncoder implements CounterBase {

	private TalonSRX srx;

	public TalonEncoder(TalonSRX srx) {
		if(srx == null)
			throw new RuntimeException("You didn't pass a value in!!!!!!!!!!!!!!!!!!!");
		this.srx = srx;
		
	}
	
	@Override
	public int get() {
		return srx.getSensorCollection().getQuadraturePosition();
	}

	@Override
	public void reset() {
		srx.getSensorCollection().setQuadraturePosition(0, 5);

	}

	@Override
	public double getPeriod() {
		throw new RuntimeException();
	}

	@Override
	public void setMaxPeriod(double maxPeriod) {
		throw new RuntimeException();

	}

	@Override
	public boolean getStopped() {
		throw new RuntimeException();
	}

	@Override
	public boolean getDirection() {
		throw new RuntimeException();
	
	}

}