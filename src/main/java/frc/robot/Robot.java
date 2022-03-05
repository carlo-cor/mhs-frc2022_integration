// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.simulation.DigitalPWMSim;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  
  ///////////////////////////////////////////////////////////
  //                        DRIVE                          //
  ///////////////////////////////////////////////////////////

  /*
  |
  |  LEFT DRIVE | 2 NEO MOTORS: * PORT NEEDED *
  |  RIGHT DRIVE | 2 NEO MOTORS: * PORT NEEDED *
  |
  */

  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;
  private RelativeEncoder relEnc;

  private Drive driveObj;

  ///////////////////////////////////////////////////////////
  //                         HANG                          //
  ///////////////////////////////////////////////////////////

  /*
  |
  |  ELEVATOR ARM | FALCON 500 MOTOR: * PORT NEEDED *
  |  PIVOT ARM | 775 PRO MOTOR: * PORT & MOTOR CONTR. NEEDED *
  |  PIVOT ENCODER | VERSA PLANETARY: * PORT NEEDED *
  |  UPPER SWITCH | DIO LIMIT SWITCH: * PORT NEEDED *
  |  BOTTOM SWITCH | DIO LIMIT SWITCH: * PORT NEEDED * 
  |  FRONT SWITCH | DIO LIMIT SWITCH: * PORT NEEDED *
  |  BACK SWITCH | DIO LIMIT SWITCH: * PORT NEEDED *
  |  WEIGHT ADJUSTER | BOSCH MOTOR: 9 
  |
  */

  private WPI_TalonFX elevMotor;
  private TalonFXSensorCollection elevEnc;
  private WPI_TalonSRX pivotMotor;
  private TalonEncoder pivotEnc;
  private DigitalInput upperLSwitch;
  private DigitalInput bottomLSwitch;
  private DigitalInput frontLSwitch;
  private DigitalInput backLSwitch;
  private WPI_TalonSRX weightAdjMotor;
  private DigitalInput weightAdjChannel;
  private SingleChannelEncoder weightAdjEnc;

  private HangPivot hangPivotObj;
  private HangElevator hangElevObj;
  private WeightAdjuster weightAdjObj;
  private Hang hangObj;

  ///////////////////////////////////////////////////////////
  //                        INTAKE                         //
  ///////////////////////////////////////////////////////////

  /*
  |
  |  INTAKE | BAG MOTOR | TALON SRX: * PORT NEEDED *
  |  HOLD SENSOR | DIO PHOTOELECTRIC SENSOR: * PORT NEEDED *
  |  INTAKE TIMER | DELAY FOR INTAKING
  |
  */

  private WPI_TalonSRX intakeMotor;
  private DigitalInput intakeSensor;
  private Timer intakeTimer;

  private Intake intakeObj;

  ///////////////////////////////////////////////////////////
  //                        SHOOTER                        //
  ///////////////////////////////////////////////////////////

  /*
  |
  |  LIMELIGHT & PID | IMPLEMENTED
  |  SHOOTER | FALCON 500 MOTOR: * PORT NEEDED *
  |
  */

  private WPI_TalonFX shooterMotor;

  private Limelight limelightObj;
  private Shooter shooterObj;

  ///////////////////////////////////////////////////////////
  //                       JOYSTICKS                       //
  ///////////////////////////////////////////////////////////

  /*
  |
  |  NOTE: UNSURE OF WHETHER WE'RE USING TANK OR ARCADE | CHECK IN WITH DRIVERS
  |  BASE JOYSTICK: 0
  |  MECHANISM JOYSTICK: 1 
  |
  */

  private Joystick baseJoy;
  //  private Joystick baseTwoJoy;
  private Joystick mechJoy;

  ///////////////////////////////////////////////////////////
  //                       AUTONOMOUS                      //
  ///////////////////////////////////////////////////////////

  private AHRS gyro;

  private Autonomous autonObj;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  
    /* ! WARNING !: PORTS ARE NOT FINAL AND NEED TO BE CHECKED STILL */

    ///////////////////////////////////////////////////////////
    //                        DRIVE                          //
    ///////////////////////////////////////////////////////////

    frontLeft = new CANSparkMax(7, MotorType.kBrushless);
    backLeft = new CANSparkMax(8, MotorType.kBrushless);
    frontRight = new CANSparkMax(5, MotorType.kBrushless);
    backRight = new CANSparkMax(6, MotorType.kBrushless);
    relEnc = backRight.getEncoder();
    
    driveObj = new Drive(frontLeft, backLeft, frontRight, backRight);
    
    
    ///////////////////////////////////////////////////////////
    //                         HANG                          //
    ///////////////////////////////////////////////////////////

    elevMotor = new WPI_TalonFX(2);
    elevMotor.setNeutralMode(NeutralMode.Brake);
    elevEnc = new TalonFXSensorCollection(elevMotor);
    pivotMotor = new WPI_TalonSRX(4);
    pivotMotor.setNeutralMode(NeutralMode.Brake);
    pivotEnc = new TalonEncoder(pivotMotor);
    upperLSwitch = new DigitalInput(2);
    bottomLSwitch = new DigitalInput(1);
    frontLSwitch = new DigitalInput(0);
    backLSwitch = new DigitalInput(3);
    weightAdjMotor = new WPI_TalonSRX(9);
    weightAdjChannel = new DigitalInput(9); // GET
    weightAdjEnc = new SingleChannelEncoder(weightAdjMotor, weightAdjChannel);
    gyro = new AHRS(Port.kMXP.kOnboard);

    hangPivotObj = new HangPivot(pivotMotor, pivotEnc, gyro, frontLSwitch, backLSwitch);
    hangElevObj = new HangElevator(elevMotor, upperLSwitch, bottomLSwitch, elevEnc);
    weightAdjObj = new WeightAdjuster(weightAdjMotor, weightAdjEnc);
    hangObj = new Hang(hangPivotObj, hangElevObj, weightAdjObj);
    
    ///////////////////////////////////////////////////////////
    //                         INTAKE                        //
    ///////////////////////////////////////////////////////////

    intakeMotor = new WPI_TalonSRX(3);
    intakeSensor = new DigitalInput(4);
    intakeTimer = new Timer();
    intakeObj = new Intake(intakeMotor, intakeSensor, intakeTimer);

    ///////////////////////////////////////////////////////////
    //                         SHOOTER                       //
    ///////////////////////////////////////////////////////////
    
    shooterMotor = new WPI_TalonFX(1);

    limelightObj = new Limelight();
    shooterObj = new Shooter(limelightObj, shooterMotor, driveObj);

    ///////////////////////////////////////////////////////////
    //                         JOYSTICKS                     //
    ///////////////////////////////////////////////////////////

    baseJoy = new Joystick(0);
    //  baseTwoJoy = new Joystick(1);
    mechJoy = new Joystick(1);

    ///////////////////////////////////////////////////////////
    //                         AUTONOMOUS                    //
    ///////////////////////////////////////////////////////////

    autonObj = new Autonomous(driveObj, shooterObj, intakeObj, relEnc, gyro);
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
      autonObj.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
      autonObj.display();
      autonObj.run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(mechJoy.getRawAxis(3) < 0){
      SmartDashboard.putString("MODE: ", "TESTING");

      if(mechJoy.getRawButton(1)){
        intakeObj.setTestingMode();
        intakeObj.setIntakeSpeed(mechJoy.getY());
      }
      else{
        intakeObj.setStopMode();
      }

      if(mechJoy.getRawButton(3)){
        hangElevObj.setElevatorTest();
        hangElevObj.testing(mechJoy.getY());
      }
      else{
        hangElevObj.setElevatorStop();
      }

      
      if(mechJoy.getRawButton(4)){
        hangPivotObj.setTesting();
        hangPivotObj.manualPivot(mechJoy.getY());
        //  hangPivotObj.setPivOutward();
      }

      /*
      else if(mechJoy.getRawButton(12)){
        hangPivotObj.setPivInward();
      }
      */
      
      else{
        hangPivotObj.setStop();
      }
      

      if(mechJoy.getRawButton(5)){
        shooterObj.setTesting();
        shooterObj.setManual(mechJoy.getY());
      }
      else{
        shooterObj.setStop();
      }
      
      if(mechJoy.getRawButton(6)){
        weightAdjObj.setWeightTest();
        weightAdjObj.manualWeight(mechJoy.getY());
      }
      else{
        weightAdjObj.setWeightStop();
      }

      intakeObj.displayMethod();
      shooterObj.displayValues();
      hangElevObj.run();
      hangPivotObj.run();
      intakeObj.run();
      shooterObj.run();
      weightAdjObj.run();
    }

    else if(mechJoy.getRawAxis(3) > 0){
      SmartDashboard.putString("MODE: ", "HYPE");
      ///////////////////////////////////////////////////////////
      //                        DRIVE                          //
      ///////////////////////////////////////////////////////////
      driveObj.arcadeDrive(baseJoy.getX(), baseJoy.getY());
      //  driveObj.tankDrive(baseJoy.getY(), baseTwoJoy.getY());
      
      ///////////////////////////////////////////////////////////
      //                        LIMELIGHT                      //
      ///////////////////////////////////////////////////////////

      if(baseJoy.getRawButton(9)){
        limelightObj.setDrivingMode();
      }
      else if(baseJoy.getRawButton(10)){
        limelightObj.setTrackingMode();
      }

      ///////////////////////////////////////////////////////////
      //                        HANG                           //
      ///////////////////////////////////////////////////////////
      /*
      if(mechJoy.getRawButton(0)){
        hangObj.setMidHang();
      }

      else if(mechJoy.getRawButton(7)){   //THERE SHOULD BE A SETHIGHHANGGRAB
        hangObj.setHighHang();
      }

      else if(mechJoy.getRawButton(8)){
        hangObj.setHighHangGrab();
      }

      else if(mechJoy.getRawButton(2)){
        hangObj.setPivotManual();
      }

      else if(mechJoy.getRawButton(3)){
        hangObj.setElevatorManual();
      }

      else{
        hangObj.setNothing();
      }
      */
      
      if(mechJoy.getRawButton(3)){
        hangPivotObj.setTesting();
        hangPivotObj.manualPivot(mechJoy.getY());
      }

      else{
        hangPivotObj.setStop();
      }

      if(mechJoy.getRawButton(7)){
        hangElevObj.setElevatorExtend();
      }

      else if(mechJoy.getRawButton(8)){
        hangElevObj.setElevatorRetract();
      }

      else{
        hangElevObj.setElevatorStop();
      }

      if(mechJoy.getRawButton(9)){
        weightAdjObj.setWeightTest();
        weightAdjObj.manualWeight(mechJoy.getY());
      }

      else{
        weightAdjObj.setWeightStop();
      }

      if (mechJoy.getRawButton(11)) {
        hangPivotObj.resetEnc();
        hangElevObj.encoderReset();
        weightAdjObj.weightReset();
      }

      ///////////////////////////////////////////////////////////
      //                        INTAKE                         //
      ///////////////////////////////////////////////////////////
      
      /*
      if(baseJoy.getRawButton(11))){
        intakeObj.setIntakeMode();
      }
      */

      if(baseJoy.getRawButton(11)){
        intakeObj.setOverrideMode();
      }

      else if(baseJoy.getPOV() == 0){
        intakeObj.setOutakeMode();
      }

      /*
      else if(baseJoy.getRawButton(6)){
        intakeObj.setFeedingMode();
      }
      */

      else{
        intakeObj.setStopMode();
      }

      ///////////////////////////////////////////////////////////
      //                         SHOOTER                       //
      ///////////////////////////////////////////////////////////
      
      if(baseJoy.getRawButton(1)){
        shooterObj.setLowHubShoot();
      }
      
      else if(baseJoy.getRawButton(3)){
        shooterObj.setUpperHubShoot();
      }

      else if(baseJoy.getRawButton(4)){
        shooterObj.setLaunchPadShoot();
      }
      
      else{
        shooterObj.setStop();
      }
      
      ///////////////////////////////////////////////////////////
      //                         RUN                           //
      ///////////////////////////////////////////////////////////

      hangObj.run();
      intakeObj.run();
      shooterObj.run();
      hangPivotObj.run();
      hangElevObj.run();
      weightAdjObj.run();
    }
  }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {

    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {

      ///////////////////////////////////////////////////////////
      //                         AUTONOMOUS                    //
      ///////////////////////////////////////////////////////////
      
      if(baseJoy.getRawButton(11)){  
        SmartDashboard.putString("AUTONOMOUS: ", "ONE BALL");
        autonObj.setOneBall();
      }

      else if(baseJoy.getRawButton(12)){
        SmartDashboard.putString("AUTONOMOUS: ", "TWO BALL");
        autonObj.setTwoBall();
      }

      else if(baseJoy.getRawButton(9)){
        SmartDashboard.putString("AUTONOMOUS: ", "THREE BALL");
        autonObj.setThreeBallHigh();
      }

      else if(baseJoy.getRawButton(10)){
        SmartDashboard.putString("AUTONOMOUS: ", "THREE BALL LOW");
        autonObj.setThreeBallHighLow();
      }
      
      else if(baseJoy.getRawButton(1)){
        SmartDashboard.putString("AUTONOMOUS: ", "NOTHING");
        autonObj.setNothing();
      }
    }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
  }
}
