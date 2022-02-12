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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogInput;

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
  |  GYRO IMPLEMENTED | AHRS PORT [SPI.Port.kMXP]
  |  LEFT DRIVE | 2 NEO MOTORS: * PORT NEEDED *
  |  RIGHT DRIVE | 2 NEO MOTORS: * PORT NEEDED *
  |  
  |  NOTE: MAY NOT BE IMPLEMENTED
  |  SHIFTER | DOUBLESOLENOID: * CHANNEL PORTS NEEDED *
  |
  */

  private CANSparkMax frontLeft;
  private CANSparkMax backLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backRight;
  private RelativeEncoder relEnc;
  private DoubleSolenoid shiftSol;

  private Drive driveObj;
  private Shifter shifterObj;

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
  |  GYRO IMPLEMENTED | AHRS PORT [SPI.Port.kMXP]
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
  private AHRS gyro;

  private Hang hangObj;

  ///////////////////////////////////////////////////////////
  //                        INTAKE                         //
  ///////////////////////////////////////////////////////////

  /*
  |
  |  INTAKE | BAG MOTOR | TALON SRX: * PORT NEEDED *
  |  HOLD SWITCH | DIO LIMIT SWITCH: * PORT NEEDED *
  |
  */

  private WPI_TalonSRX intakeMotor;
  private DigitalInput holdLSwitch;

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
  //                       CAMERA                          //
  ///////////////////////////////////////////////////////////

  /*
  |
  |  WIDEFIELD CAMERA: * PORT NEEDED *
  |  INTAKE CAMERA: * PORT NEEDED *
  |  HANG CAMERA: * PORT NEEDED *
  |
  */

  private Camera wideFieldCamera;
  private Camera intakeCamera;
  private Camera hangCamera;
  private AnalogInput lineTrack;

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

  private Autonomous autonObj;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    /* ! WARNING !: PORTS ARE NOT FINAL AND NEED TO BE CHECKED STILL */
    lineTrack = new AnalogInput(0);
    ///////////////////////////////////////////////////////////
    //                        DRIVE                          //
    ///////////////////////////////////////////////////////////
    /*
    frontLeft = new CANSparkMax(6, MotorType.kBrushless);
    backLeft = new CANSparkMax(15, MotorType.kBrushless);
    frontRight = new CANSparkMax(16, MotorType.kBrushless);
    backRight = new CANSparkMax(3, MotorType.kBrushless);
    //  relEnc = frontLeft.getEncoder();
    //  shiftSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    
    driveObj = new Drive(frontLeft, backLeft, frontRight, backRight);
    //  shifterObj = new Shifter(shiftSol);
    */
    ///////////////////////////////////////////////////////////
    //                         HANG                          //
    ///////////////////////////////////////////////////////////
    /*
    elevMotor = new WPI_TalonFX(0);
    pivotMotor = new WPI_TalonSRX(0);
    pivotEnc = new TalonEncoder(pivotMotor);
    upperLSwitch = new DigitalInput(0);
    bottomLSwitch = new DigitalInput(1);
    frontLSwitch = new DigitalInput(2);
    backLSwitch = new DigitalInput(3);
    gyro = new AHRS(SPI.Port.kMXP);
    
    hangObj = new Hang(elevMotor, upperLSwitch, bottomLSwitch, elevEnc, pivotMotor, pivotEnc, gyro, frontLSwitch, backLSwitch);
    */
    ///////////////////////////////////////////////////////////
    //                         INTAKE                        //
    ///////////////////////////////////////////////////////////
    
    intakeMotor = new WPI_TalonSRX(8);
    holdLSwitch = new DigitalInput(0);

    intakeObj = new Intake(intakeMotor, holdLSwitch);
    
    ///////////////////////////////////////////////////////////
    //                         SHOOTER                       //
    ///////////////////////////////////////////////////////////
    /*
    shooterMotor = new WPI_TalonFX(1);

    limelightObj = new Limelight();
    shooterObj = new Shooter(limelightObj, shooterMotor, intakeObj);
    */
    ///////////////////////////////////////////////////////////
    //                         CAMERA                        //
    ///////////////////////////////////////////////////////////
    /*
    wideFieldCamera = new Camera(0);
    intakeCamera = new Camera(1);
    hangCamera = new Camera(2);
    */
    ///////////////////////////////////////////////////////////
    //                         JOYSTICKS                     //
    ///////////////////////////////////////////////////////////

    //  baseJoy = new Joystick(0);
    //  baseTwoJoy = new Joystick(1);
    //  mechJoy = new Joystick(1);

    ///////////////////////////////////////////////////////////
    //                         AUTONOMOUS                    //
    ///////////////////////////////////////////////////////////

    //  autonObj = new Autonomous();
    
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

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //  autonObj.run();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  SmartDashboard.putNumber("VOLTAGE", lineTrack.getVoltage());
    ///////////////////////////////////////////////////////////
    //                        DRIVE                          //
    ///////////////////////////////////////////////////////////

    //  driveObj.arcadeDrive(baseJoy.getY(), baseJoy.getX());
    //  driveObj.tankDrive(baseJoy.getY(), baseTwoJoy.getY());

    ///////////////////////////////////////////////////////////
    //                        SHIFTER                        //
    ///////////////////////////////////////////////////////////
    /*
    if(baseJoy.getRawButton(1)){
      shifterObj.setPower();
    }
    else if(baseJoy.getRawButton(2)){
      shifterObj.setSpeed();
    }
    */
    ///////////////////////////////////////////////////////////
    //                        HANG                           //
    ///////////////////////////////////////////////////////////
    /*
    if(mechJoy.getRawButton(0)){
      hangObj.setPivotInward();
    }

    else if(mechJoy.getRawButton(1)){
      hangObj.setPivotOutward();
    }

    else if(mechJoy.getRawButton(2)){
      hangObj.setElevatorExtend();
    }

    else if(mechJoy.getRawButton(3)){
      hangObj.setElevatorRetract();
    }

    else{
      hangObj.setElevatorStop();
    }
    */
    ///////////////////////////////////////////////////////////
    //                        INTAKE                         //
    ///////////////////////////////////////////////////////////
    /*
    if(baseJoy.getRawButton(4)){
      intakeObj.setTestingMode();
      intakeObj.intake(-0.4);
    }
    
    else if(baseJoy.getRawButton(5)){
      intakeObj.setOutakeMode();
    }

    else if(mechJoy.getRawButton(6)){
      intakeObj.setFeedingMode();
    }
  
    else{
      intakeObj.setStopMode();
    }
    */
    ///////////////////////////////////////////////////////////
    //                         SHOOTER                       //
    ///////////////////////////////////////////////////////////
    /*
    if(baseJoy.getRawButton(7)){
      shooterObj.setTesting();
      shooterObj.setSpeedManual(0.4);
    }

    else if(mechJoy.getRawButton(8)){
      shooterObj.setAutoShoot();
    }
    

    else if(mechJoy.getRawButton(9)){
      shooterObj.setLaunchPadShoot();
    }
    */
    /*
    else{
      shooterObj.setStop();
    }
    */
    ///////////////////////////////////////////////////////////
    //                         RUN                           //
    ///////////////////////////////////////////////////////////

    //  hangObj.run();
    /*
    intakeObj.displayMethod();
    shooterObj.displayValues();
    */
    intakeObj.displayMethod();
    intakeObj.run();
    //shooterObj.run();
    
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

    /* ! WARNING !: AUTONOMOUS ROUTINES HAVE YET TO BE MADE */
    /*
    if(baseJoy.getRawButton(1)){  
      SmartDashboard.putString("AUTONOMOUS: ", "ROUTINE 1");
    }

    else if(baseJoy.getRawButton(7)){
      SmartDashboard.putString("AUTONOMOUS: ", "ROUTINE 2");
    }

    else if(baseJoy.getRawButton(8)){
      SmartDashboard.putString("AUTONOMOUS: ", "ROUTINE 3");
    }

    else if(baseJoy.getRawButton(9)){
      SmartDashboard.putString("AUTONOMOUS: ", "ROUTINE 4");
    }

    else if(baseJoy.getRawButton(10)){
      SmartDashboard.putString("AUTONOMOUS: ", "ROUTINE 5");
    }

    else if(baseJoy.getRawButton(11)){
      SmartDashboard.putString("AUTONOMOUS: ", "ROUTINE 6");
    }

    else if(baseJoy.getRawButton(12)){
      SmartDashboard.putString("AUTONOMOUS: ", "ROUTINE 7");
    }

    else{
      SmartDashboard.putString("AUTONOMOUS: ", "ROUTINE 8");
    }
    */
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
