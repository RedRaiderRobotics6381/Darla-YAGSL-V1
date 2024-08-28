// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AprilTagConstants;
// import frc.robot.subsystems.Vision.AprilTagVisionSubsystem;
import frc.robot.subsystems.Vision.ObjectVisionSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
// import frc.robot.subsystems.Vision.AprilTagVisionSubsystem;
// import frc.robot.subsystems.swervedrive.Vision;
// import frc.robot.subsystems.Vision.AprilTagVisionSubsystem.Cameras;
import frc.robot.subsystems.LEDsSubSystem;
import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  private static Robot   instance;
  private Command m_autonomousCommand;
  ObjectVisionSubsystem m_ObjectVision;
  Vision m_Vision;
  // Field2d field2d;
  Supplier<Pose2d> poseProvider;


  // public static PhotonCamera camObj = new PhotonCamera("camObj"); // Create a new PhotonCamera object
  // public static PhotonCamera camAprTgLow = new PhotonCamera("camAprTgLow"); // Create a new PhotonCamera object
  public static LEDsSubSystem m_LEDsSubSystem = new LEDsSubSystem(); // Create a new LEDsSubSystem object
  // public static LEDsSubsystem m_LEDsSubSystem = new LEDsSubsystem(); // Create a new LEDsSubSystem object
  //private LEDsSubSystem m_LEDsSubSystem;
  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  SwerveDrive swerveDrive;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_ObjectVision = new ObjectVisionSubsystem(m_LEDsSubSystem);
    //m_Vision = new Vision(poseProvider, field2d);
    


    
    DriverStation.silenceJoystickConnectionWarning(true); // Silence the joystick connection warning

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    //LEDsSubSystem.rainbow();


    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // AprilTagVisionSubsystem.camAprTgLowCamLatancyAlert.set(AprilTagVisionSubsystem.getLatestResult(Cameras.CAM_APR_TG_LOW).getLatencyMillis() > 100);
    // AprilTagVisionSubsystem.updateVisionField();

    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
    // LEDsSubSystem.fadeEffect(60, 255);
    //m_LEDsSubSystem.strobeEffectVar(75, 255, 255, 0.25);
    // LEDsSubSystem.fireEffect();
    m_LEDsSubSystem.scanEffect(60, 255, 255);
    // LEDsSubSystem.waveEffect(60, 50);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    aprilTagAlliance();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
    aprilTagAlliance();
    
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    m_robotContainer.spencerButtons();
    m_ObjectVision.watchForNote(); 
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
    // AprilTagVisionSubsystem.getVisionSim().update(swerveDrive.getPose());
  // m_Vision.getVisionSim().update(swerveDrive.getPose());
  }
  
  /**
   * Sets the AprilTag constants based on the alliance color.
   */
  public static void aprilTagAlliance(){
    AprilTagConstants.ampID     = DriverStation.getAlliance().get() == Alliance.Blue ? 6 : 5;
    AprilTagConstants.speakerID = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    AprilTagConstants.stageIDA  = DriverStation.getAlliance().get() == Alliance.Blue ? 14 : 13;
    AprilTagConstants.stageIDB  = DriverStation.getAlliance().get() == Alliance.Blue ? 15 : 12;
    AprilTagConstants.stageIDC  = DriverStation.getAlliance().get() == Alliance.Blue ? 16 : 11;
  }
  
}
