// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.VisionCmd.DriveToNoteCmd;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Vision.ObjectVisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

//import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  final CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  ObjectVisionSubsystem visionSubsystem;
  Vision vision;
  //private final LEDsSubSystem ledsSubSystem;
  
  //private final DriveToAprilTagPosCmd driveToAprilTagPosCmd = new DriveToAprilTagPosCmd(drivebase);
   /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Register Named Commands
    //NamedCommands.registerCommand(null, null);
    //NamedCommands.registerCommand("Shoot", new ScoreAutoCmd(launcherSubsystem));
    //drivebase.setupPathPlanner();
    //NamedCommands.registerCommand("DriveToSpeaker", new DriveToSpeakerCmd(drivebase));

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity 
    // buttons are quick rotation positions to different ways to face
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
    // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
    //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
    //                                                                                              OperatorConstants.LEFT_Y_DEADBAND),
    //                                                                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
    //                                                                                              OperatorConstants.LEFT_X_DEADBAND),
    //                                                                                              () -> yawToSpeaker(),
    //                                                                 driverXbox.povUp(),
    //                                                                 driverXbox.povDown(),
    //                                                                 driverXbox.povRight(),
    //                                                                 driverXbox.povLeft());
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                    OperatorConstants.LEFT_Y_DEADBAND),
                                                                      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                    OperatorConstants.LEFT_X_DEADBAND),
                                                                                                    () -> driverXbox.getRightX() * 0.5,
                                                                      driverXbox.povUp(),
                                                                      driverXbox.povDown(),
                                                                      driverXbox.povRight(),
                                                                      driverXbox.povLeft(),
                                                                      driverXbox.a());
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //     () -> driverXbox.getRightX(),
    //     () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX() * 0.5);

    // Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
    //     () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) *
    //                                   DrivebaseConstants.Max_Speed_Multiplier,
    //     () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) *
    //                                   DrivebaseConstants.Max_Speed_Multiplier,
    //     () -> yawToSpeaker());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX());

    drivebase.setDefaultCommand(
        // !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
        !RobotBase.isSimulation() ? closedAbsoluteDriveAdv : closedAbsoluteDriveAdv);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //Button 1 is "A" on xbox controller
    //Button 2 is "B" on xbox controller
    //Button 3 is "X" on xbox controller  
    //Button 4 is "Y" on xbox controller
    //Button 5 is "Left Bumper" on xbox controller
    //Button 6 is "Right Bumper" on xbox controller
    //Button 7 is "Back" on xbox controller
    //Button 8 is "Start" on xbox controller
    //Button 9 is "Left Joystick" on xbox controller
    //Button 10 is "Right Joystick" on xbox controller
    //Axis 0 is left joystick x side to side
    //Axis 1 is left joystick y forward and back
    //Axis 2 is left trigger 
    //Axis 3 is right trigger
    //Axis 4 is right joystick x side to side
    //Axis 5 is right joystick y forward and back[\]
    

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //driverXbox.b().whileTrue(new DriveDistance(.25, 0, 6, drivebase));
    //new JoystickButton(driverXbox, 4).whileTrue(new DriveDistance(drivebase));
    // driverXbox.povUp().onTrue(new DriveDistancePPID(1, 0, 0, .1, drivebase));
    // driverXbox.povDown().onTrue(new DriveDistancePPID(-1, 0, 0, .1, drivebase));
    // driverXbox.povRight().onTrue(new DriveDistancePPID(0, -1, 0, .1, drivebase));
    // driverXbox.povLeft().onTrue(new DriveDistancePPID(0, 1, 0, .1, drivebase));

    //driverXbox.y().onTrue((Commands.run(drivebase::sysIdDriveMotorCommand)));
    driverXbox.y().whileTrue(Commands.deferredProxy(() -> drivebase.sysIdAngleMotorCommand()));
    

    driverXbox.x().whileTrue(Commands.deferredProxy(() -> drivebase.sysIdDriveMotorCommand()));

    // driverXbox.y().onTrue((Commands.run(drivebase::SwerveDriveTest)));
    // driverXbox.b().onTrue((Commands.run(drivebase::sysIdAngleMotorCommand)));
    //driverXbox.x().whileTrue(new DriveToAprilTagPosCmdOld(drivebase));
    
    // driverXbox.x().whileTrue(new DriveToAprilTagPosCmd("Speaker",
    //                                                    1.7,
    //                                                    0.0,
    //                                                    0.1,
    //                                                    vision,
    //                                                    drivebase)); 
    
    // driverXbox.x().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
    //                          Vision.getAprilTagPose(AprilTagConstants.speakerID, 1.7, 0.0, 0.0))));

    // driverXbox.b().whileTrue(new DriveToNoteCmd(drivebase).andThen
    //                         (new DriveDistancePPID(-.5, 0, 0, .1, drivebase)));

    // driverXbox.b().whileTrue(new DriveToNoteCmd(drivebase).andThen
    //                         (drivebase.driveToPose(drivebase.getOffsetPose(0.5, 0.0, 0.0))));

    // driverXbox.a().whileTrue(Commands.deferredProxy(() ->
    //                         (drivebase.driveToPose(drivebase.getOffsetPose(1.0, 1.0, 30.0)))));

    // driverXbox.y().whileTrue(Commands.deferredProxy(() ->
    //                         (drivebase.driveToPose(drivebase.getOffsetPose(-1.0, -1.0, -30.0)))));

    // driverXbox.a().whileTrue(Commands.deferredProxy(() ->
    //                         (drivebase.aimAtSpeaker(2.0))));

    // driverXbox.y().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
    //               new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(180.0)))));

    //driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected(); //Returns the autonomous command selected from the autoChooser.
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  /**
   * Sets the speed multiplier for the drivebase based on the state of the right and left bumpers on the driver's Xbox controller.
   * If both bumpers are pressed, the speed multiplier is set to 1 (HighSpd).
   * If either bumper is pressed, the speed multiplier is set to 0.75 (MedSpd).
   * If neither bumper is pressed, the speed multiplier is set to 0.50 (LowSpd).
   */
  public void spencerButtons(){

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("HighSpd");
      DrivebaseConstants.Max_Speed_Multiplier = 1;
    }

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == false ||
        driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("MedSpd");
      DrivebaseConstants.Max_Speed_Multiplier = .75;
    }

    if (driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == false){
      //System.out.println("LowSpd");
      DrivebaseConstants.Max_Speed_Multiplier = .50;
    }
    
  }
  
  /**
   * Calculates the yaw value to the speaker so it can be automtically yawed to via the right stick button.
   * 
   * @return The calculated yaw value.
   */
  // public double yawToSpeaker(){
  //   double yawToSpeakerValue = 0.0;
  //   PIDController zController = null;
  //   int fiducialId = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4; //speakerID
  //   try {
  //     zController = new PIDController(.04, 0.0, 0.0);//0.07,0.0, 0.000;
  //     zController.setTolerance(.5);

  //     if (driverXbox.getHID().getRightStickButton() == false){  
  //       yawToSpeakerValue = MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND);
  //     } else{
          
  //         if (vision.getLatestResult(Cameras.APR_TG_LOW_CAM).hasTargets() == true){
  //         if (vision.getLatestResult(Cameras.APR_TG_LOW_CAM).getBestTarget().getFiducialId() == fiducialId){
  //           yawToSpeakerValue = MathUtil.clamp(-zController.calculate(vision.getLatestResult(Cameras.APR_TG_LOW_CAM).getBestTarget().getYaw(),0), -1.0 , 1.0);
  //         }
  //       }
  //     }
  //   } finally {
  //     if (zController != null) {
  //       zController.close();
  //     }
  //   }
  //   return yawToSpeakerValue;
  // }

}
