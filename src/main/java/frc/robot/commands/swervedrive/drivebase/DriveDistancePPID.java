package frc.robot.commands.swervedrive.drivebase;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveDistancePPID extends Command
{
  private final SwerveSubsystem swerveSubsystem;
  private Pose2d endPose;
  private double xDist;
  private double yDist;
  private double rotation;
  private double xyTol;
  private boolean drvDstCmp;
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(4.0, 4.0);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(4.0, 4.0);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
  private final ProfiledPIDController xController = new ProfiledPIDController(2.5, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(2.5, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);
  private final Supplier<Pose2d> poseProvider;

  public DriveDistancePPID(double xDist, double yDist, double rotation, double xyTol, SwerveSubsystem swerveSubsystem){  
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.xDist = xDist;
    this.yDist = yDist;
    this.rotation = rotation;
    this.xyTol = xyTol;
    this.swerveSubsystem = swerveSubsystem;
    this.poseProvider = swerveSubsystem::getPose;
    addRequirements(swerveSubsystem);  
  }

  /*
   * Initializes the DriveDistancePPID command.
   * Sets the tolerance for x, y, and omega controllers.
   * Resets the controllers and calculates the end pose based on the distance and rotation.
   */
  @Override
  public void initialize()
  {
    xController.setTolerance(xyTol); //meters
    yController.setTolerance(xyTol); //meters
    omegaController.setTolerance(Units.degreesToRadians(3.0)); //3 degrees
    omegaController.enableContinuousInput(-Math.PI, Math.PI); // -180 to 180 degrees
    //var startPose = poseProvider.get().; // Get the current pose
    //startpose = swerveSubsystem.getPose();
    //omegaController.reset(startPose.getRotation().getRadians());
    omegaController.reset(swerveSubsystem.getPose().getRotation().getRadians()); // Reset the omega controller
    xController.reset(swerveSubsystem.getPose().getTranslation().getX()); // Reset the x controller
    yController.reset(swerveSubsystem.getPose().getTranslation().getY()); // Reset the y controller
    // xController.reset(startPose.getX()); // Reset the x controller
    // yController.reset(startPose.getY()); // Reset the y controller
    // Calculate the new pose
    // double newX = startPose.getTranslation().getX() + xDist; // Calculate the new x position
    // double newY = startPose.getTranslation().getY() + yDist;
    // double newX = startPose.getTranslation().getX() + distance * Math.cos(startPose.getRotation().getRadians());
    // double newY = startPose.getTranslation().getY() + distance * Math.sin(startPose.getRotation().getRadians());
    // double newX = startPose.getTranslation().getX() + (xDist * Math.cos(startPose.getRotation().getRadians())); // Calculate the new x position
    // double newY = startPose.getTranslation().getY() + (yDist * Math.sin(startPose.getRotation().getRadians())); // Calculate the new y position
    // double newRot = startPose.getRotation().getRadians() + Math.toRadians(rotation); // Calculate the new y position
    // endPose = new Pose2d(newX, newY, new Rotation2d(newRot));
    // double newX = startPose.getTranslation().getX() + 
    //               (xDist * Math.cos(startPose.getRotation().getRadians()) +
    //               (yDist * Math.cos(startPose.getRotation().getRadians()) +
    //               (xDist * Math.sin(startPose.getRotation().getRadians()) + 
    //               (yDist * Math.sin(startPose.getRotation().getRadians()))))); // Calculate the new x position
    // double newY = startPose.getTranslation().getY() + 
    //               (xDist * Math.cos(startPose.getRotation().getRadians()) +
    //               (yDist * Math.cos(startPose.getRotation().getRadians()) +
    //               (xDist * Math.sin(startPose.getRotation().getRadians()) + 
    //               (yDist * Math.sin(startPose.getRotation().getRadians()))))); // Calculate the new y position
    double newX = swerveSubsystem.getPose().getTranslation().getX() + 
                  (xDist * Math.cos(swerveSubsystem.getPose().getRotation().getRadians()) + 
                  (yDist * Math.sin(swerveSubsystem.getPose().getRotation().getRadians()))); // Calculate the new x position
    double newY = swerveSubsystem.getPose().getTranslation().getY() + 
                  (xDist * Math.sin(swerveSubsystem.getPose().getRotation().getRadians()) +
                  (yDist * Math.cos(swerveSubsystem.getPose().getRotation().getRadians()))); // Calculate the new y position
    endPose = new Pose2d(newX, newY, new Rotation2d(swerveSubsystem.getPose().getRotation().getRadians() + Math.toRadians(rotation))); // Create the new pose
    drvDstCmp = false; // Set the drive distance complete flag to false
    //System.out.println("Distance: x = " + xDist + ", y = " + yDist + ", newX =" + newX + ", newY = " + newY + ", Rotation = " + rotation);
    //System.out.println("Pose Rotation: " + startPose.getRotation().getDegrees() + " Pose X: " + startPose.getX()  + " Pose y: " + startPose.getY() + ", newX =" + newX + ", newY = " + newY + ", Rotation = " + rotation );
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   *
   * Executes the drive distance PID command.
   * Calculates the desired speeds for the x, y, and omega axes based on the current robot pose and the goal end pose.
   * Updates the x, y, and omega controllers with the goal values.
   * Calculates the speeds for each axis using the controllers.
   * If the controllers have reached their goals, sets the speeds to zero.
   * If all speeds are zero, sets the drvDstCmp flag to true.
   * Otherwise, drives the swerve subsystem with the calculated speeds.
   */
  @Override
  public void execute()
  {
    var robotPose2d = poseProvider.get();
    var robotPose = new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    xController.setGoal(endPose.getX());
    yController.setGoal(endPose.getY());
    omegaController.setGoal(endPose.getRotation().getRadians());
    
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
    if (omegaController.atGoal()) {
      omegaSpeed = 0;
    }

    if (xSpeed ==0 && ySpeed == 0 && omegaSpeed == 0) {
      drvDstCmp = true;
    }
    else {
      swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
    }
  }

  @Override
  public boolean isFinished()
  {
    return drvDstCmp;
  }

  @Override
  public void end(boolean interrupted) {
    //swerveSubsystem.lock();
  }

}