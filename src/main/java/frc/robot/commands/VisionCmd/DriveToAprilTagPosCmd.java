package frc.robot.commands.VisionCmd;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.subsystems.Vision.AprilTagVisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class DriveToAprilTagPosCmd extends Command
{
  int aprilTagNum;

  private String m_aprilTag;
  private double m_xOffset;
  private double m_yOffset;
  private double m_xyTol;
  private boolean m_atSetPoint;
  private final SwerveSubsystem m_swerveSubsystem;
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.MAX_SPEED * 0.75, Constants.MAX_SPEED * 0.75);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.MAX_SPEED * 0.75, Constants.MAX_SPEED * 0.75);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI*2, Math.PI);
  private Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(0, 0, 0),
  new Rotation3d(0.0,0.0,Math.PI));
  
  private final Supplier<Pose2d> m_poseProvider;
  
  private final ProfiledPIDController m_xController = new ProfiledPIDController(4.0, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController m_yController = new ProfiledPIDController(4.0, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController m_omegaController = new ProfiledPIDController(3.0, 0, 0, OMEGA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;

public DriveToAprilTagPosCmd(String aprilTag, double xOffset, double yOffset, double xyTol, SwerveSubsystem swerveSubsystem)
  {  
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.m_aprilTag = aprilTag;
    this.m_xOffset = xOffset;
    this.m_yOffset = yOffset;
    this.m_xyTol = xyTol;
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_poseProvider = swerveSubsystem::getPose;
    
    TAG_TO_GOAL = new Transform3d(new Translation3d(m_xOffset, m_yOffset, 0),
                                  new Rotation3d(0.0,0.0,Math.PI));

    m_xController.setTolerance(m_xyTol); //meters
    m_yController.setTolerance(m_xyTol); //meters
    
    addRequirements(swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    lastTarget = null;
    m_atSetPoint = false;
    Robot.aprilTagAlliance();
    /*
    * This is being used because for some reason the alliance is not being passed to
    * the command from Robot.aprilTagAlliance() as an integer, so we are using a string instead.
    */
    if(m_aprilTag == "Amp"){aprilTagNum = AprilTagConstants.ampID;}
    if(m_aprilTag == "Speaker"){aprilTagNum = AprilTagConstants.speakerID;}
    if(m_aprilTag == "StageA"){aprilTagNum = AprilTagConstants.stageIDA;}
    if(m_aprilTag == "StageB"){aprilTagNum = AprilTagConstants.stageIDB;}
    if(m_aprilTag == "StageC"){aprilTagNum = AprilTagConstants.stageIDC;}

    m_omegaController.setTolerance(Units.degreesToRadians(3.0)); //3 degrees
    m_omegaController.enableContinuousInput(-Math.PI, Math.PI);
    Pose2d robotPose = m_poseProvider.get();
    m_omegaController.reset(robotPose.getRotation().getRadians());
    m_xController.reset(robotPose.getX());
    m_yController.reset(robotPose.getY());
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    Pose2d robotPose2d = m_poseProvider.get(); // Get the robot's pose
    /**
     * Represents a 3D pose in space.
     * 
     * The Pose3d class stores the position and orientation of an object in a 3D space.
     * It consists of X, Y, and Z coordinates, as well as a rotation represented by a Rotation3d object.
     */
    Pose3d robotPose3d = new Pose3d(
                             robotPose2d.getX(),
                             robotPose2d.getY(),
                             0.0,
                             new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    PhotonPipelineResult photonRes = AprilTagVisionSubsystem.camAprTgLow.getLatestResult(); // Get the latest result from PhotonVision

    if (photonRes.hasTargets()) { // Check if the latest result has any targets
      
      Optional<PhotonTrackedTarget> targetOpt = photonRes.getTargets().stream() // Get the targets from the latest result
      .filter(t -> t.getFiducialId() == aprilTagNum) // Filter the targets to find the target with the specified ID
      //.filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() != -1) // Filter the targets to find the target that is not the last target and has a valid pose
      .findFirst(); // Get the first target that matches the criteria

      if (targetOpt.isPresent()) { // If a target is found
        PhotonTrackedTarget target = targetOpt.get(); // Get the target
        lastTarget = target; // Set the last target to the current target

        // Transform the robot's pose to find the camera's pose
        Pose3d cameraPose3d = robotPose3d
            .transformBy(new Transform3d(new Translation3d(0.051, 0.0, 0.536), new Rotation3d(0, Math.toRadians(-20), Math.toRadians(0))));

        // Trasnform the camera's pose to the target's pose
        Transform3d camToTarget3d = target.getBestCameraToTarget(); // Get the best camera to target transform
        Pose3d targetPose3d = cameraPose3d.transformBy(camToTarget3d); // Transform the camera's pose to the target's pose
        Pose2d goalPose2d = targetPose3d.transformBy(TAG_TO_GOAL).toPose2d(); // Transform the target's pose to the goal's pose

        // Drive
        m_xController.setGoal(goalPose2d.getX()); // Set the goal for the x controller
        m_yController.setGoal(goalPose2d.getY()); // Set the goal for the y controller
        m_omegaController.setGoal(goalPose2d.getRotation().getRadians()); // Set the goal for the omega controller
      }
      else{ // If a target is not found
        m_xController.setGoal(robotPose2d.getX()); // Set the goal for the x controller
        m_yController.setGoal(robotPose2d.getY()); // Set the goal for the y controller
        m_omegaController.setGoal(robotPose2d.getRotation().getRadians()); // Set the goal for the omega controller
      }
    }

    if (lastTarget != null) { // If a target is found
      // Drive to the target
      double xSpeed; // Declare a variable to store the x speed
      if (!m_xController.atGoal()) {xSpeed = m_xController.calculate(robotPose3d.getX());} // If the x controller is not at the goal, calculate the x speed
      else {xSpeed = 0;} // If the x controller is at the goal, set the x speed to 0
    

      double ySpeed; // Declare a variable to store the y speed
      if (!m_yController.atGoal()) {ySpeed = m_yController.calculate(robotPose3d.getY());} // If the y controller is not at the goal, calculate the y speed
      else {ySpeed = 0;} // If the y controller is at the goal, set the y speed to 0

      double omegaSpeed; // Declare a variable to store the omega speed
      if (!m_omegaController.atGoal()) {omegaSpeed = m_omegaController.calculate(robotPose2d.getRotation().getRadians());} // If the omega controller is not at the goal, calculate the omega speed
      else {omegaSpeed = 0;} // If the omega controller is at the goal, set the omega speed to 0

      if (!m_xController.atGoal() && !m_yController.atGoal() && !m_omegaController.atGoal()){ // If the x, y, and omega controllers are not at the goal
        m_swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 
        ySpeed, omegaSpeed, robotPose2d.getRotation())); // Drive the swerve subsystem based on the calculated speeds
      }
      else { // If the x, y, and omega controllers are at the goal
        m_swerveSubsystem.lock(); // Lock the swerve subsystem
        m_atSetPoint = true; // Set atSetPoint to true
      }
    }     
  }

  @Override
  public boolean isFinished() // Returns true when the command should end
  {
    return m_atSetPoint; // Return atSetPoint
  }

  @Override
  public void end(boolean interrupted) { // Called once the command ends or is interrupted
    m_swerveSubsystem.lock(); // Lock the swerve subsystem
  }

}