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
import frc.robot.Robot;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class DriveToAprilTagPosCmd extends Command
{
  Pose2d robotPose2d;
  Pose3d robotPose3d;
  PhotonPipelineResult photonRes;
  Optional<PhotonTrackedTarget> targetOpt;
  PhotonTrackedTarget target;
  Pose3d cameraPose3d;
  Transform3d camToTarget3d;
  Pose3d targetPose3d;
  Pose2d goalPose2d;
  double xSpeed;
  double ySpeed;
  double omegaSpeed;
  int aprilTagNum;

  private String m_aprilTag;
  private double m_xOffset;
  private double m_yOffset;
  private double m_xyTol;
  // private String m_alliance;
  private boolean m_atSetPoint;
  private final SwerveSubsystem m_swerveSubsystem;
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(6.0, 6.0);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(6.0, 6.0);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
  private Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(0, 0, 0),
  new Rotation3d(0.0,0.0,Math.PI));
  
  //private final PhotonCamera photonCamera;
  private final Supplier<Pose2d> m_poseProvider;
  
  private final ProfiledPIDController m_xController = new ProfiledPIDController(2.25, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController m_yController = new ProfiledPIDController(2.25, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController m_omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

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
    m_yController.setTolerance(xyTol); //meters
    
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
    var robotPose = m_poseProvider.get();
    m_omegaController.reset(robotPose.getRotation().getRadians());
    m_xController.reset(robotPose.getX());
    m_yController.reset(robotPose.getY());
    //int allianceAprilTag = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    //System.out.println(" April Tag: " + aprilTag + " April Tag Number: " + aprilTagNum + " X Offset: " + xOffset + " Y Offset: " + yOffset + " XY Tolerance: " + xyTol);
  
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    robotPose2d = m_poseProvider.get();
    robotPose3d = new Pose3d(
                             robotPose2d.getX(),
                             robotPose2d.getY(),
                             0.0,
                             new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

    photonRes = VisionSubsystem.camAprTgLow.getLatestResult();

    if (photonRes.hasTargets()) {
      //Find the tag we want to chase
      targetOpt = photonRes.getTargets().stream()
      .filter(t -> t.getFiducialId() == aprilTagNum)
      .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() != -1)
      .findFirst();

      if (targetOpt.isPresent()) {
        target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;

        // Transform the robot's pose to find the camera's pose
        cameraPose3d = robotPose3d
            .transformBy(new Transform3d(new Translation3d(0, 0, 0), new Rotation3d()));

        // Trasnform the camera's pose to the target's pose
        camToTarget3d = target.getBestCameraToTarget();
        targetPose3d = cameraPose3d.transformBy(camToTarget3d);

        // Transform the tag's pose to set our goal
        goalPose2d = targetPose3d.transformBy(TAG_TO_GOAL).toPose2d();

        // Drive
        m_xController.setGoal(goalPose2d.getX());
        m_yController.setGoal(goalPose2d.getY());
        m_omegaController.setGoal(goalPose2d.getRotation().getRadians());
      }
      else{
        m_xController.setGoal(robotPose2d.getX());
        m_yController.setGoal(robotPose2d.getY());
        m_omegaController.setGoal(robotPose2d.getRotation().getRadians());
      }

    }

    if (lastTarget != null) {
      // Drive to the target
      if (!m_xController.atGoal()) {xSpeed = m_xController.calculate(robotPose3d.getX());}
      else {xSpeed = 0;}

      if (!m_yController.atGoal()) {ySpeed = m_yController.calculate(robotPose3d.getX());}
      else {ySpeed = 0;}

      if (!m_omegaController.atGoal()) {omegaSpeed = m_omegaController.calculate(robotPose3d.getX());}
      else {omegaSpeed = 0;}

      if (!m_xController.atGoal() && !m_yController.atGoal() && !m_omegaController.atGoal()){
        m_swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, omegaSpeed, robotPose2d.getRotation()));
      }
      else {
        m_swerveSubsystem.lock();
        m_atSetPoint = true;
      }
    }     
  }

  @Override
  public boolean isFinished()
  {
    return m_atSetPoint;
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.lock();
  }

}