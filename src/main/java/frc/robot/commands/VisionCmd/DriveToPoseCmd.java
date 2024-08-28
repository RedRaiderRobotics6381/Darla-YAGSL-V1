package frc.robot.commands.VisionCmd;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToPoseCmd extends Command
{

  private final SwerveSubsystem m_swerveSubsystem;
  private String m_aprilTag;
  int aprilTagNum;
  double newX;
  double newY;


  
  private double m_xOffset;
  
  final Supplier<Pose2d> m_poseProvider;
  
  Pose2d aprilTagPose2d;
  Pose2d aprilTagTargetPose2d;
  

  
public DriveToPoseCmd(String aprilTag, Double xOffset, SwerveSubsystem swerveSubsystem)
  {  
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.m_aprilTag = aprilTag;
    this.m_xOffset = xOffset;
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_poseProvider = swerveSubsystem::getVisionPose;


    
    addRequirements(swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
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
    Optional<Pose3d> aprilTagPose3d = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo).getTagPose(aprilTagNum);
    aprilTagPose2d = aprilTagPose3d.get().toPose2d();

    // Calculate new x and y positions based on xOffset and rotation of aprilTagPose2d
    newX = aprilTagPose2d.getTranslation().getX() + m_xOffset * Math.cos(aprilTagPose2d.getRotation().getRadians());
    newY = aprilTagPose2d.getTranslation().getY() + m_xOffset * Math.sin(aprilTagPose2d.getRotation().getRadians());
    // aprilTagTargetPose2d = new Pose2d(newX, newY, aprilTagPose2d.getRotation());
    //System.out.println("AprilTag Pose: " + newX + ", " + newY + ", " + aprilTagPose2d.getRotation().getDegrees());
    //System.out.println("AprilTag Target Pose: " + aprilTagTargetPose2d.getTranslation().getX() + ", " + aprilTagTargetPose2d.getTranslation().getY() + ", " + aprilTagTargetPose2d.getRotation().getDegrees());

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    // m_swerveSubsystem.driveToPose(aprilTagTargetPose2d);

    m_swerveSubsystem.driveToPose(new Pose2d(new Translation2d(newX, newY), aprilTagPose2d.getRotation()));
    
  }

  // @Override
  // public boolean isFinished() // Returns true when the command should end
  // {
  //   return; // Return atSetPoint
  // }

  // @Override
  // public void end(boolean interrupted) { // Called once the command ends or is interrupted
  //   m_swerveSubsystem.lock(); // Lock the swerve subsystem
  // }

}