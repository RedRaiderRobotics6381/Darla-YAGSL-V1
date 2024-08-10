// package frc.robot.commands.Vision;

// // import org.photonvision.PhotonCamera;
// // import org.photonvision.PhotonUtils;
// // import org.photonvision.PhotonVersion;
// // import org.photonvision.proto.Photon;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// public class PickUpNoteCmd extends Command
// {
//   private final SwerveSubsystem swerveSubsystem;
//   private final PIDController   xController;
//   //private final PIDController   yController;
//   private final PIDController   zController;
//   private boolean hasTargets;
//   boolean droveToNote;
  
//   int peakVelocity;
//   int currentVelocity;

//   boolean lowerIntakeHasNote;
//   boolean hasNote;
//   boolean outtakeHasNote;
//   boolean intakeHasNote;

//   public PickUpNoteCmd(SwerveSubsystem swerveSubsystem)
//   {
//     this.swerveSubsystem = swerveSubsystem;
    
//     xController = new PIDController(0.055, 0.00, 0.0);
//     //yController = new PIDController(0.0625, 0.00375, 0.0001);
//     zController = new PIDController(0.015,0.0, 0.000);
//     xController.setTolerance(3);
//     //yController.setTolerance(3);
//     zController.setTolerance(.5);

//     // each subsystem used by the command must be passed into the
//     // addRequirements() method (which takes a vararg of Subsystem)
//     addRequirements(swerveSubsystem);
//   }

//   /**
//    * The initial subroutine of a command.  Called once when the command is initially scheduled.
//    */
//   @Override
//   public void initialize()
//   {
//     droveToNote = false;
//     hasNote = false;
//   }

//     /**
//    * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
//    * until {@link #isFinished()}) returns true.)
//    */
//   @Override
//   public void execute()
//   {
//     var result = Robot.camObj.getLatestResult();  // Get the latest result from PhotonVision
//     hasTargets = result.hasTargets(); // Check if the latest result has any targets.
//     PhotonTrackedTarget target = result.getBestTarget();
//     if (hasTargets == true) { // && RobotContainer.driverXbox.getRawButton(2) == true
//       double TZ = target.getYaw();
//       double TX = target.getPitch();

//       double translationValx = MathUtil.clamp(xController.calculate(TX, -19), -8.0 , 8.0); //Tune the setpoint to be where the note is just barely found.
//       double translationValz = MathUtil.clamp(zController.calculate(TZ, 0.0), -6.0 , 6.0); //* throttle, 2.5 * throttle);

//       if (xController.atSetpoint() != true) {
//           swerveSubsystem.drive(new Translation2d(translationValx, 0.0), translationValz, false);
//         } else {
//               //swerveSubsystem.drive(new Translation2d(0.5, 0.0), 0.0, false);
//               //new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
//               swerveSubsystem.driveToPose(
//                             new Pose2d(new Translation2d(
//                               swerveSubsystem.getPose().getX() + 0.25 * Math.cos(swerveSubsystem.getPose().getRotation().getRadians()*12),
//                               swerveSubsystem.getPose().getY() + 0.25 * Math.sin(swerveSubsystem.getPose().getRotation().getRadians()*12)),
//                               Rotation2d.fromDegrees(swerveSubsystem.getPose().getRotation().getDegrees())));
//             }             // swerveSubsystem.drive(new Translation2d(0.5, 0.0), 0.0, false);
//     }    
//   }


//   /**
//    * <p>
//    * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
//    * the scheduler will call its {@link #end(boolean)} method.
//    * </p><p>
//    * Returning false will result in the command never ending automatically. It may still be cancelled manually or
//    * interrupted by another command. Hard coding this command to always return true will result in the command executing
//    * once and finishing immediately. It is recommended to use *
//    * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
//    * </p>
//    *
//    * @return whether this command has finished.
//    */
//   @Override
//   public boolean isFinished()
//   {
//     return hasNote;
//     //return droveToNote;
//   }

//   /**
//    * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
//    * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
//    * up loose ends, like shutting off a motor that was being used in the command.
//    *
//    * @param interrupted whether the command was interrupted/canceled
//    */
//   @Override
//   public void end(boolean interrupted)
//   {
//     swerveSubsystem.lock();
//     //ledsSubSystem.fireEffect();
//   }
// }


