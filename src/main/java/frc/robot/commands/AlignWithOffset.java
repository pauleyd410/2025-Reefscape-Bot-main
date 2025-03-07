package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithOffset extends Command{
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private double tagID = -1;
  private SwerveSubsystem driveBase;


  public AlignWithOffset(boolean isRightScore, SwerveSubsystem driveBase) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0, 0);
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0, 0);
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);
    this.isRightScore = isRightScore;
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT );
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("");

      double xSpeed = xController.calculate(positions[2]);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      driveBase.drive(new Translation2d(yController.getError() < Constants.Y_TOLERANCE_REEF_ALIGNMENT ? xSpeed : 0, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
        stopTimer.reset();
      }
    }
    else {
      driveBase.drive(new Translation2d(), 0, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
     // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}
