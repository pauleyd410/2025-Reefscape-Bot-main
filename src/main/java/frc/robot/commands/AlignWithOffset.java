package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithOffset extends Command{
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer noTagTimer, stopTimer;
  private SwerveSubsystem driveBase;


  public AlignWithOffset(boolean isRightScore, SwerveSubsystem driveBase) {
    xController = new PIDController(2.5, 0, 0);
    yController = new PIDController(4.5, 0, 0);
    rotController = new PIDController(0.05, 0, 0);
    this.isRightScore = isRightScore;
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.noTagTimer = new Timer();
    this.noTagTimer.start();

    rotController.setSetpoint(0);
    rotController.setTolerance(0.5);

    xController.setSetpoint(-0.5);
    xController.setTolerance(0.005);

    yController.setSetpoint(isRightScore ? -2.0 : 2.0);
    yController.setTolerance(0.005);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("")) {
      this.noTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("X", positions[2]);
      double xSpeed = xController.calculate(positions[2]);
      SmartDashboard.putNumber("X Speed", xSpeed);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      driveBase.drive(new Translation2d(yController.getError() < 0.005 ? xSpeed : 0, ySpeed), rotValue, false);

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
    return this.noTagTimer.hasElapsed(1) ||
        stopTimer.hasElapsed(0.3);
  }
}
