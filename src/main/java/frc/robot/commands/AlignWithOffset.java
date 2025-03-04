package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;

public class AlignWithOffset extends CommandBase {
  private final double offsetInches;
  private final double inchesToDegrees = 1.0; // Conversion factor, adjust as needed

  public AlignWithOffset(double offsetInches) {
    this.offsetInches = offsetInches;
  }

  @Override
  public void initialize() {
    // Initialization code
  }

  @Override
  public void execute() {
    double currentOffset = LimelightHelpers.getTargetOffsetX();
    double desiredOffset = currentOffset + (offsetInches * inchesToDegrees);
    // Add code to align the robot based on desiredOffset
  }

  @Override
  public void end(boolean interrupted) {
    // Cleanup code
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}