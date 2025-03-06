// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  private final SwerveDrive swerveDrive;
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");

  public SwerveSubsystem() {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.MAX_SPEED,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
  }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
   }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  /**
	 * The primary method for controlling the drivebase. Takes a
	 * {@link Translation2d} and a rotation rate, and
	 * calculates and commands module states accordingly. Can use either open-loop
	 * or closed-loop velocity control for
	 * the wheel velocities. Also has field- and robot-relative modes, which affect
	 * how the translation vector is used.
	 *
	 * @param translation   {@link Translation2d} that is the commanded linear
	 *                      velocity of the robot, in meters per
	 *                      second. In robot-relative mode, positive x is torwards
	 *                      the bow (front) and positive y is
	 *                      torwards port (left). In field-relative mode, positive x
	 *                      is away from the alliance wall
	 *                      (field North) and positive y is torwards the left wall
	 *                      when looking through the driver station
	 *                      glass (field West).
	 * @param rotation      Robot angular rate, in radians per second. CCW positive.
	 *                      Unaffected by field/robot
	 *                      relativity.
	 * @param fieldRelative Drive mode. True for field-relative, false for
	 *                      robot-relative.
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
		swerveDrive.drive(translation,
				rotation,
				fieldRelative,
				false); // Open loop is disabled since it shouldn't be used most of the time.
	}

  /**
	 * Drive according to the chassis robot oriented velocity.
	 *
	 * @param velocity Robot oriented {@link ChassisSpeeds}
	 */
	public void drive(ChassisSpeeds velocity) {
		swerveDrive.drive(velocity);
	}


}
