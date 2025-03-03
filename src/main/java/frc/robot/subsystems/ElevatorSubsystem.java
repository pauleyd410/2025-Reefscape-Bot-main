package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMath.Elevator;

public class ElevatorSubsystem extends SubsystemBase {

  // Add the code for the ElevatorSubsystem here
  private final SparkMax m_motor = new SparkMax(10, MotorType.kBrushless);
  private final SparkMax f_motor = new SparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final DigitalInput m_limitSwitchLow = new DigitalInput(0);
  
  private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd, new Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));


  // Standard classes for controlling our elevator
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);

   /**
   * The velocity of the elevator in meters per second.
   *
   * @return velocity in meters per second
   */
  public double getVelocityMetersPerSecond()
  {
    return ((m_encoder.getVelocity() / 60)/ ElevatorConstants.kElevatorGearing) *
           (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
  }

  /**
   * Subsystem constructor
   * 
   */
  public ElevatorSubsystem() {
      SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi, ElevatorConstants.kElevatorKd)
        .outputRange(-1, 1)
        .maxMotion
        .maxVelocity(ElevatorConstants.kMaxVelocity)
        .maxAcceleration(ElevatorConstants.kMaxAcceleration);
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
      double voltsOut = MathUtil.clamp(
      m_controller.calculate(getHeightMeters(), goal) +
      m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
                                            m_controller.getSetpoint().velocity), -7, 7);
      m_motor.setVoltage(voltsOut);
  }
  
  /**
   * Get Elevator Velocity
   *
   * @return Elevator Velocity
   */
  public LinearVelocity getVelocity()
  {
    return Elevator.convertRotationsToDistance(Rotations.of(m_encoder.getVelocity())).per(Minute);
  }

  /**
   * Get the height of the Elevator
   *
   * @return Height of the elevator
   */
  public Distance getHeight()
  {
    return Elevator.convertRotationsToDistance(Rotations.of(m_encoder.getPosition()));
  }

   /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeightMeters()
  {
    return Elevator.convertRotationsToDistance(Rotations.of(m_encoder.getPosition())).in(Meters);
  }

 /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getHeightMeters(),
                                             tolerance));
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }

  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_motor.set(0.0);
  }

  /**
   * Update telemetry, including the mechanism visualization.
   */
  public void updateTelemetry()
  {
    updateTelemetry();
  }

  @Override
  public void periodic()
  {
    // Put smart dashboard values here
    SmartDashboard.putNumber("Height in meters", getHeightMeters());
  }
}

