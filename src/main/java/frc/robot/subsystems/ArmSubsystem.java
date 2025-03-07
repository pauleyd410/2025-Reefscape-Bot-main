package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotMath.Arm;
import edu.wpi.first.math.controller.ArmFeedforward;


public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_motor = new SparkMax(12, MotorType.kBrushless);
    private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final AbsoluteEncoder m_absEncoder = m_motor.getAbsoluteEncoder();

    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_angle = Rotations.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity = RPM.mutable(0);

    //Standard classes for controlling the arm
    private final ProfiledPIDController m_pidController;
    private final ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.kArmkS,
                                                                    ArmConstants.kArmkG,
                                                                    ArmConstants.kArmkV,
                                                                    ArmConstants.kArmkA); 

    /**
     * Subsystem Constructor
     */

    public ArmSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(ArmConstants.kArmCurrentLimit)
            .closedLoopRampRate(ArmConstants.kArmRampRate)
            .idleMode(IdleMode.kBrake)
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ArmConstants.kArmKp, ArmConstants.kArmKi, ArmConstants.kArmKd)
            .outputRange(-1, 1)
            .maxMotion
            .maxVelocity(ArmConstants.kArmMaxVelocityRPM)
            .maxAcceleration(ArmConstants.kArmMaxAccelerationRPMperSecond)
            .allowedClosedLoopError(ArmConstants.kArmAllowedClosedLoopError.in(Rotations));
            m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            synchronizeAbsoluteEncoder();
            m_pidController = new ProfiledPIDController(ArmConstants.kArmKp,
                                                        ArmConstants.kArmKi,
                                                        ArmConstants.kArmKd,
                                                        new Constraints(ArmConstants.kArmMaxVelocityRPM,
                                                        ArmConstants.kArmMaxAccelerationRPMperSecond));
            m_pidController.setTolerance(0.01);

    }

    /**
   * Near the maximum Angle of the arm within X degrees.
   *
   * @param toleranceDegrees Degrees close to maximum of the Arm.
   * @return is near the maximum of the arm.
   */
    public boolean nearMax(double toleranceDegrees)
    {
    if (getAngle().isNear(ArmConstants.kMaxAngle, Degrees.of(toleranceDegrees)))
    {
      System.out.println("Current angle: " + getAngle().in(Degrees));
      System.out.println("At max:" + getAngle().isNear(ArmConstants.kMaxAngle, Degrees.of(toleranceDegrees)));
    }
    return getAngle().isNear(ArmConstants.kMaxAngle, Degrees.of(toleranceDegrees));
  }

  /**
   * Near the minimum angle of the Arm in within X degrees.
   *
   * @param toleranceDegrees Tolerance of the Arm.
   * @return is near the minimum of the arm.
   */
   public boolean nearMin(double toleranceDegrees)
    {
    if (getAngle().isNear(ArmConstants.kMinAngle, Degrees.of(toleranceDegrees)))
    {
      System.out.println("Current angle: " + getAngle().in(Degrees));
      System.out.println("At min:" + getAngle().isNear(ArmConstants.kMinAngle, Degrees.of(toleranceDegrees)));
    }
    return getAngle().isNear(ArmConstants.kMinAngle, Degrees.of(toleranceDegrees));

    }

    /**
   * Run the control loop to reach and maintain the setpoint from the preferences.
   */
  public void reachSetpoint(double setPointDegree)
  {
    double  goalPosition = Arm.convertAngleToSensorUnits(Degrees.of(setPointDegree)).in(Rotations);
    boolean rioPID       = true;
    if (rioPID)
    {
      double pidOutput = m_pidController.calculate(m_encoder.getPosition(), goalPosition);
      State setpointState = m_pidController.getSetpoint();
      m_motor.setVoltage(pidOutput +
                         m_feedforward.calculate(setpointState.position,
                                                 setpointState.velocity)
                        );
    } else
    {
      m_controller.setReference(goalPosition,
                                ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
  }
     /**
   * Get the Angle of the Arm.
   *
   * @return Angle of the Arm.
   */
     public Angle getAngle()
   {
    m_angle.mut_replace(Arm.convertSensorUnitsToAngle(m_angle.mut_replace(m_encoder.getPosition(), Rotations)));
    return m_angle;
    }

    /**
   * Get the velocity of Arm.
   *
   * @return Velocity of the Arm.
   */
  public AngularVelocity getVelocity()
  {
    return m_velocity.mut_replace(Arm.convertSensorUnitsToAngle(Rotations.of(m_encoder.getVelocity())).per(Minute));
  }

  public Command setGoal(double degree) 
  {
    return run(() -> reachSetpoint(degree));
  }

  public void stop()
  {
    m_motor.set(0.0);
  }

    /**
   * Synchronizes the NEO encoder with the attached Absolute Encoder.
   */
    public void synchronizeAbsoluteEncoder()
     {
    m_encoder.setPosition(Rotations.of(m_absEncoder.getPosition()).minus(ArmConstants.kArmOffsetToHorizantalZero)
                                   .in(Rotations));
    }

    @Override
    public void periodic()
    {
    System.out.println(getAngle());
    }  
}
