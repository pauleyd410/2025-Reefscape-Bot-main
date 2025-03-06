package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    private SparkMax algaeMotor;
    private SparkMaxConfig motorConfig;

    private AlgaeSubsystem(){
        algaeMotor = new SparkMax(Constants.ALGAE_MOTOR_ID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();

        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.smartCurrentLimit(Constants.SmartAlgaeLimit);
    }

    public void setPower(double power) {
        algaeMotor.set(power);
    }
    
    @Override
    public void periodic(){}
}
