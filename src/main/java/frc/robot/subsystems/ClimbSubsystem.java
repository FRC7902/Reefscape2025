package frc.robot.subsystems;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax  m_climbLeaderMotor = new SparkMax(ClimbConstants.kClimbLeaderMotorCANID, MotorType.kBrushless);
    private final SparkMax m_climbFollowerMotor = new SparkMax(ClimbConstants.kClimbLeaderMotorCANID, MotorType.kBrushless);
    private final DigitalInput m_LimitSwitch = new DigitalInput(ClimbConstants.kLimitSwitchPin);
    private final SparkMaxConfig m_climbLeaderMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_climbFollowerMotorConfig = new SparkMaxConfig();



    public ClimbSubsystem() {
        m_climbLeaderMotor.clearFaults();
        m_climbFollowerMotor.clearFaults();

        m_climbFollowerMotorConfig.follow(m_climbLeaderMotor);

        m_climbLeaderMotorConfig.inverted(false);
        m_climbFollowerMotorConfig.inverted(false);

        m_climbLeaderMotorConfig.idleMode(IdleMode.kBrake);
        m_climbFollowerMotorConfig.idleMode(IdleMode.kBrake);       

        m_climbLeaderMotorConfig.smartCurrentLimit(ClimbConstants.kMotorStallCurrent, ClimbConstants.kMotorFreeSpeedCurrent, ClimbConstants.kMotorRPMLimit);
        m_climbFollowerMotorConfig.smartCurrentLimit(ClimbConstants.kMotorStallCurrent, ClimbConstants.kMotorFreeSpeedCurrent, ClimbConstants.kMotorRPMLimit);

        m_climbLeaderMotor.configure(m_climbLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_climbFollowerMotor.configure(m_climbLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
