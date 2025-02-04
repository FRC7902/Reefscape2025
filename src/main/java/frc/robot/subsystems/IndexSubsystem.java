.package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexConstants;


public class IndexSubsystem extends SubsystemBase {
    public SparkMax indexMotor = new SparkMax(Constants.IndexConstants.kIndexMotorCAN, SparkMax.MotorType.kBrushless);
    public SparkMaxConfig indexMotorConfig;
    public SparkClosedLoopController m_pidController;
    private RelativeEncoder m_encoder;
    
    public double indexSpeed = 0;  

    public IndexSubsystem() {
        indexMotorConfig = new SparkMaxConfig();
        m_pidController = indexMotor.getClosedLoopController();

        indexMotorConfig.smartCurrentLimit(20); // 20 Amps
        indexMotorConfig.openLoopRampRate(IndexConstants.kRampRate);
        indexMotorConfig.idleMode(IdleMode.kBrake);
        
        indexMotorConfig.closedLoop.pid(Constants.IndexConstants.indexkP, Constants.IndexConstants.indexkI, Constants.IndexConstants.indexkD);
        indexMotorConfig.closedLoop.velocityFF(Constants.IndexConstants.indexkFF);
        indexMotorConfig.closedLoop.outputRange(0, 1);
        m_pidController.setReference(0, SparkBase.ControlType.kVelocity);

    }

    public void setSpeed(double speed) {
        indexSpeed = speed;
        m_pidController.setReference(speed, SparkMax.ControlType.kVelocity);
    }

    public double getSpeed() {
        return indexSpeed;
    }

    public void stop() {
        indexMotor.stopMotor();
    }

    public void coast() {
        indexMotorConfig.idleMode(IdleMode.kCoast);
    }
    
      public void brake() {
        indexMotorConfig.idleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        // Output the current speed to the SmartDashboard
        SmartDashboard.putNumber("Index Speed", indexSpeed); 
        SmartDashboard.putNumber("Motor Velocity", m_encoder.getVelocity());
    }
}
