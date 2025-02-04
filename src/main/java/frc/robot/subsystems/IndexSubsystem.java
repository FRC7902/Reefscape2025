package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
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
    public SparkMaxConfig indexMotorConfig = new SparkMaxConfig();
    
    private RelativeEncoder m_encoder;
    private SparkClosedLoopController m_pidController;
    

    public double indexSpeed = 0;  


    public IndexSubsystem() {
        indexMotorConfig.idleMode(IdleMode.kBrake);
        indexMotorConfig.openLoopRampRate(IndexConstants.kRampRate);
        indexMotorConfig.smartCurrentLimit(20); // 20 Amps
        m_encoder = indexMotor.getEncoder();
        m_pidController = indexMotor.getClosedLoopController();

        m_pidController.setP(Constants.IndexConstants.kP);
        m_pidController.setI(Constants.IndexConstants.kI);
        m_pidController.setD(Constants.IndexConstants.kD);
        m_pidController.setFF(Constants.IndexConstants.kFF);

        SmartDashboard.putNumber("P Gain", Constants.IndexConstants.kP);
        SmartDashboard.putNumber("I Gain", Constants.IndexConstants.kI);
        SmartDashboard.putNumber("D Gain", Constants.IndexConstants.kD);
        SmartDashboard.putNumber("Feed Forward", Constants.IndexConstants.kFF);
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
            // Update PID coefficients from SmartDashboard (if applicable)
            Constants.IndexConstants.kP = SmartDashboard.getNumber("P Gain", Constants.IndexConstants.kP);
            Constants.IndexConstants.kI = SmartDashboard.getNumber("I Gain", Constants.IndexConstants.kI);
            Constants.IndexConstants.kD = SmartDashboard.getNumber("D Gain", Constants.IndexConstants.kD);
            Constants.IndexConstants.kFF = SmartDashboard.getNumber("Feed Forward", Constants.IndexConstants.kFF);
            
            // Update the PID controller if the coefficients have changed
            SparkClosedLoopController.setP(Constants.IndexConstants.kP);
            SparkClosedLoopController.setI(Constants.IndexConstants.kI);
            SparkClosedLoopController.setD(Constants.IndexConstants.kD);
            SparkClosedLoopController.setFF(Constants.IndexConstants.kFF);
    
            // Output the current speed to the SmartDashboard
            SmartDashboard.putNumber("Index Speed", indexSpeed); 
            SmartDashboard.putNumber("Motor Velocity", m_encoder.getVelocity());
    }
}
