package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class IndexSubsystem extends SubsystemBase {
    public SparkMax indexMotor = new SparkMax(Constants.IndexConstants.kIndexMotorCAN, SparkMax.MotorType.kBrushless);
    public SparkMaxConfig indexMotorConfig = new SparkMaxConfig();
    
    public double indexSpeed = 0;


    public IndexSubsystem() {
        indexMotorConfig.idleMode(IdleMode.kBrake);
        indexMotorConfig.openLoopRampRate(0.5);
        indexMotorConfig.smartCurrentLimit(20); // 20 Amps
    }

    public void setSpeed(double speed) {
        indexSpeed = speed;
        indexMotor.set(speed);
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
        SmartDashboard.putNumber("Index Speed", indexSpeed); 
    }
}
