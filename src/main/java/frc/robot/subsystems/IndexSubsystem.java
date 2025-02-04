package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexConstants;


public class IndexSubsystem extends SubsystemBase {
    public SparkMax indexMotor = new SparkMax(Constants.IndexConstants.kIndexMotorCAN, SparkMax.MotorType.kBrushless);
    public SparkMaxConfig indexMotorConfig = new SparkMaxConfig();
    
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    IndexConstants.kS, 
    IndexConstants.kV
    );

    public double indexSpeed = 0;  


    public IndexSubsystem() {
        indexMotorConfig.idleMode(IdleMode.kBrake);
        indexMotorConfig.openLoopRampRate(IndexConstants.kRampRate);
        indexMotorConfig.smartCurrentLimit(20); // 20 Amps
    }

    public void setSpeed(double speed) {
        indexSpeed = speed;
        double feedforwardVoltage = feedforward.calculate(speed);
        indexMotor.setVoltage(feedforwardVoltage); 
        //indexMotor.set(speed);
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
