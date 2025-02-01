package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkRelativeEncoder;


public class IndexSubsystem extends SubsystemBase {
    public SparkMax indexMotor = new SparkMax(Constants.IndexConstants.kIndexMotorCAN, SparkMax.MotorType.kBrushless);
    SparkMaxConfig indexMotorConfig = new SparkMaxConfig();


    // public SparkPIDController speedPID = indexMotor.getPIDController();

    public double indexSpeed = 0;


    public IndexSubsystem() {
        indexMotorConfig.idleMode(IdleMode.kBrake);
        // speedPID.setReference(0, CANSparkBase.ControlType.kVelocity);
        // speedPID.setOutputRange(0, 1);
    }

    public void setSpeed(double speed) {
        indexSpeed = speed;
        indexMotor.set(speed);
    }

    public double getSpeed() {
        return indexSpeed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Index Speed", indexSpeed); 
    }
}
