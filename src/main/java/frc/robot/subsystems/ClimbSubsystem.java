package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    //object creation of motors
    private final SparkMax  m_climbLeaderMotor = new SparkMax(ClimbConstants.kClimbLeaderMotorCANID, MotorType.kBrushless);
    private final SparkMax m_climbFollowerMotor = new SparkMax(ClimbConstants.kClimbLeaderMotorCANID, MotorType.kBrushless);

    //object creation of motor configuration
    private final SparkMaxConfig m_climbLeaderMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_climbFollowerMotorConfig = new SparkMaxConfig();

    private final Encoder m_absoluteEncoder = new Encoder(1, 0, false);

    private final BangBangController m_bangBang = new BangBangController();

    public ClimbSubsystem() {
    
        //clears any previous faults on motors
        //do this so that any errors from previous usage are cleared
        //if any errors persist, then we know there's an issue with the motors
        m_climbLeaderMotor.clearFaults();
        m_climbFollowerMotor.clearFaults();

        //sets it so that the secondary motor (follower) follows any commands sent to the primary motor (leader).
        //recommended as you will now only need to send commands to the primary motor instead of having to do both primary and secondary.
        m_climbFollowerMotorConfig.follow(m_climbLeaderMotor);

        //inverts motor run position (clockwise or counterclockwise).
        //change booleans when necessary.
        m_climbLeaderMotorConfig.inverted(false);
        m_climbFollowerMotorConfig.inverted(false);

        //ensures motors stay at their current position when no power is being applied to them.
        //we don't want the motors moving downwards when we want to it to stay up!
        m_climbLeaderMotorConfig.idleMode(IdleMode.kCoast);
        m_climbFollowerMotorConfig.idleMode(IdleMode.kCoast);       

        //sets current limits to motors to ensure they do not exceed a set current limit
        //prevents damage towards motors from pulling too much current -- VERY IMPORTANT!
        m_climbLeaderMotorConfig.smartCurrentLimit(ClimbConstants.kMotorStallCurrent, ClimbConstants.kMotorFreeSpeedCurrent, ClimbConstants.kMotorRPMLimit);
        m_climbFollowerMotorConfig.smartCurrentLimit(ClimbConstants.kMotorStallCurrent, ClimbConstants.kMotorFreeSpeedCurrent, ClimbConstants.kMotorRPMLimit);

        m_absoluteEncoder.reset();
        m_absoluteEncoder.setDistancePerPulse(0);
        m_absoluteEncoder.setMinRate(0);
        m_absoluteEncoder.setSamplesToAverage(0);
        
        //sets all configuration to the motors.
        //any previous parameters are reset here to be overwritten with new parameters.
        //these parameters will persist. This is incredibly important as without this, all parameters are wiped on reboot.        
        m_climbLeaderMotor.configure(m_climbLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_climbFollowerMotor.configure(m_climbLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //Returns the reading of the encoder.
    public double getEncoderDistance() {
        return m_absoluteEncoder.getDistance();
    }

    //Stops outputting to motors.
    public void stopMotors() {
        m_climbLeaderMotor.stopMotor();
    }

    //Sets powers to motors.
    public void runMotors() {
        m_climbLeaderMotor.set(m_bangBang.calculate(m_absoluteEncoder.getRate(), ClimbConstants.kClimbRaisedPosition));
    }
    

    //function that is constantly run in code every 20 ms.
    @Override
    public void periodic() {
        //Displays live motor and limit switch metrics on SmartDashboard 
        SmartDashboard.putNumber("Encoder reading", getEncoderDistance());
        SmartDashboard.putBoolean("Climb stopped", m_absoluteEncoder.getStopped());

        SmartDashboard.putNumber("Leader Motor Voltage", m_climbLeaderMotor.getBusVoltage());
        SmartDashboard.putNumber("Leader Motor Current", m_climbLeaderMotor.getOutputCurrent());
        SmartDashboard.putNumber("Leader Motor Temperature", m_climbLeaderMotor.getMotorTemperature());

        SmartDashboard.putNumber("Follower Motor Voltage", m_climbFollowerMotor.getBusVoltage());
        SmartDashboard.putNumber("Follower Motor Current", m_climbFollowerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Follower Motor Temperature", m_climbFollowerMotor.getMotorTemperature());

        //Checks to see if motors are going upwards and if the encoder has reached the set limit
        if (m_climbLeaderMotor.get() > 0 && getEncoderDistance() >= ClimbConstants.kClimbRaisedPosition) {
            stopMotors();
        }

    }
}
