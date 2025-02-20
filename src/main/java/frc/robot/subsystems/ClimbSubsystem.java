package frc.robot.subsystems;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Robot;

public class ClimbSubsystem extends SubsystemBase {

    //object creation of motors
    private final SparkMax m_climbLeaderMotor = new SparkMax(ClimbConstants.kClimbLeaderMotorCANID, MotorType.kBrushless);
    private final SparkMax m_climbFollowerMotor = new SparkMax(ClimbConstants.kClimbFollowerMotorCANID, MotorType.kBrushless);

    //object creation of simulation placeholder motor
    private final DCMotor motorSim = DCMotor.getNeo550(2);

    //object creation of simulation SPARK MAX
    private final SparkMaxSim s_climbLeaderMotor = new SparkMaxSim(m_climbLeaderMotor, motorSim);

    //object creation of motor configuration (used to configure tbe motors)
    private final SparkMaxConfig m_climbLeaderMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_climbFollowerMotorConfig = new SparkMaxConfig();

    // Object creation of absolute encoder. The REV Through bore encoder is used for climb
    // The DutyCycleEncoder is used as the REV Through bore encoder is an absolute encoder that uses PWM via one of the DI (Digital IO) pins on the RIO
    private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(ClimbConstants.kRevThroughBoreIO);
    private final DutyCycleEncoderSim s_absoluteEncoder = new DutyCycleEncoderSim(m_absoluteEncoder);

     
    private final ElevatorSim m_climbSim =
    new ElevatorSim(
        motorSim,
        5,
        70,
        6,
        0,
        100,
        true,
        0.0, 
        0.01,
        0);
    

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
        m_climbLeaderMotorConfig.idleMode(IdleMode.kBrake);
        m_climbFollowerMotorConfig.idleMode(IdleMode.kBrake);       

        //sets current limits to motors to ensure they do not exceed a set current limit
        //prevents damage towards motors from pulling too much current -- VERY IMPORTANT!
        m_climbLeaderMotorConfig.smartCurrentLimit(ClimbConstants.kMotorStallCurrent, ClimbConstants.kMotorFreeSpeedCurrent);
        m_climbFollowerMotorConfig.smartCurrentLimit(ClimbConstants.kMotorStallCurrent, ClimbConstants.kMotorFreeSpeedCurrent);

        //inverts the encoder on true (if motor is spinning counter-clockwise)
        m_absoluteEncoder.setInverted(false);
        //sets the duty cycle of the encoder to ensure accuracy (much match with encoder specifications)
        m_absoluteEncoder.setDutyCycleRange(ClimbConstants.kRevThroughBoreMinPulse, ClimbConstants.kRevThroughBoreMaxPulse);
        //sets the expected frequency the encoder will output to the RIO (must match with encoder specifications)
        //recommended to set this as it improves the reliability of the encoder's output
        m_absoluteEncoder.setAssumedFrequency(ClimbConstants.kRevThroughBoreFrequency);
        
        //sets all configuration to the motors.
        //any previous parameters are reset here to be overwritten with new parameters.
        //these parameters will persist. This is incredibly important as without this, all parameters are wiped on reboot.        
        m_climbLeaderMotor.configure(m_climbLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_climbFollowerMotor.configure(m_climbFollowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //powers motors with set voltage
    public void driveMotors (double voltage) {
        m_climbLeaderMotor.setVoltage(voltage);
    }

    //Returns the reading of the encoder.
    public double getClimbArmAngle() {
        return m_absoluteEncoder.get() * 360;
    } 

    public double getSimEncoderDistance() {
        return s_absoluteEncoder.get() * 360;
    }

    //Stops outputting to motors.
    public void stopMotors() {
        m_climbLeaderMotor.stopMotor();
    }
    
    //This function returns the type of error the motor is experiencing should it have an error.
    public String reportMotorError(SparkMax motor) {
        Faults motorFault = motor.getFaults();
        if (motorFault.can) {
            return "CAN";
        }
        else if (motorFault.escEeprom) {
            return "ESC_EEPROM";
        }
        else if (motorFault.firmware) {
            return "FIRMWARE";
        }
        else if (motorFault.gateDriver) {
            return "GATE_DRIVER";
        }
        else if (motorFault.motorType) {
            return "MOTOR_TYPE";
        }
        else if (motorFault.sensor) {
            return "SENSOR";
        } 
        else if (motorFault.temperature) {
            return "TEMPERATURE";
        }
        return null;        
    }

    //function that is constantly run in code every 20 ms.
    @Override
    public void periodic() {
        //Displays live motor and limit switch metrics on SmartDashboard
        if (!(Robot.isSimulation())) {
            SmartDashboard.putNumber("Climb Arm Angle", getClimbArmAngle());
            SmartDashboard.putBoolean("Climb Arm Stopped", m_climbLeaderMotor.get() == 0);

            SmartDashboard.putNumber("Climb Motor (" + ClimbConstants.kClimbLeaderMotorCANID + ") Speed", m_climbLeaderMotor.get());
            SmartDashboard.putNumber("Climb Motor (" + ClimbConstants.kClimbLeaderMotorCANID + ") Bus Voltage", m_climbLeaderMotor.getBusVoltage());
            SmartDashboard.putNumber("Climb Motor (" + ClimbConstants.kClimbLeaderMotorCANID + ") Current", m_climbLeaderMotor.getOutputCurrent());
            SmartDashboard.putNumber("Climb Motor (" + ClimbConstants.kClimbLeaderMotorCANID + ") Temperature", m_climbLeaderMotor.getMotorTemperature());

            SmartDashboard.putNumber("Climb Motor (" + ClimbConstants.kClimbFollowerMotorCANID + ") Speed", m_climbLeaderMotor.get());
            SmartDashboard.putNumber("Climb Motor (" + ClimbConstants.kClimbFollowerMotorCANID + ") Bus Voltage", m_climbLeaderMotor.getBusVoltage());
            SmartDashboard.putNumber("Climb Motor (" + ClimbConstants.kClimbFollowerMotorCANID + ") Current", m_climbLeaderMotor.getOutputCurrent());
            SmartDashboard.putNumber("Climb Motor (" + ClimbConstants.kClimbFollowerMotorCANID + ") Temperature", m_climbLeaderMotor.getMotorTemperature());

            if (m_climbLeaderMotor.hasActiveFault()) {
            DriverStation.reportWarning("MOTOR WARNING: Climb SPARK MAX ID " + ClimbConstants.kClimbLeaderMotorCANID + " is currently reporting an error with: \"" + reportMotorError(m_climbLeaderMotor) + "\"", true);
            }
            if (m_climbFollowerMotor.hasActiveFault()) {
            DriverStation.reportWarning("MOTOR WARNING: Climb SPARK MAX ID " + ClimbConstants.kClimbFollowerMotorCANID + " is currently reporting an error with: \"" + reportMotorError(m_climbFollowerMotor) + "\"", true);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        //Displays live motor and limit switch metrics on SmartDashboard
        if (Robot.isSimulation()) { 
            //SmartDashboard.putNumber("Encoder reading", getEncoderDistance());
            SmartDashboard.putBoolean("Climb stopped", s_climbLeaderMotor.getAppliedOutput() == 0);
            SmartDashboard.putNumber("Motor Bus Voltage", s_climbLeaderMotor.getBusVoltage());
            SmartDashboard.putNumber("Motor Current", s_climbLeaderMotor.getMotorCurrent());
            SmartDashboard.putNumber("Climb Setpoint", s_climbLeaderMotor.getSetpoint());
            SmartDashboard.putNumber("Applied Output", s_climbLeaderMotor.getAppliedOutput());
            SmartDashboard.putNumber("Climb Velocity", s_climbLeaderMotor.getVelocity());
            SmartDashboard.putNumber("Climb Position", s_climbLeaderMotor.getPosition());


            //Checks to see if motors are going upwards and if the encoder has reached the set limit

            s_climbLeaderMotor.iterate(m_climbLeaderMotor.getAppliedOutput(), 12, 0.02);
            m_climbSim.setInput(s_climbLeaderMotor.getVelocity() * RobotController.getBatteryVoltage());
            m_climbSim.update(0.020);
            s_absoluteEncoder.set(m_climbSim.getPositionMeters());
            RoboRioSim.setVInVoltage(
                        BatterySim.calculateDefaultBatteryLoadedVoltage(m_climbSim.getCurrentDrawAmps()));
            }
    }
}
