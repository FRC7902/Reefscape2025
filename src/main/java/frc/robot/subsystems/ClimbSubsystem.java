package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {

    //object creation of motors
    private final SparkMax  m_climbLeaderMotor = new SparkMax(ClimbConstants.kClimbLeaderMotorCANID, MotorType.kBrushless);
    private final SparkMax m_climbFollowerMotor = new SparkMax(ClimbConstants.kClimbFollowerMotorCANID, MotorType.kBrushless);

    private final DCMotor motorSim = DCMotor.getNeo550(2);

    private final SparkMaxSim s_climbLeaderMotor = new SparkMaxSim(m_climbLeaderMotor, motorSim);

    
    //object creation of motor configuration (used to configure tbe motors)
    private final SparkMaxConfig m_climbLeaderMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_climbFollowerMotorConfig = new SparkMaxConfig();

    //object creation of absolute encoder. The REV Through bore encoder is used for climb
    private final AnalogEncoder m_absoluteEncoder = new AnalogEncoder(0, 0, 0);
    private final AnalogEncoderSim s_absoluteEncoder = new AnalogEncoderSim(m_absoluteEncoder);


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

    //object creation of bangbang controller
    //bang-bang controllers are used in high-inertia system. Considering the climb will be carrying the whole robot, it falls under "high inertia".
    //bang-bang controllers have faster recovery times than pid controllers, resulting in the control effort in the front direction to be as large as possible
    //we want this as the climb should move as fast as possible (as you will generally be climbing in the last few seconds of the match)
    //BE SURE TO SET THE MOTORS TO COAST INSTEAD OF BRAKE FOR IDLE ACTION!!! BRAKE CAN CAUSE DESTRUCTIVE OSCILLATION and a very upset argoon
    //ariana grande would love this controller
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

        //inverts the encoder on true (if motor is spinning counter-clockwise)
        m_absoluteEncoder.setInverted(false);
        //similar to controller deadband, reduces sensitivity of inputs
        //in this case, it is when the absolute encoder reaches the end of its range, which can result in instability.
        m_absoluteEncoder.setVoltagePercentageRange(0, 0);
        
        //sets all configuration to the motors.
        //any previous parameters are reset here to be overwritten with new parameters.
        //these parameters will persist. This is incredibly important as without this, all parameters are wiped on reboot.        
        m_climbLeaderMotor.configure(m_climbLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_climbFollowerMotor.configure(m_climbLeaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void reachGoal (double goal) {
        m_bangBang.setSetpoint(goal);
        double bangBangOutput = m_bangBang.calculate(m_absoluteEncoder.get());
        m_climbLeaderMotor.setVoltage(bangBangOutput);
    }

    //Returns the reading of the encoder.
    public double getEncoderDistance() {
        return m_absoluteEncoder.get();
    } 

    public double getSimEncoderDistance() {
        return s_absoluteEncoder.get();
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
        SmartDashboard.putNumber("Encoder reading", getEncoderDistance());
        SmartDashboard.putBoolean("Climb stopped", m_climbLeaderMotor.get() == 0);

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

        //Checks to see if any of the motors have any faults, and if so, reports it to DriverStation
        if (m_climbLeaderMotor.hasActiveFault()) {
            DriverStation.reportWarning("MOTOR WARNING: SparkMax ID " + ClimbConstants.kClimbLeaderMotorCANID + " is currently reporting an error with: \"" + reportMotorError(m_climbLeaderMotor) + "\"", true);
        }
        if (m_climbFollowerMotor.hasActiveFault()) {
            DriverStation.reportWarning("MOTOR WARNING: SparkMax ID " + ClimbConstants.kClimbFollowerMotorCANID + " is currently reporting an error with: \"" + reportMotorError(m_climbFollowerMotor) + "\"", true);
        }

    }

    @Override
    public void simulationPeriodic() {
        //Displays live motor and limit switch metrics on SmartDashboard 
        SmartDashboard.putNumber("Encoder reading", getEncoderDistance());
        SmartDashboard.putBoolean("Climb stopped", s_climbLeaderMotor.getAppliedOutput() == 0);

        SmartDashboard.putNumber("Motor Voltage", s_climbLeaderMotor.getBusVoltage());
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
