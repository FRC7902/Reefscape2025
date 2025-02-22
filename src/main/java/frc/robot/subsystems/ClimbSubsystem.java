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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Robot;

public class ClimbSubsystem extends SubsystemBase {

    // object creation of motors
    private final SparkMax m_leaderMotor =
            new SparkMax(ClimbConstants.kClimbLeaderMotorCANID, MotorType.kBrushless);
    private final SparkMax m_followerMotor =
            new SparkMax(ClimbConstants.kClimbFollowerMotorCANID, MotorType.kBrushless);

    private final DCMotor m_simMotor = DCMotor.getNeo550(2);

    private final SparkMaxSim s_simLeaderMotor = new SparkMaxSim(m_leaderMotor, m_simMotor);

    // object creation of motor configuration (used to configure tbe motors)
    private final SparkMaxConfig m_leaderMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_followerMotorConfig = new SparkMaxConfig();

    // object creation of absolute encoder. The REV Through bore encoder is used for
    // climb
    private final DutyCycleEncoder m_absoluteEncoder =
            new DutyCycleEncoder(ClimbConstants.kRevThroughBoreIO, 0, 0);

    private final Servo m_leftServo = new Servo(ClimbConstants.kLeftServoID);
    // private final Servo m_rightServo = new Servo(ClimbConstants.kRightServoID);

    /*
     * private final ElevatorSim m_climbSim = new ElevatorSim( motorSim, 5, 70, 6, 0, 100, true,
     * 0.0, 0.01, 0);
     */

    // object creation of bangbang controller
    // bang-bang controllers are used in high-inertia system. Considering the climb
    // will be carrying the whole robot, it falls under "high inertia".
    // bang-bang controllers have faster recovery times than pid controllers,
    // resulting in the control effort in the front direction to be as large as
    // possible
    // we want this as the climb should move as fast as possible (as you will
    // generally be climbing in the last few seconds of the match)
    // BE SURE TO SET THE MOTORS TO COAST INSTEAD OF BRAKE FOR IDLE ACTION!!! BRAKE
    // CAN CAUSE DESTRUCTIVE OSCILLATION and a very upset argoon
    // ariana grande would love this controller
    // private final BangBangController m_bangBang = new BangBangController();

    public ClimbSubsystem() {

        // clears any previous faults on motors
        // do this so that any errors from previous usage are cleared
        // if any errors persist, then we know there's an issue with the motors
        m_leaderMotor.clearFaults();
        m_followerMotor.clearFaults();

        // sets it so that the secondary motor (follower) follows any commands sent to
        // the primary motor (leader).
        // recommended as you will now only need to send commands to the primary motor
        // instead of having to do both primary and secondary.
        m_followerMotorConfig.follow(m_leaderMotor);

        // inverts motor run position (clockwise or counterclockwise).
        // change booleans when necessary.
        m_leaderMotorConfig.inverted(false);
        m_followerMotorConfig.inverted(false);

        // ensures motors stay at their current position when no power is being applied
        // to them.
        // we don't want the motors moving downwards when we want to it to stay up!
        m_leaderMotorConfig.idleMode(IdleMode.kBrake);
        m_followerMotorConfig.idleMode(IdleMode.kBrake);

        // sets current limits to motors to ensure they do not exceed a set current
        // limit
        // prevents damage towards motors from pulling too much current -- VERY
        // IMPORTANT!
        m_leaderMotorConfig.smartCurrentLimit(ClimbConstants.kMotorStallCurrent,
                ClimbConstants.kMotorFreeSpeedCurrent);
        m_followerMotorConfig.smartCurrentLimit(ClimbConstants.kMotorStallCurrent,
                ClimbConstants.kMotorFreeSpeedCurrent);

        // inverts the encoder on true (if motor is spinning counter-clockwise)
        m_absoluteEncoder.setInverted(false);
        // similar to controller deadband, reduces sensitivity of inputs
        // in this case, it is when the absolute encoder reaches the end of its range,
        // which can result in instability.

        // sets all configuration to the motors.
        // any previous parameters are reset here to be overwritten with new parameters.
        // these parameters will persist. This is incredibly important as without this,
        // all parameters are wiped on reboot.
        m_leaderMotor.configure(m_leaderMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        m_followerMotor.configure(m_followerMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setVoltage(double voltage) {
        m_leaderMotor.setVoltage(voltage);
    }

    // Returns the reading of the encoder.
    public double getEncoderDistance() {
        return m_absoluteEncoder.get();
    }

    // public double getSimEncoderDistance() {
    // return s_absoluteEncoder.get();
    // }

    // Stops outputting to motors.
    public void stopMotors() {
        m_leaderMotor.stopMotor();
    }

    public void setLeftServo(double degrees) {
        m_leftServo.setAngle(degrees);
    }

    // public void setRightServo(double degrees) {
    //     m_rightServo.setAngle(degrees);
    // }

    // This function returns the type of error the motor is experiencing should it
    // have an error.
    public String reportMotorError(SparkMax motor) {
        Faults motorFault = motor.getFaults();
        if (motorFault.can) {
            return "CAN";
        } else if (motorFault.escEeprom) {
            return "ESC_EEPROM";
        } else if (motorFault.firmware) {
            return "FIRMWARE";
        } else if (motorFault.gateDriver) {
            return "GATE_DRIVER";
        } else if (motorFault.motorType) {
            return "MOTOR_TYPE";
        } else if (motorFault.sensor) {
            return "SENSOR";
        } else if (motorFault.temperature) {
            return "TEMPERATURE";
        }
        return null;
    }

    // function that is constantly run in code every 20 ms.
    @Override
    public void periodic() {
        // Displays live motor and limit switch metrics on SmartDashboard
        if (!(Robot.isSimulation())) {
            SmartDashboard.putNumber("Encoder reading", getEncoderDistance());
            SmartDashboard.putBoolean("Climb stopped", m_leaderMotor.get() == 0);

            SmartDashboard.putNumber("Leader Motor Speed", m_leaderMotor.get());
            SmartDashboard.putNumber("Leader Motor Voltage", m_leaderMotor.getBusVoltage());
            SmartDashboard.putNumber("Leader Motor Current", m_leaderMotor.getOutputCurrent());
            SmartDashboard.putNumber("Leader Motor Temperature",
                    m_leaderMotor.getMotorTemperature());

            SmartDashboard.putNumber("Follower Motor Speed", m_followerMotor.get());
            SmartDashboard.putNumber("Follower Motor Voltage", m_followerMotor.getBusVoltage());
            SmartDashboard.putNumber("Follower Motor Current", m_followerMotor.getOutputCurrent());
            SmartDashboard.putNumber("Follower Motor Temperature",
                    m_followerMotor.getMotorTemperature());

            if (m_leaderMotor.hasActiveFault()) {
                DriverStation.reportWarning(
                        "MOTOR WARNING: SparkMax ID " + ClimbConstants.kClimbLeaderMotorCANID
                                + " is currently reporting an error with: \""
                                + reportMotorError(m_leaderMotor) + "\"",
                        true);
            }
            if (m_followerMotor.hasActiveFault()) {
                DriverStation.reportWarning(
                        "MOTOR WARNING: SparkMax ID " + ClimbConstants.kClimbFollowerMotorCANID
                                + " is currently reporting an error with: \""
                                + reportMotorError(m_followerMotor) + "\"",
                        true);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        // Displays live motor and limit switch metrics on SmartDashboard
        if (Robot.isSimulation()) {
            // SmartDashboard.putNumber("Encoder reading", getEncoderDistance());
            SmartDashboard.putBoolean("Climb stopped", s_simLeaderMotor.getAppliedOutput() == 0);
            SmartDashboard.putNumber("Motor Bus Voltage", s_simLeaderMotor.getBusVoltage());
            SmartDashboard.putNumber("Motor Current", s_simLeaderMotor.getMotorCurrent());
            SmartDashboard.putNumber("Climb Setpoint", s_simLeaderMotor.getSetpoint());
            SmartDashboard.putNumber("Applied Output", s_simLeaderMotor.getAppliedOutput());
            SmartDashboard.putNumber("Climb Velocity", s_simLeaderMotor.getVelocity());
            SmartDashboard.putNumber("Climb Position", s_simLeaderMotor.getPosition());

            // Checks to see if motors are going upwards and if the encoder has reached the
            // set limit

            s_simLeaderMotor.iterate(m_leaderMotor.getAppliedOutput(), 12, 0.02);
            // m_climbSim.setInput(s_climbLeaderMotor.getVelocity() *
            // RobotController.getBatteryVoltage());
            // m_climbSim.update(0.020);
            // s_absoluteEncoder.set(m_climbSim.getPositionMeters());
            // RoboRioSim.setVInVoltage(
            // BatterySim.calculateDefaultBatteryLoadedVoltage(m_climbSim.getCurrentDrawAmps()));
        }
    }
}
