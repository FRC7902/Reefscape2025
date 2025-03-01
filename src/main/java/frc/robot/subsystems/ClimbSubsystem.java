package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Robot;

public class ClimbSubsystem extends SubsystemBase {

    //object creation of motors
    private final SparkMax m_climbLeaderMotor = new SparkMax(ClimbConstants.kClimbLeaderMotorCANID, MotorType.kBrushless);
    private final SparkMax m_climbFollowerMotor = new SparkMax(ClimbConstants.kClimbFollowerMotorCANID, MotorType.kBrushless);

    private final RelativeEncoder m_climbLeaderEncoder = m_climbLeaderMotor.getEncoder();

    private final Servo m_leftServo = new Servo(ClimbConstants.kLeftServoID);
    private final Servo m_rightServo = new Servo(ClimbConstants.kRightServoID);

    //object creation of simulation placeholder motor
    private final DCMotor motorSim = DCMotor.getNeo550(2);

    //object creation of simulation SPARK MAX
    private final SparkMaxSim s_climbLeaderMotor = new SparkMaxSim(m_climbLeaderMotor, motorSim);

    //object creation of motor configuration (used to configure tbe motors)
    private final SparkMaxConfig m_climbLeaderMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_climbFollowerMotorConfig = new SparkMaxConfig();

    // Object creation of absolute encoder. The REV Through bore encoder is used for climb
    // The DutyCycleEncoder is used as the REV Through bore encoder is an absolute encoder that uses PWM via one of the DI (Digital IO) pins on the RIO
    private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(ClimbConstants.kRevThroughBoreIO); //need to find exact angle where climb is 90 degrees to the horizontal
    private final DutyCycleEncoderSim s_absoluteEncoder = new DutyCycleEncoderSim(m_absoluteEncoder);

    private boolean isFunnelUnlocked;

    //variable used during initialization of robot to ensure initialization functions stop running when they are no longer needed
    private int setupClimb = 0; 

    //to be updated
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
        
        lockFunnel();
        isFunnelUnlocked = false;
    }

    //powers motors with set voltage
    public void driveMotors (double voltage) {
        m_climbLeaderMotor.setVoltage(voltage);
    }

    //Returns the reading of the encoder in degrees.
    public double getClimbArmAngle() {
        return m_absoluteEncoder.get() * 360;
    }
     
    public double getClimbMotorPosition() {
        return m_climbLeaderEncoder.getPosition();
        
    }

    //315.1 degrees - true 90
    //110.1 degrees - angle of attack
    //205.1 degrees - angle limit when climbing up
    public double getSimClimbArmAngle() {
        return (s_absoluteEncoder.get() * 360);
    }

    //Stops outputting to motors.
    public void stopMotors() {
        m_climbLeaderMotor.stopMotor();
    }
    
    //drives arm to specified angle
    //direction determines which way the arm should go
    // -1 -> counter clockwise
    //  1 - > clockwise
    // do not put any other values in direction or you risk damaging the motors

    public boolean isAtTargetAngle(double currentAngle, double targetAngle, double direction) {
        return (currentAngle * direction >= targetAngle * direction);
    }

    public void runToAngle(CommandXboxController m_operatorController, double currentAngle, double targetAngle, double direction) {
        if (currentAngle * direction < targetAngle * direction) {
            driveMotors(12 * direction);
        }
        else if (currentAngle * direction >= targetAngle * direction) {
            stopMotors();
            m_operatorController.setRumble(RumbleType.kBothRumble, 1);
            setupClimb = 2;
        }
    }

    public void runToAngle(double currentAngle, double targetAngle, double direction) {
        if (currentAngle * direction < targetAngle * direction) {
            driveMotors(12 * direction);
        }
        else if (currentAngle * direction >= targetAngle * direction) {
            stopMotors();
            setupClimb = 2;
        }
    }

    //positions the climb to the attack position after being homed
    //this is only called once during initialization of the robot and is run after the climb is homed
    public void setClimbToAttack() {
        runToAngle(getClimbArmAngle(), ClimbConstants.kClimbForwardLimit, 1);
    }

    //drives the climb to its default position, which is 90 degrees from the horizontal
    public void homeClimb() {
        if (setupClimb == 0) {
            if (getClimbArmAngle() == ClimbConstants.kClimbHomePose) { //to test, may require rounding
                //ensures the homeClimb function will stop running as it is no longer necessary
                setupClimb = 1;         
            }
            else if (getClimbArmAngle() > ClimbConstants.kClimbHomePose) { //adjustment of encoder is necessary for this to work
                //moves climb backwards
                runToAngle(getClimbArmAngle(), ClimbConstants.kClimbHomePose, -1);
            }
            else if (getClimbArmAngle() < ClimbConstants.kClimbHomePose) {
                //moves climb forward
                runToAngle(getClimbArmAngle(), ClimbConstants.kClimbHomePose, 1); //adjustment of encoder is necessary for this to work
            }
        }
    }

    public void lockFunnel() {
        m_leftServo.setAngle(80);
        m_rightServo.setAngle(95);
    }

    public void unlockFunnel() {
        m_leftServo.setAngle(180);
        m_rightServo.setAngle(0);
        isFunnelUnlocked = true;
    }

    public void stopFunnelServos() {
        m_leftServo.setAngle(90);
        m_rightServo.setAngle(90);
    }

    public boolean isFunnelUnlocked() {
        return isFunnelUnlocked;
    }

    //function that is constantly run in code every 20 ms.
    @Override
    public void periodic() {
        //determines if robot is being simulated. if so, will not display the metrics below
        if (!(Robot.isSimulation())) {
            //Displays live motor and limit switch metrics on SmartDashboard
            SmartDashboard.putNumber("Climb Arm Angle", getClimbArmAngle());
            SmartDashboard.putNumber("Climb Motor Speed", m_climbLeaderMotor.get());
            SmartDashboard.putNumber("Climb Motor (" + ClimbConstants.kClimbLeaderMotorCANID + ") Current", m_climbLeaderMotor.getOutputCurrent());
            SmartDashboard.putNumber("Climb Motor (" + ClimbConstants.kClimbFollowerMotorCANID + ") Current", m_climbLeaderMotor.getOutputCurrent());

            //home the climb when the robot boots up
            /* 
            if (setupClimb == 0) {
                homeClimb();
            }
            //sets climb to attack position afte    r being homed
            else if (setupClimb == 1) {
                setClimbToAttack();
            }
            */
        }
    }

    //simulation code to be updated
    @Override
    public void simulationPeriodic() {
        //Displays live motor and limit switch metrics on SmartDashboard
        //SmartDashboard.putNumber("Encoder reading", getEncoderDistance());
        SmartDashboard.putBoolean("Climb stopped", s_climbLeaderMotor.getAppliedOutput() == 0);
        SmartDashboard.putNumber("Motor Bus Voltage", s_climbLeaderMotor.getBusVoltage());
        SmartDashboard.putNumber("Motor Current", s_climbLeaderMotor.getMotorCurrent());
        SmartDashboard.putNumber("Climb Setpoint", s_climbLeaderMotor.getSetpoint());
        SmartDashboard.putNumber("Applied Output", s_climbLeaderMotor.getAppliedOutput());
        SmartDashboard.putNumber("Climb Velocity", s_climbLeaderMotor.getVelocity());
        SmartDashboard.putNumber("Climb Position", s_climbLeaderMotor.getPosition());


        s_climbLeaderMotor.iterate(m_climbLeaderMotor.getAppliedOutput(), 12, 0.02);
        m_climbSim.setInput(s_climbLeaderMotor.getVelocity() * RobotController.getBatteryVoltage());
        m_climbSim.update(0.020);
        s_absoluteEncoder.set(m_climbSim.getPositionMeters());
        RoboRioSim.setVInVoltage(
                    BatterySim.calculateDefaultBatteryLoadedVoltage(m_climbSim.getCurrentDrawAmps()));
            
    }
}
