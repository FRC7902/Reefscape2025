package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeEffectorSubsystem extends SubsystemBase {

    //The beam break sensor 
    private DigitalInput beamBreakSensor = new DigitalInput(OperatorConstants.beamBreakPort); 

    //The intake motor
    private SparkMax intakeMotor = new SparkMax(0, MotorType.kBrushless); 

    //The SparkMaxConfig object for the intake motor
    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig(); 
    
    //constructor
    public AlgaeEffectorSubsystem() { 
        

        //Set the current limit for the intake motor 
        intakeMotorConfig.smartCurrentLimit(OperatorConstants.motorCurrentLimit,OperatorConstants.motorCurrentLimit); 

        //dont invert the intake motor 
        intakeMotorConfig.inverted(false); 

        //configure the intake motor 
        intakeMotor.configure(intakeMotorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

   //sets the voltage for the algae intake motor, when it's trying to accept the algae 
    public void algaeIntake() {
        intakeMotor.setVoltage(OperatorConstants.intakeVoltage);
    }

    //sets the voltage for the algae intake motor, when it's trying to shoot the algae 
    public void algaeOuttake() {
        intakeMotor.setVoltage(OperatorConstants.outtakeVoltage);
    }

    //Check if the beam break sensor detects algae 
    public boolean hasAlgae(){
        return beamBreakSensor.get();
    }
  
    //Stop the algae intake motor 
    public void algaeStop() {
        intakeMotor.setVoltage(0);
    }

    //Move to a certain angle using PID method

    public void goToHeight(){
        
    }

}

  
