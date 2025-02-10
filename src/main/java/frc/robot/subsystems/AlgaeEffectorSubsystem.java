package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeEffectorSubsystem extends SubsystemBase {

    //The beam break sensor 
    private DigitalInput beamBreakSensor = new DigitalInput(OperatorConstants.beamBreakPort); 


 
    //The intake motor
    private SparkMax groundIntakeRoller = new SparkMax(0, MotorType.kBrushless); 

    //The SparkMaxConfig object for the intake motor
    private final SparkMaxConfig groundIntakeRollerConfig = new SparkMaxConfig(); 
    
    //
    private final SparkMax groundIntakePivot= new SparkMax(IntakeConstants.intakeCANid,MotorType.kBrushless); //Caeley can u do this pls🥺
    private final SparkMaxConfig groundIntakePivotConfig = new SparkMaxConfig(); //Caeley can u do this pls🥺
    //
    private final SparkMax  elevatorManipulator = new SparkMax(1, MotorType.kBrushless);
    private final SparkMaxConfig elevatorManipulatorConfig = new SparkMaxConfig();



    //constructor
    public AlgaeEffectorSubsystem() { 
        

        //Set the current limit for the intake motor 
        groundIntakeRollerConfig.smartCurrentLimit(OperatorConstants.motorCurrentLimit,OperatorConstants.motorCurrentLimit); 

        //dont invert the intake motor 
        groundIntakeRollerConfig.inverted(false); 

        //configure the intake motor 
        groundIntakeRoller.configure(groundIntakeRollerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

   //sets the voltage for the algae intake motor, when it's trying to accept the algae 
    public void algaeIntake() {
        groundIntakeRoller.setVoltage(OperatorConstants.intakeVoltage);
    }

    //sets the voltage for the algae intake motor, when it's trying to shoot the algae 
    public void algaeOuttake() {
        groundIntakeRoller.setVoltage(OperatorConstants.outtakeVoltage);
    }

    //Check if the beam break sensor detects algae 
    public boolean hasAlgae(){
        return beamBreakSensor.get();
    }
  
    //Stop the algae intake motor 
    public void algaeStop() {
        groundIntakeRoller.setVoltage(0);
    }

    public void startElevatorManipulator(){
           elevatorManipulator.setVoltage(7);
    }

    public void stopElevatorManipulator(){
        elevatorManipulator.setVoltage(0);
    }

    public void reverseElevatorManipulator(){
        elevatorManipulator.setVoltage(-7);
    }

    //Move to a certain angle using PID method

    public void goToHeight(){
        
    }

}

  
