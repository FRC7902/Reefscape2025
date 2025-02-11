package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeEffectorSubsystem extends SubsystemBase {

    //The beam break sensor 
    private DigitalInput beamBreakSensor = new DigitalInput(OperatorConstants.beamBreakPort); 

    // all the motors, and their configs, are here 
    //The intake motor
    private SparkMax groundIntakeRoller = new SparkMax(0, MotorType.kBrushless); 
    private final SparkMaxConfig groundIntakeRollerConfig = new SparkMaxConfig(); 

    private final SparkMax groundIntakePivot= new SparkMax(IntakeConstants.intakeCANid,MotorType.kBrushless); 
    private final SparkMaxConfig groundIntakePivotConfig = new SparkMaxConfig(); 

    private final SparkMax  elevatorManipulator = new SparkMax(1, MotorType.kBrushless);
    private final SparkMaxConfig elevatorManipulatorConfig = new SparkMaxConfig();


    // the pid controller 
    private final PIDController algaeArmController = new PIDController(0.02, 0, 0);

    //the encoder 
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    //constructor
    public AlgaeEffectorSubsystem() { 
        
        //configs for the intake motor 
        //Set the current limit for the intake motor 
        groundIntakeRollerConfig.smartCurrentLimit(OperatorConstants.motorCurrentLimit,OperatorConstants.motorCurrentLimit); 

        //dont invert the intake motor 
        groundIntakeRollerConfig.inverted(false); 

        //configure the intake motor 
        groundIntakeRoller.configure(groundIntakeRollerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        //configs for the intake motor 

        //configs for the ground intake pivot motor
        groundIntakePivotConfig.inverted(false);

        groundIntakePivotConfig.smartCurrentLimit(OperatorConstants.motorCurrentLimit,OperatorConstants.motorCurrentLimit);

        groundIntakePivot.configure(groundIntakePivotConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        //configs for the elevator manipulator motor
        elevatorManipulatorConfig.smartCurrentLimit(OperatorConstants.motorCurrentLimit,OperatorConstants.motorCurrentLimit);
        
        elevatorManipulatorConfig.inverted(false);  //dont invert the elevator manipulator motor

        elevatorManipulator.configure(elevatorManipulatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //configs for the elevator manipulator motor

        //set the PID controller for the algae arm 
      //  algaeArmController.setTolerance(0.1);
      //  algaeArmController.setContinuous(true);
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
        groundIntakeRoller.stopMotor();
    }

    public void startElevatorManipulator(){
           elevatorManipulator.setVoltage(7);
    }

    public void stopElevatorManipulator(){
        elevatorManipulator.stopMotor();
    }

    public void reverseElevatorManipulator(){
        elevatorManipulator.setVoltage(-7);
    }


    public void armUp(){
       groundIntakePivot.setVoltage(-7);
    }

    public void armDown(){
        groundIntakePivot.setVoltage(7);
    }

    public void armStop(){

        if (encoder.get()<=10){

            groundIntakePivot.stopMotor();
        }

    }


    
    //Move to a certain angle using PID method

    

    public void pidArmHorizontal(){
        
        groundIntakeRoller.set(algaeArmController.calculate(encoder.get(), OperatorConstants.horizontalArmAngleSetpoint));

    }


    public void pidArmUp(){
        
        groundIntakeRoller.set(algaeArmController.calculate(encoder.get(), OperatorConstants.FullyUpArmAngleSetpoint));

    }

    public void pidArmDown(){
        
        groundIntakeRoller.set(algaeArmController.calculate(encoder.get(), OperatorConstants.FullyDownArmAngleSetpoint));

    }
    

}

  
