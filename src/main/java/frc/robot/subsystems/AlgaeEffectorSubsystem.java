package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;

import java.util.Hashtable;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeEffectorSubsystem extends SubsystemBase {

    //The beam break sensor 
    private DigitalInput beamBreakSensor = new DigitalInput(OperatorConstants.beamBreakPort); 

    // all the motors, and their configs, are here 
    //The intake motor
    private SparkMax groundIntakeRoller = new SparkMax(OperatorConstants.groundIntakeRollerID, MotorType.kBrushless); 
    private final SparkMaxConfig groundIntakeRollerConfig = new SparkMaxConfig(); 

    private final SparkMax groundIntakePivot= new SparkMax(IntakeConstants.intakeCANid,MotorType.kBrushless); 
    private final SparkMaxConfig groundIntakePivotConfig = new SparkMaxConfig(); 

    private final SparkMax  elevatorManipulator = new SparkMax(OperatorConstants.elevatorManipulatorID, MotorType.kBrushless);
    private final SparkMaxConfig elevatorManipulatorConfig = new SparkMaxConfig();

    //
    private double targetAngle;

    // the pid controller 
    private final PIDController algaeArmController = new PIDController(OperatorConstants.algaeArmControllerKp,OperatorConstants.algaeArmControllerKi, OperatorConstants.algaeArmControllerKd);

    //the encoder 
    private final DutyCycleEncoder  armPivotEncoder = new DutyCycleEncoder(OperatorConstants.armPivotEncoderPin);

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

        groundIntakePivotConfig.smartCurrentLimit(OperatorConstants.groundIntakePivotLimit,OperatorConstants.groundIntakePivotLimit);

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

   public void setGroundAlgaeVoltage(int choice){
    if (choice==1){
        groundIntakeRoller.setVoltage(OperatorConstants.intakeVoltage);
    }
    else if (choice==2){
        groundIntakeRoller.setVoltage(OperatorConstants.outtakeVoltage);
    }
    else {
        groundIntakeRoller.stopMotor();
    }
   }
    //Check if the beam break sensor detects algae 
    public boolean hasAlgae(){
        return beamBreakSensor.get();
    }
  

    public void setElevatorManipulator(int choice){
        if (choice==1){
            elevatorManipulator.setVoltage(OperatorConstants.elevatorManipulatorForward);
        }
        else if (choice==2){
            elevatorManipulator.setVoltage(OperatorConstants.elevatorManipulatorReverse);
        }
        else {
            elevatorManipulator.stopMotor();
        }
    }

    @Override
    public void periodic() {

        double output = algaeArmController.calculate(armPivotEncoder.get(), targetAngle);

        groundIntakePivot.set(output);

            if (hasAlgae()&&(groundIntakeRoller.get()<0)){ //groundIntakeRoller.get() returns the rpm value of the motor
                groundIntakeRoller.stopMotor();            //which can be -1 to +1 depending on the speed and direction of the motor
            }
    }

    public void setArmAngle(double targetAngleDegrees) {
        // Check if the target angle is within the valid range
        if (targetAngleDegrees >= OperatorConstants.MinAngle && targetAngleDegrees <= OperatorConstants.MaxAngle) {
            targetAngle = targetAngleDegrees;
            

        } else {
            
        }
       
    }

    public void armStop(){ //might not need

        if (armPivotEncoder.get()<=10){

            groundIntakePivot.stopMotor();
        }

    }

    //Move to a certain angle using PID method
    public void setArmAngle(int angle){
        if ((armPivotEncoder.get()-angle)>2||(armPivotEncoder.get()-angle)<-2){
            groundIntakeRoller.set(algaeArmController.calculate(armPivotEncoder.get(), angle));
        }
        else {
            groundIntakeRoller.stopMotor();
        }

    }

}
