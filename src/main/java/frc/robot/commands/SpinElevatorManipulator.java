// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Caeley is supposed to do this
package frc.robot.commands;
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
import frc.robot.Constants.OperatorConstants;

import java.security.Key;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class SpinElevatorManipulator extends SubsystemBase {


    public static class AlgaeManipulatorConstants {
        public static final double IntakeMotorSpeed = 0.5; // Set the appropriate value
    }
    
        public static class IntakeConstants {
            public static final int intakeCANid = 1; // Set the appropriate value
        }
        
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
    
    
        // the pid controller 
        private final PIDController algaeArmController = new PIDController(OperatorConstants.algaeArmControllerKp,OperatorConstants.algaeArmControllerKi, OperatorConstants.algaeArmControllerKd);
    
        //the encoder 
        private final DutyCycleEncoder  armPivotEncoder = new DutyCycleEncoder(OperatorConstants.armPivotEncoderPin);
    
        //constructor
        public SpinElevatorManipulator() { 
            
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
    
        public void armStop(){ //might not need
    
            if (armPivotEncoder.get()<=10){
    
                groundIntakePivot.stopMotor();
            }
    
        }
    
        //Move to a certain angle using PID method
        public void setArmAngle(int angle){
            if (armPivotEncoder.get()-angle>2||armPivotEncoder.get()-angle<-2){
                groundIntakeRoller.set(algaeArmController.calculate(armPivotEncoder.get(), angle));
            }
            else {
                groundIntakeRoller.stopMotor();
            }
        }
        
            public SparkMax elevatorManipulatorsSparkMax = new SparkMax(OperatorConstants.elevatorManipulatorID, MotorType.kBrushless);
            
            public double AlgaeSpeed = AlgaeManipulatorConstants.IntakeMotorSpeed;
    
        public void setElevatorManipulator() {
            elevatorManipulatorsSparkMax.set(speedConvert(AlgaeSpeed));
        }
    
        public void Outtake() {
            elevatorManipulatorsSparkMax.set(speedConvert(-AlgaeSpeed));
        }
    
        public double speedConvert(double inSpeed) {
            if (inSpeed < 0.2 && inSpeed > -0.2) {
                return 0.0;
            }
            return inSpeed;
        }
        
        public void stop() {
            elevatorManipulatorsSparkMax.set(0);
        }
    
        public boolean hasAlgaeTwo() {
            return beamBreakSensor.get();
        }
        @Override
        public void periodic() {
            if (hasAlgaeTwo() && elevatorManipulatorsSparkMax.get() > 0) {
                elevatorManipulatorsSparkMax.stopMotor();
    
            // This will be called once per scheduler run
        }
    }
  
}
