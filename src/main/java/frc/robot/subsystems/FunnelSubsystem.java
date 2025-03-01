// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class FunnelSubsystem extends SubsystemBase {

    private boolean isFunnelUnlocked;

    private final Servo m_leftServo = new Servo(ClimbConstants.kLeftServoID);
    private final Servo m_rightServo = new Servo(ClimbConstants.kRightServoID);

    /** Creates a new FunnelSubsystem. */
    public FunnelSubsystem() {
        lockFunnel();
        isFunnelUnlocked = false;
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
