package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class SetShootSpeed extends Command {
    private IndexSubsystem m_index;


    public SetShootSpeed(IndexSubsystem index) {
        index = m_index;
        // addRequirements(___);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_index.shoot();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_index.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
