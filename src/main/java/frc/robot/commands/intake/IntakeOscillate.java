package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;
import frc.robot.testingdashboard.Command;

public class IntakeOscillate extends Command {
    private final Intake m_intake;
    private final double m_intakeSpeed;

    public IntakeOscillate() {
        super(Intake.getInstance(), "Intake", "IntakeOscillate");

        m_intake = Intake.getInstance();
        m_intakeSpeed = 0.5;

        addRequirements(m_intake);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute() {
        if ((Timer.getFPGATimestamp()%0.5) < 0.175) {
            m_intake.spinOut(m_intakeSpeed);
        } else {
            m_intake.spinIn(m_intakeSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    
}
