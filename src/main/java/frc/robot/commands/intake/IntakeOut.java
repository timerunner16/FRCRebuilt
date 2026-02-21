package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import frc.robot.testingdashboard.Command;
import frc.robot.utils.Configuration;

public class IntakeOut extends Command{

    Intake m_intake;
    Configuration cfg;
    

    public IntakeOut(){
        super(Intake.getInstance(), "Intake", "Ground Intake");

        m_intake = Intake.getInstance();
        cfg = Configuration.getInstance();
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute() {

        m_intake.spinOut(cfg.getInt("Intake", "intakeSpeed")); 
        //TODO use same variable as IntakeIn or vice versa 
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
