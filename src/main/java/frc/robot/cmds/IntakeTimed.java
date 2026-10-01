package frc.robot.cmds;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;
import lib.Commands.TimedCommand;

public class IntakeTimed extends TimedCommand {
    private final Intake intake;

    public IntakeTimed(Intake intake, double seconds, Subsystem... requirments) {
        super(seconds, requirments);
        this.intake = intake;
    }

    @Override
    public void init() {

    }

    @Override
    public void exec() {
        intake.intake().run();
    }

    @Override
    public void end() {
        intake.stopIntake().run();
    }
    
}
