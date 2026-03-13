package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;


public class ManualClimb extends Command {
    
    private  final Climb climb;
    private  double output;

    public ManualClimb(Climb climb, double output) {
        this.climb = climb;
        this.output = output;

        addRequirements(climb);
    }

      @Override
    public void initialize() {}

    @Override
    public void execute() {
        climb.setMotorSpeed(output, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // climb.keepPosition();
        climb.setMotorSpeed(0, true);
    }
    

}
