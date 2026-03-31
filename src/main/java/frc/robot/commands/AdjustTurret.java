package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tourelle;

public class AdjustTurret extends Command {
        Tourelle turret;

        public AdjustTurret(Tourelle turret) {
            this.turret = turret;
        }
    
        @Override
        public void initialize() {
            
        }
    
        @Override
        public void execute() {
        }
    
        @Override
        public void end(boolean interrupted) {
        }
    
        @Override
        public boolean isFinished() {
            return false;
        }

}
