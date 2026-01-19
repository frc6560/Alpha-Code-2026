package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.superstructure.Sotm;
import frc.robot.ManualControls;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.ShooterLUT;
import frc.robot.subsystems.superstructure.Hood;

public class SotmCommands extends Command {
    public SotmCommands(Sotm sotm, SwerveSubsystem drivebase, Flywheel flywheel, Turret turret, ManualControls controls, ShooterLUT shooterLUT, Hood hood) {
        
    }
}
