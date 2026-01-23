package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.superstructure.Snotm;
import frc.robot.ManualControls;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.ShooterLUT;
import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Feeder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SnotmCommand extends Command {

    private final Snotm snotm;
    private final ManualControls controls;
    private final Feeder feeder;

    public SnotmCommand(Snotm snotm, ManualControls controls, Feeder feeder) {
        this.snotm = snotm;
        this.controls = controls;
        this.feeder = feeder;
        addRequirements(snotm);
    }
    

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (controls.BallOut()) {
            feeder.feed();
        }
    }

    public void periodic() {
        snotm.ShootBall();
    }

}
