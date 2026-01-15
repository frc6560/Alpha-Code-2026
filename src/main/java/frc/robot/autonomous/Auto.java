package frc.robot.autonomous;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;


/** Defines an Auto class, which is a name, an enum ID, and a command. */
public class Auto {
    private final AutoNames autoRoutine;
    private final String name;
    private Pair<Pose2d, Command> autoCommand;
    private final AutoCommands factory;

    public Auto(AutoNames auto, AutoCommands factory){
        this.autoRoutine = auto;
        this.factory = factory;
        this.autoCommand = null;
        this.name = getAutos().getSecond();
        
    }

    public Command getCommand() {
        update();
        return autoCommand.getSecond();
    }

    public String getName() {
        return name;
    }

    public void update(){
        this.factory.updateAlliance(DriverStation.getAlliance().get());
        Pair<Pair<Pose2d, Command>, String> autos = getAutos();
        this.autoCommand = autos.getFirst();
    }

    public Pair<Pair<Pose2d, Command>, String> getAutos(){
        Pair<Pose2d, Command> command;
        String name;
        switch(autoRoutine){
            case IDLE:
                command = null;
                name = "Idle Left";
                break;
            case TEST:
                command = null;
                name = "Test Auto";
                break;
            default:
                command = Pair.of(new Pose2d(), null);
                name = "";
                throw new IllegalArgumentException("Invalid Auto Routine: " + autoRoutine);
        }
        return Pair.of(command, name);
    }
}
