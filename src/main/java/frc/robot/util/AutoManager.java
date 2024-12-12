package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The {@link AutoManager} can combine PathPlanner autonomous routes and custom 
 * autonomous routes into one {@link SendableChooser}. It also has support for delay before running autonomous commands.
 */

public class AutoManager {
    private SendableChooser<Command> autoChooser;

    public AutoManager() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto", autoChooser);
    }

    public void addCustomAutoCommand(String name, Command autoCommand) {
        autoChooser.addOption(name, autoCommand);
    }

    public Command getSelectedCommand() {
        return autoChooser.getSelected();
    }
}