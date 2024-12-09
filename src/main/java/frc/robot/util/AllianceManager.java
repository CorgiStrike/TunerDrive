package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AllianceManager {
  private static DriverStation.Alliance alliance = DriverStation.Alliance.Red;
  private static boolean overrideAlliance = false;

  // List of runnables to execute whenever the alliance changes
  private static List<Runnable> allianceChangeHooks = new ArrayList<>();

  public static void applyAlliance(Optional<DriverStation.Alliance> newAlliance) {
    if (!overrideAlliance && newAlliance.isPresent()) {
      setAlliance(newAlliance.get());
    }
  }

  public static InstantCommand switchAlliance() {
    return new WhileDisabledInstantCommand(
        () -> {
          setAlliance(alliance == Alliance.Red ? Alliance.Blue : Alliance.Red);
          overrideAlliance = true;
        });
  }

  public static InstantCommand syncAlliance() {
    return new WhileDisabledInstantCommand(
        () -> {
          applyAlliance(DriverStation.getAlliance());
          overrideAlliance = false;
        });
  }

  private static void setAlliance(Alliance newAlliance) {
    alliance = newAlliance;

    allianceChangeHooks.forEach(
        (e) -> {
          e.run();
        });
  }

  public static Alliance getAlliance() {
    return alliance;
  }

  public static void addAllianceChangeHook(Runnable toRun) {
    allianceChangeHooks.add(toRun);
  }
}