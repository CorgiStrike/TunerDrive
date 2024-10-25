package frc.robot.Motors.tuning;

import frc.robot.Motors.talonfx.PIDSVGains;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class LoggedTunablePIDSV {

  private final LoggedDashboardPIDSV dashboardPIDSV;
  private final BooleanSupplier allowTuningSupplier;

  public LoggedTunablePIDSV(
      String name, PIDSVGains defaultValue, BooleanSupplier allowTuningSupplier) {
    this.allowTuningSupplier = allowTuningSupplier;

    dashboardPIDSV = new LoggedDashboardPIDSV(name, defaultValue);
  }

  public PIDSVGains get() {
    if (allowTuningSupplier.getAsBoolean()) {
      return dashboardPIDSV.get();
    } else {
      // reset dashboard number to default and return the default
      PIDSVGains defaultValue = dashboardPIDSV.getDefault();
      dashboardPIDSV.set(defaultValue);
      return defaultValue;
    }
  }

  public void setOnChange(Consumer<PIDSVGains> toRun) {
    dashboardPIDSV.setOnChange(toRun);
  }

  public void set(PIDSVGains value) {
    dashboardPIDSV.set(value);
  }

  public void setDefault(PIDSVGains value) {
    dashboardPIDSV.setDefault(value);
  }

  public PIDSVGains getDefault() {
    return dashboardPIDSV.getDefault();
  }

  public boolean diff(PIDSVGains val) {
    return !get().equals(val);
  }
}
