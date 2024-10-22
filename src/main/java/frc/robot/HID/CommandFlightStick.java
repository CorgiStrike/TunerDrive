package frc.robot.HID;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandFlightStick extends CommandGenericHID {
  private final FlightStick hid;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandFlightStick(int port) {
    super(port);
    hid = new FlightStick(port);
  }

  public CommandFlightStick(int port, FlightStick hid) {
    super(port);
    this.hid = hid;
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public FlightStick getHID() {
    return hid;
  }

  /**
   * Constructs an event instance around the trigger's digital signal.
   *
   * @return an event instance representing the trigger's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #trigger(EventLoop)
   */
  public Trigger trigger() {
    return trigger(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the trigger's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the trigger's digital signal attached to the given loop.
   */
  public Trigger trigger(EventLoop loop) {
    return hid.trigger(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around the top base's digital signal.
   *
   * @return an event instance representing the top base's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #topBase(EventLoop)
   */
  public Trigger topBase() {
    return topBase(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the top base's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the top base's digital signal attached to the given
   *     loop.
   */
  public Trigger topBase(EventLoop loop) {
    return hid.topBase(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around the top left's digital signal.
   *
   * @return an event instance representing the top left's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #topBase(EventLoop)
   */
  public Trigger topLeft() {
    return topLeft(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the top left's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the top left's digital signal attached to the given
   *     loop.
   */
  public Trigger topLeft(EventLoop loop) {
    return hid.topLeft(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around the top right's digital signal.
   *
   * @return an event instance representing the top right's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #topBase(EventLoop)
   */
  public Trigger topRight() {
    return topRight(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the top right's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the top right's digital signal attached to the given
   *     loop.
   */
  public Trigger topRight(EventLoop loop) {
    return hid.topRight(loop).castTo(Trigger::new);
  }

  /**
   * Get the X axis value of the controller.
   *
   * @return The axis value.
   */
  public double getX() {
    return hid.getXAxis();
  }

  /**
   * Get the Y axis value of the controller.
   *
   * @return The axis value.
   */
  public double getY() {
    return hid.getYAxis();
  }

  /**
   * Get the twist axis value of the controller.
   *
   * @return The axis value.
   */
  public double getTwist() {
    return hid.getTwistAxis();
  }

  /**
   * Get the throttle axis value of the controller. Note that this axis is bound to the range of [0,
   * 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getThrottleAxis() {
    return hid.getThrottleAxis();
  }
}
