package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Indexer.Hardware;
import frc.robot.SMF.StateMachine;
import frc.robot.subsystems.Indexer.IndexerIO.*;


public class Indexer extends StateMachine<Indexer.State>{
    private final IndexerIO io;

    private final IndexerIOInputs inputs = new IndexerIOInputs();

    public Indexer(IndexerIO io){
        super("Indexer",State.UNDETERMINED,State.class);
        this.io = io;

        io.updateInputs(inputs);

        registerStateCommands();
        registerTransitions();
    }


    public void registerStateCommands() {
        registerStateCommand(State.IDLE, new ConditionalCommand(transitionCommand(State.INDEXING), new InstantCommand(io::stop), () -> ringPresent()));

        registerStateCommand(State.INDEXING, 
            new ConditionalCommand(
                transitionCommand(State.HAS_NOTE), 
                new ConditionalCommand(
                    transitionCommand(State.RING_BACK),
                    new SequentialCommandGroup(
                        new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.indexingSpeed)),
                        new WaitUntilCommand(() -> hasnoteCorrectly() || tooFar()),
                        new ConditionalCommand(transitionCommand(State.HAS_NOTE), transitionCommand(State.RING_BACK), () -> hasnoteCorrectly())
                    ), 
                    () -> tooFar()
            ), 
            () -> hasnoteCorrectly())
        );

        registerStateCommand(State.PASS_THROUGH, new SequentialCommandGroup(
            new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.passThroughSpeed)),
            new WaitUntilCommand(() -> !ringPresent()),
            new InstantCommand(io::stop),
            transitionCommand(State.IDLE)
        ));

        registerStateCommand(State.HAS_NOTE, new InstantCommand(io::stop));

        registerStateCommand(State.FEED_TO_SHOOTER, new SequentialCommandGroup(
            new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.feedSpeed)),
            new WaitUntilCommand(() -> !ringPresent()),
            new InstantCommand(io::stop),
            transitionCommand(State.IDLE)
        ));

        registerStateCommand(State.HUMAN_PLAYER_INTAKE, 
            new SequentialCommandGroup(
                new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.humanPlayerIntakeSpeed)),
                new WaitUntilCommand(() -> allProxActive() || hasnoteCorrectly()),
                new ConditionalCommand(transitionCommand(State.HAS_NOTE), transitionCommand(State.RING_BACK), () -> hasnoteCorrectly())
        ));

        registerStateCommand(State.RING_BACK, 
            new SequentialCommandGroup(
                new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.ringBackSpeed)),
                new WaitUntilCommand(() -> hasnoteCorrectly()),
                new InstantCommand(io::stop),
                transitionCommand(State.HAS_NOTE)
            ));

        registerStateCommand(State.SOFT_E_STOP, new InstantCommand(io::stop));
    }

    public void registerTransitions() {
        addOmniTransition(State.SOFT_E_STOP);
        addOmniTransition(State.IDLE);
        addOmniTransition(State.INDEXING);
        addOmniTransition(State.PASS_THROUGH);

        addTransition(State.HUMAN_PLAYER_INTAKE, State.RING_BACK);
        addTransition(State.INDEXING, State.RING_BACK);
        addTransition(State.RING_BACK, State.HAS_NOTE);
        addTransition(State.INDEXING, State.HAS_NOTE);
        addTransition(State.IDLE, State.HUMAN_PLAYER_INTAKE);
        addTransition(State.HAS_NOTE, State.FEED_TO_SHOOTER);

        removeTransition(State.HAS_NOTE, State.INDEXING);
        removeTransition(State.RING_BACK, State.INDEXING);
    }

    public boolean ringPresent() {
        return inputs.prox1Tripped || inputs.prox2Tripped || inputs.prox3Tripped;
    }

    public boolean hasnoteCorrectly (){
        return inputs.prox1Tripped && inputs.prox2Tripped && !inputs.prox3Tripped;
    }

    public boolean allProxActive () {
       return inputs.prox1Tripped && inputs.prox2Tripped && inputs.prox3Tripped;
    }

    public boolean tooFar (){
        return inputs.prox3Tripped;
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    @Override
    protected void update(){
        io.updateInputs(inputs);
    }

    public enum State{
        UNDETERMINED,
        IDLE,
        INDEXING,
        HAS_NOTE,
        PASS_THROUGH,
        FEED_TO_SHOOTER,
        HUMAN_PLAYER_INTAKE,
        RING_BACK,
        SOFT_E_STOP
    }
}
