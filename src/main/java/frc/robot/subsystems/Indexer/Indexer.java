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
        registerStateCommand(State.IDLE, new InstantCommand(io::stop));
        registerStateCommand(State.INDEXING, 
            new SequentialCommandGroup(
                new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.indexingSpeed)),
                new WaitUntilCommand(() -> hasRingCorrectly() || inputs.prox3Tripped),
                new ConditionalCommand(transitionCommand(State.HAS_NOTE), transitionCommand(State.RING_BACK), () -> hasRingCorrectly())
                )
        );
        registerStateCommand(State.PASS_THROUGH, new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.passThroughSpeed)));
        registerStateCommand(State.HAS_NOTE, new InstantCommand(io::stop));
        registerStateCommand(State.FEED_TO_SHOOTER, new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.feedSpeed)));
        registerStateCommand(State.HUMAN_PLAYER_INTAKE, 
            new SequentialCommandGroup(
                new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.humanPlayerIntakeSpeed)),
                new WaitUntilCommand(() -> allProxActive()),
                transitionCommand(State.RING_BACK)
        ));
        registerStateCommand(State.RING_BACK, 
            new SequentialCommandGroup(
                new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.ringBackSpeed)),
                new WaitUntilCommand(() -> hasRingCorrectly()),
                new InstantCommand(io::stop),
                transitionCommand(State.HAS_NOTE)
            )

        );
    }

    public void registerTransitions() {
        addOmniTransition(State.IDLE);
        addOmniTransition(State.INDEXING);
        addOmniTransition(State.PASS_THROUGH);

        addTransition(State.HUMAN_PLAYER_INTAKE, State.RING_BACK);
        addTransition(State.RING_BACK, State.HAS_NOTE);
        addTransition(State.IDLE, State.HUMAN_PLAYER_INTAKE);
        addTransition(State.INDEXING, State.HAS_NOTE);
        addTransition(State.HAS_NOTE, State.FEED_TO_SHOOTER);

        removeTransition(State.HAS_NOTE, State.INDEXING);
        removeTransition(State.RING_BACK, State.INDEXING);
    }

    public boolean hasRingCorrectly (){
        return inputs.prox1Tripped && inputs.prox2Tripped && !inputs.prox3Tripped;
    }

    public boolean allProxActive () {
       return inputs.prox1Tripped && inputs.prox2Tripped && inputs.prox3Tripped;
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
        RING_BACK
    }
}
