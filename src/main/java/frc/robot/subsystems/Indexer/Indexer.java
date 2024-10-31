package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.time.Instant;
import java.util.concurrent.atomic.AtomicBoolean;
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
        registerStateCommand(State.SOFT_E_STOP, new InstantCommand(io::stop));

        registerStateCommand(State.IDLE, new InstantCommand(io::stop)); //maybe add a conditional here if decided its needed to check if theres a note
                                                                        
        registerStateCommand(State.INDEXING, new SequentialCommandGroup(
            indexCommand()
                .withTimeout(Hardware.indexerTimeout) //stop it in case the indexer is having a hard time
                .andThen(new ConditionalCommand(
                    transitionCommand(State.HAS_NOTE), //transition to has note if we have the note correctly
                    new ConditionalCommand(
                        transitionCommand(State.STUCK_NOTE), //transition to stuck note if we have a note, but can't index it
                        transitionCommand(State.LOST_NOTE), //transition to lost note if we don't have the note
                        () -> ringPresent()), 
                    () -> hasNoteCorrectly()))
        ));

        registerStateCommand(State.STUCK_NOTE, new SequentialCommandGroup(
            new InstantCommand((() -> io.setBeltTargetVelocity(Hardware.passThroughSpeed)))
        ));

        registerStateCommand(State.AWAITING_RING_BACK, new SequentialCommandGroup(
            new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.indexingSpeed)),
            new WaitUntilCommand(() -> ringPresent()),
            transitionCommand(State.INDEXING)
        ));
    }

    public void registerTransitions() {
        addOmniTransition(State.SOFT_E_STOP);
        addOmniTransition(State.IDLE);
        addOmniTransition(State.INDEXING);
        addOmniTransition(State.PASS_THROUGH);

        addTransition(State.HAS_NOTE, State.FEED_TO_SHOOTER);
    }

    public boolean ringPresent() {
        return inputs.prox1Tripped || inputs.prox2Tripped || inputs.prox3Tripped;
    }

    public boolean hasNoteCorrectly (){
        return inputs.prox1Tripped && inputs.prox2Tripped && !inputs.prox3Tripped;
    }

    public boolean allProxActive () {
       return inputs.prox1Tripped && inputs.prox2Tripped && inputs.prox3Tripped;
    }

    public boolean tooFar (){
        return inputs.prox3Tripped || inputs.prox2Tripped;
    }

    private Command indexCommand() {
        AtomicBoolean isFinished = new AtomicBoolean(false);

        return new FunctionalCommand(
            () -> isFinished.set(false), 
            () -> {

                //check if we have the note correctly
                if (hasNoteCorrectly()) isFinished.set(true);

                if (!isFinished.get()) {
                    
                    //no proxes active, we lost the ring
                    if (!ringPresent()) isFinished.set(true);

                    //check if prox2 or prox3 are tripped, if they are, the note is too far (you check prox2 because if it didnt trip
                    //hasNoteCorrectly(), then it could be in a spot where it only triggers prox1, not prox2 or prox3)
                    else if (inputs.prox2Tripped || inputs.prox3Tripped) io.setBeltTargetVelocity(-Hardware.indexingSpeed);
                    
                    //index forward
                    else io.setBeltTargetVelocity(Hardware.indexingSpeed);
                }
            }, 
            (interrupeted) -> io.stop(), 
            isFinished::get
            );
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
        SOFT_E_STOP,
        INDEXING,
        HAS_NOTE,
        PASS_THROUGH,
        FEED_TO_SHOOTER,
        AWATING_RING_FRONT,
        AWAITING_RING_BACK,
        LOST_NOTE,
        STUCK_NOTE
    }
}
