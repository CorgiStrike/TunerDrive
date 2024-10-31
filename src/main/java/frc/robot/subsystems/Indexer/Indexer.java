package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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

        registerStateCommand(State.IDLE, 
            new ConditionalCommand( //transitions the indexer to INDEXING if a note is detected
                transitionCommand(State.INDEXING), 
                new InstantCommand(io::stop), 
                () -> notePresent()
            )
        ); 
                                                                        
        registerStateCommand(State.INDEXING, new SequentialCommandGroup(
            indexCommand()
                .withTimeout(Hardware.indexerTimeout) //stop it in case the indexer is having a hard time
                .andThen(new ConditionalCommand(
                    transitionCommand(State.HAS_NOTE), //transition to HAS_NOTE if we have the note correctly
                    new ConditionalCommand(
                        transitionCommand(State.STUCK_NOTE), //transition to stuck note if we have a note, but can't index it
                        transitionCommand(State.LOST_NOTE), //transition to lost note if we don't have the note
                        () -> notePresent()), 
                    () -> hasNoteCorrectly()))
        ));

        registerStateCommand(State.STUCK_NOTE, new SequentialCommandGroup(
            new InstantCommand(io::stop), // configure robotContainer to change LEDs
            new WaitUntilCommand(() -> !notePresent()),
            transitionCommand(State.IDLE)
        ));

        registerStateCommand(State.LOST_NOTE, new SequentialCommandGroup(
            new InstantCommand(io::stop) // configure robotContainer to change LEDs/try intaking maybe?
        ));

        registerStateCommand(State.AWAITING_NOTE_BACK, new SequentialCommandGroup(
            new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.indexingSpeed)),
            new WaitUntilCommand(() -> notePresent()),
            transitionCommand(State.INDEXING)
        ));

        registerStateCommand(State.AWAITING_NOTE_FRONT, new SequentialCommandGroup(
            new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.humanPlayerIntakeSpeed)),
            new WaitUntilCommand(() -> notePresent()),
            transitionCommand(State.INDEXING)
        ));

        registerStateCommand(State.PASS_THROUGH, 
            new SequentialCommandGroup(
                new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.passThroughSpeed)),
                new WaitUntilCommand(() -> !notePresent()).withTimeout(Hardware.ejectTimeout),
                new ConditionalCommand(
                    transitionCommand(State.IDLE),
                    transitionCommand(State.STUCK_NOTE),
                    () -> !notePresent()
                )
            )
        );
        
        registerStateCommand(State.FEED_TO_SHOOTER,
            new SequentialCommandGroup(
                new InstantCommand (() -> io.setBeltTargetVelocity(Hardware.feedSpeed)),
                new WaitUntilCommand(() -> !notePresent()).withTimeout(Hardware.shootTimeout),
                new ConditionalCommand(
                    transitionCommand(State.STUCK_NOTE), 
                    transitionCommand(State.IDLE),
                    () -> notePresent())
            )
        );

        registerStateCommand(State.HAS_NOTE, new SequentialCommandGroup(
            new InstantCommand(io::stop),
            new RunCommand(
                () -> {
                    if (!notePresent()) requestTransition(State.IDLE);
                    if (!hasNoteCorrectly()) requestTransition(State.INDEXING);
                }
            )
        ));
    }

    public void registerTransitions() {
        addOmniTransition(State.SOFT_E_STOP);
        addOmniTransition(State.IDLE);
        addOmniTransition(State.INDEXING);
        addOmniTransition(State.PASS_THROUGH);
        addOmniTransition(State.STUCK_NOTE);
        addOmniTransition(State.LOST_NOTE);

        addTransition(State.INDEXING, State.HAS_NOTE);
        addTransition(State.HAS_NOTE, State.FEED_TO_SHOOTER);
        addTransition(State.IDLE, State.AWAITING_NOTE_BACK);
        addTransition(State.IDLE, State.AWAITING_NOTE_FRONT);
    }

    public boolean notePresent() {
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
                    
                    //no proxes active, we lost the NOTE
                    if (!notePresent()) isFinished.set(true);

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
        AWAITING_NOTE_FRONT,
        AWAITING_NOTE_BACK,
        LOST_NOTE,
        STUCK_NOTE
    }
}
