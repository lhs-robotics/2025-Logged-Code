package frc.robot.subsystems.coral.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Indexer extends SubsystemBase{
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final IndexerIO indexer;

    public Indexer (IndexerIO indexerIO){
        this.indexer = indexerIO;
    }

    @Override
    public void periodic() {
        indexer.updateInputs(inputs);
        Logger.processInputs("Coral/Indexer", inputs);
    }

    public Trigger getAutoLoadTrigger() {
        return indexer.getAutoLoadTrigger();
    }

    public void enableIndexMotor() {
        Logger.recordOutput("Coral/Indexer/enabled", true);
        indexer.enableIndexer();
    }

    public void disableIndexMotor() {
        Logger.recordOutput("Coral/Indexer/enabled", false);
        indexer.disableIndexer();
    }
    
    
}
