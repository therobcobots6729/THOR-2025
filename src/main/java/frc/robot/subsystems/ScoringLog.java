package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ScoringLog extends SubsystemBase {
    public BooleanSupplier override;
    public ScoringLog(BooleanSupplier override){
        this.override = override;
    }
    private List<ScoredPiece> scoredPieces = new ArrayList<>();

    // Adds a scored piece to the log
    public void logScoredPiece(double tagID, double height) {
        scoredPieces.add(new ScoredPiece(tagID, height));
    }

    // Checks if a slot is available
    public boolean isSlotAvailable(double tagID, double height) {
        for (ScoredPiece piece : scoredPieces) {
            if (piece.tagID == tagID && piece.height == height && override.getAsBoolean() == false) {
                return false; // Slot is occupied
            }
            else if (piece.tagID == tagID && piece.height == height && override.getAsBoolean()== true){
                return true;
            }
            else if (piece.tagID != tagID && piece.height != height && override.getAsBoolean()== true){
                return false;
            }
            
        }
        return true;
         // Slot is open
    }

    // Clears the log (useful between matches)
    public void resetLog() {
        scoredPieces.clear();
    }

    // Internal class to track a scored piece
    private static class ScoredPiece {
        double tagID, height;

        public ScoredPiece(double tagID, double height) {
            this.tagID = tagID;
            this.height = height;
        }
    }
}

