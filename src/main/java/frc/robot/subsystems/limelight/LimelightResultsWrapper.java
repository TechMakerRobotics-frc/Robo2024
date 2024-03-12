package frc.robot.subsystems.limelight;

import com.fasterxml.jackson.annotation.JsonProperty;

public class LimelightResultsWrapper {
  @JsonProperty("Results")
  public LimelightResults targetingResults;

  public LimelightResultsWrapper() {
      targetingResults = new LimelightResults();
  }
}