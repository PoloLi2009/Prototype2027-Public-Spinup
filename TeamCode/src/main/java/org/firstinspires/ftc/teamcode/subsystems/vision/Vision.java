package org.firstinspires.ftc.teamcode.subsystems.vision;

/** No image processing here. Your vision code publishes a coherent observation. */
public class Vision implements AimingInput {
    private volatile Observation observation = new Observation(
            Double.NaN, Double.NaN, Double.NaN,
            Cell.UNKNOWN, Cell.UNKNOWN, 0, false);

    public void update(Observation observation) {
        if (observation == null) throw new IllegalArgumentException("observation is null");
        this.observation = observation;
    }

    @Override
    public Observation getObservation() {
        return observation;
    }
}
