package frc.robot.telemetry;

import java.util.LinkedList;

public class TimeSeriesMetric {
  private static class DataSample {
    double value;
    long timestamp;

    DataSample(double value, long timestamp) {
      this.value = value;
      this.timestamp = timestamp;
    }
  }

  private final LinkedList<DataSample> samples = new LinkedList<>();
  private static final long TIME_WINDOWS_MILLIS = 2000; // 2 seconds

  public synchronized void add(double value) {
    long currentTime = System.currentTimeMillis();
    samples.add(new DataSample(value, currentTime));
    removeOldSamples(currentTime);
  }

  public synchronized double getMinValue() {
    //    removeOldSamples(System.currentTimeMillis());
    return samples.stream().mapToDouble(sample -> sample.value).min().orElse(Double.NaN);
  }

  public synchronized double getMaxValue() {
    //    removeOldSamples(System.currentTimeMillis());
    return samples.stream().mapToDouble(sample -> sample.value).max().orElse(Double.NaN);
  }

  private synchronized void removeOldSamples(long currentTime) {
    while (!samples.isEmpty() && currentTime - samples.getFirst().timestamp > TIME_WINDOWS_MILLIS) {
      samples.removeFirst();
    }
  }
}
