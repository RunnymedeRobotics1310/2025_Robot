package frc.robot;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

import static frc.robot.RunnymedeUtils.calculateSpeed;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class RunnymedeUtilsTests {

  @ParameterizedTest
  @CsvSource({
    "6.5, 0.5, 0.3, 4.8, .75, .75, 1.36",
    "6.5, 0, 0.3, 4.8, .75, .75, .96",
    ".8, 5, 0.3, 4.8, .75, .75, 4.51",
    ".8, 4.51, 0.3, 4.8, .75, .75, 4.57",
    ".7, 4.0, 0.3, 4.8, .75, .75, 3.32",
    "7, 1.3, 0.3, 4.8, .75, .75, 2.00",
    "4, 4.8, 1.0, 4.8, 1.0, 1.5, 4.8",
  })
  void calcAndPrint(
      double distToTravel,
      double currentSpeed,
      double minSpeed,
      double maxSpeed,
      double accelZone,
      double decelZone,
      double expectedSpeed) {
    double s = calculateSpeed(distToTravel, currentSpeed, minSpeed, maxSpeed, accelZone, decelZone);
    String str =
        String.format(
            "toTravel: %.2f currSpd: %.2f minSpd: %.2f maxSpd: %.2f accelZone: %.2f decelZone: %.2f s: %.2f, expectedSpd: %.2f\n",
            distToTravel, currentSpeed, minSpeed, maxSpeed, accelZone, decelZone, s, expectedSpeed);
    System.out.println(str);
    assertEquals(expectedSpeed, s, 1e-2, str);
  }
}
