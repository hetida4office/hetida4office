package de.neustasdwest.h4o.mqtttestsensorsclient.publisher;

import java.util.Random;

public class PublishHelperUtils {
    private static final Random r = new Random();

    private static int getRandomNumber(int low, int high) {
        return r.nextInt(high-low) + low;
    }

    public static int getFakeTempMeasurement() {
        return getRandomNumber(1000, 2500);
    }

    public static int getFakeCO2Measurement() {
        return getRandomNumber(100, 3500);
    }
}
