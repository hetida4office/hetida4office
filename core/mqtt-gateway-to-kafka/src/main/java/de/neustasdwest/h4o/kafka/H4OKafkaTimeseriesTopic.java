package de.neustasdwest.h4o.kafka;

import de.neustasdwest.h4o.common.model.Measurement;

import java.util.List;

public interface H4OKafkaTimeseriesTopic {
    void sendMeasurements(List<Measurement> measurements);
}
