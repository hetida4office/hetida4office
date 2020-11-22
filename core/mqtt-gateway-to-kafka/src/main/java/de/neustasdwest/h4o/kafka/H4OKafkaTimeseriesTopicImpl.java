package de.neustasdwest.h4o.kafka;

import de.neustasdwest.h4o.common.model.Measurement;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Component;

import java.util.List;


@Component
@Qualifier("timeScaleDbKafkaGateway")
@Slf4j
public class H4OKafkaTimeseriesTopicImpl implements H4OKafkaTimeseriesTopic {
    private final H4OKafkaTimeseriesProducer h4oKafkaTimeseriesProducer;

    public H4OKafkaTimeseriesTopicImpl(final H4OKafkaTimeseriesProducer h4oKafkaTimeseriesProducer) {
        this.h4oKafkaTimeseriesProducer = h4oKafkaTimeseriesProducer;
    }

    @Override
    public void sendTimeSeriesDecimals(final List<Measurement> measurement) {
        h4oKafkaTimeseriesProducer.sendDecimalValues(measurement);
    }
}
