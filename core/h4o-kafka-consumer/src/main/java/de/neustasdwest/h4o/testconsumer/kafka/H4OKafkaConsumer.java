package de.neustasdwest.h4o.testconsumer.kafka;

import de.neustasdwest.h4o.common.model.Measurement;
import de.neustasdwest.h4o.testconsumer.batch.TSWriterRepository;
import lombok.extern.slf4j.Slf4j;
import org.apache.kafka.clients.consumer.ConsumerRecord;
import org.springframework.kafka.annotation.KafkaListener;
import org.springframework.stereotype.Component;

import java.io.IOException;

import static de.neustasdwest.h4o.common.deserializer.MeasurementDeserializer.deserializeMeasurement;
import static java.util.Collections.singletonList;

@Slf4j
@Component
public class H4OKafkaConsumer {
    private final TSWriterRepository tsWriter;

    public H4OKafkaConsumer(final TSWriterRepository tsWriterRepository) {
        this.tsWriter = tsWriterRepository;
    }

    @KafkaListener(topics = "#{'${kafka.topic.channelvalue.name}'}")
    public void listen(final ConsumerRecord<String, byte[]> cr) throws IOException {
        log.info("Consuming Topic {} record from partition {} with offset {}.", cr.topic(), cr.partition(), cr.offset());
        final Measurement measurement = deserializeMeasurement(cr.value());
        this.tsWriter.batchInsertMeasurements(singletonList(measurement));
        log.info("Added Measurement: channelId::{}, timestamp::{}, measurement::{}",
                measurement.getChannelId(),
                measurement.getTimestamp(),
                measurement.getMeasurement());
    }
}
