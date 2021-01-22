package de.neustasdwest.h4o.connectormqtt.client.subscriber;

import com.hivemq.client.mqtt.datatypes.MqttQos;
import de.neustasdwest.h4o.common.model.Measurement;
import de.neustasdwest.h4o.connectormqtt.client.H4OSingletonMqttClient;
import de.neustasdwest.h4o.kafka.H4OKafkaTimeseriesTopic;
import lombok.extern.log4j.Log4j2;

import java.math.BigDecimal;
import java.time.Instant;
import java.util.Collections;
import java.util.Objects;

import static java.nio.charset.StandardCharsets.UTF_8;

@Log4j2
public class H4OMqttTopicToKafkaTimeseriesBridge {
    private final String topicName;

    public H4OMqttTopicToKafkaTimeseriesBridge(final String topicName) {
        this.topicName = topicName;
    }

    public void forwardMqttTo(final H4OKafkaTimeseriesTopic h4oKafkaTimeseriesTopic) {
        H4OSingletonMqttClient.getInstance().toAsync().subscribeWith()
                .topicFilter(topicName)
                .qos(MqttQos.EXACTLY_ONCE)
                .callback(mqtt5Publish -> {
                    final String topic = mqtt5Publish.getTopic().toString();
                    final Measurement measurement = buildMeasurement(topic, mqtt5Publish.getPayloadAsBytes());
                    log.info("Send {} to kafka...", measurement);
                    h4oKafkaTimeseriesTopic.sendMeasurements(Collections.singletonList(measurement));
                })
                .send();
    }

    private Measurement buildMeasurement(final String channelId, final byte[] value) {
        final BigDecimal measurement = convertRawValue(value, channelId);
        return Measurement.builder()
                .channelId(channelId)
                .measurement(measurement)
                .timestamp(Instant.now())
                .build();
    }

    private BigDecimal convertRawValue(final byte[] value, final String channelId) {
        Objects.requireNonNull(topicName, "Topic name must not be null!");
        Objects.requireNonNull(value, "RAW Measurement value must not be null!");
        final String converted = new String(value, UTF_8);
        final double measurement = Double.parseDouble(converted);
        BigDecimal h4oMeasurement;
        final String[] channelIdParts = channelId.split("/");
        switch (channelIdParts[channelIdParts.length - 1]) {
            case "temp":
                h4oMeasurement = BigDecimal.valueOf(measurement / 100L);
                break;

            case "hum":
                h4oMeasurement = BigDecimal.valueOf(measurement / 1000L);
                break;

            case "press":
                h4oMeasurement = BigDecimal.valueOf(measurement / 10000L);
                break;

            case "bat":
                h4oMeasurement = BigDecimal.valueOf(measurement / 1000000L);
                break;

            default:
                h4oMeasurement = BigDecimal.valueOf(measurement);
        }
        log.info("Topic -> {} :: Measurement -> {}", topicName, h4oMeasurement);
        return h4oMeasurement;
    }
}
