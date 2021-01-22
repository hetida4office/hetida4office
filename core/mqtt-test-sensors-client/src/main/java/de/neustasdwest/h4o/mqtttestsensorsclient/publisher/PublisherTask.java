package de.neustasdwest.h4o.mqtttestsensorsclient.publisher;

import com.hivemq.client.mqtt.datatypes.MqttQos;
import de.neustasdwest.h4o.mqtttestsensorsclient.client.H4OMqttPublisherClientImpl;
import lombok.extern.log4j.Log4j2;

import java.util.Set;

import static de.neustasdwest.h4o.mqtttestsensorsclient.publisher.PublishUtils.getFakeTempMeasurement;
import static de.neustasdwest.h4o.mqtttestsensorsclient.publisher.PublishUtils.getRandomMeasurement;

@Log4j2
public class PublisherTask implements Runnable {
    private final Set<String> topicNames;

    public PublisherTask(final Set<String> topicNames) {
        this.topicNames = topicNames;
    }

    @Override
    public void run() {
        if (!H4OMqttPublisherClientImpl.getInstance().getState().isConnected()) {
            log.error("Error while trying to publish -> client is not connected");
            return;
        }

        this.topicNames.forEach(topic -> {
            int measurement = topic.equals("TEMP") ? getFakeTempMeasurement() : getRandomMeasurement();
            H4OMqttPublisherClientImpl.getInstance().publishWith().topic(topic)
                    .payload(String.valueOf(measurement).getBytes()).qos(MqttQos.EXACTLY_ONCE).send()
                    .whenComplete((mqtt3Publish, throwable) -> {
                        if (throwable != null) {
                            log.error("Error while publishing random value -> {} :: error -> {}", measurement,
                                    throwable);
                        } else {
                            log.info("Published -> {} to topic -> {}", measurement, topic);
                        }
                    });
        });
    }
}
