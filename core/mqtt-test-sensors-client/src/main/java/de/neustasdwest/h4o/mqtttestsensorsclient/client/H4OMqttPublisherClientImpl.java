package de.neustasdwest.h4o.mqtttestsensorsclient.client;

import com.hivemq.client.mqtt.mqtt5.Mqtt5AsyncClient;
import com.hivemq.client.mqtt.mqtt5.Mqtt5Client;
import lombok.extern.log4j.Log4j2;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import java.util.UUID;
import java.util.concurrent.TimeUnit;

@Log4j2
@Component
public class H4OMqttPublisherClientImpl implements H4OMqttPublisherClient {
    private static Mqtt5AsyncClient client;
    private final String mqttBrokerHost;

    public H4OMqttPublisherClientImpl(@Value("${mqtt.broker.host}") final String mqttBrokerHost,
                                      @Value("${mqtt.broker.port}") final Integer mqttBrokerPort) {
        this.mqttBrokerHost = mqttBrokerHost;
        if (client == null) {
            final String uuid = UUID.randomUUID().toString();
            client = Mqtt5Client.builder()
                    .identifier(uuid)
                    .serverHost(this.mqttBrokerHost)
                    .serverPort(mqttBrokerPort)
                    .automaticReconnect()
                    .initialDelay(500, TimeUnit.MILLISECONDS)
                    .maxDelay(1, TimeUnit.MINUTES)
                    .applyAutomaticReconnect()
                    .buildAsync();

        }
    }

    public static Mqtt5AsyncClient getInstance() {
        return client;
    }

    @Override
    public void connect() {
        if (!client.getState().isConnected()) {
            log.info("Connection to client at {}...", this.mqttBrokerHost);
            client.connect();
            return;
        }
        log.info("Client was already connected...");
    }
}
