package de.neustasdwest.h4o.connectormqtt.client.builder;

import lombok.extern.log4j.Log4j2;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

import static java.lang.String.join;

@Log4j2
@Component
public class H4OTopicNamesBuilder implements TopicNamesBuilder {
    private final String topicPrefix;
    private final String[] topicsSuffixes;
    private final String[] devicesNames;

    public H4OTopicNamesBuilder(@Value("${h4o.device.topic.prefix}") final String topicPrefix,
                                @Value("${h4o.device.topic.suffixes}") final String[] topicsSuffixes,
                                @Value("${h4o.devices.names}") final String[] devicesNames) {
        this.topicPrefix = topicPrefix;
        this.topicsSuffixes = topicsSuffixes;
        this.devicesNames = devicesNames;
    }

    @Override
    public Set<String> buildTopicNames() {
        Objects.requireNonNull(topicPrefix, "Topic prefix must not be null");
        Objects.requireNonNull(topicsSuffixes, "Topics suffixes must not be null");
        Objects.requireNonNull(devicesNames, "Devices names must not be null");
        log.info("Generating topic names...");
        final Set<String> topics = new HashSet<>();
        Arrays.asList(topicsSuffixes).forEach(suffix -> {
            Arrays.asList(devicesNames).forEach(deviceName -> {
                final String newTopicName = join("/", topicPrefix, deviceName, suffix);
                log.info(newTopicName);
                topics.add(newTopicName);
            });
        });
        return topics;
    }
}
