package de.neustasdwest.h4o.mqtttestsensorsclient.builder;

import lombok.extern.log4j.Log4j2;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Objects;
import java.util.Set;

@Log4j2
@Component
public class H4OTopicNamesBuilder implements TopicNamesBuilder {
    private final String[] topicSuffixes;
    private final String topicPrefix;
    private final String[] devicesNames;

    public H4OTopicNamesBuilder(@Value("${h4o.device.topic.prefix}") final String topicPrefix,
                                @Value("${h4o.device.topic.suffixes}") final String[] topicSuffixes,
                                @Value("${h4o.devices.names}") final String[] devicesNames) {
        this.topicSuffixes = topicSuffixes;
        this.topicPrefix = topicPrefix;
        this.devicesNames = devicesNames;
    }

    @Override
    public Set<String> buildTopicNames() {
        Objects.requireNonNull(topicSuffixes, "Topic suffixes must not be null");
        Objects.requireNonNull(topicPrefix, "Topics prefix must not be null");
        Objects.requireNonNull(devicesNames, "Devices names must not be null");
        log.info("Generating topic names...");
        final Set<String> topics = new HashSet<>();
        Arrays.asList(topicSuffixes).forEach(suffix -> {
            Arrays.asList(devicesNames).forEach(deviceName -> {
                final String newTopicName = String.join("/", topicPrefix, deviceName, suffix);
                log.info(newTopicName);
                topics.add(newTopicName);
            });
        });
        return topics;
    }
}
