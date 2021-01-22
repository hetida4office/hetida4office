package de.neustasdwest.h4o.common.model;

import lombok.*;

import java.io.Serializable;
import java.math.BigDecimal;
import java.time.Instant;

@Getter
@Setter
@ToString
@AllArgsConstructor
@NoArgsConstructor
@Builder
public class Measurement implements Serializable {
    private static final long serialVersionUID = -7327452512286172439L;
    private String channelId;
    private Instant timestamp;
    private BigDecimal measurement;
}
