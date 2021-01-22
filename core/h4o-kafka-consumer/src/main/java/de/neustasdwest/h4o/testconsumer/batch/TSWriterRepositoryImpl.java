package de.neustasdwest.h4o.testconsumer.batch;

import de.neustasdwest.h4o.common.model.Measurement;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.jdbc.core.JdbcTemplate;
import org.springframework.stereotype.Repository;

import java.sql.Timestamp;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.util.List;

@Slf4j
@Repository
public class TSWriterRepositoryImpl implements TSWriterRepository {
    private static final String INSERT_MEASUREMENTS_QUERY = "insert into timeseries.measurements (channel_id, timestamp, measurement) values(?,?,?)";

    private final JdbcTemplate jdbcTemplate;
    private final int batchSize;

    public TSWriterRepositoryImpl(@Value("${time.series.insert.batch_size}") final int batchSize, final JdbcTemplate jdbcTemplate) {
        this.batchSize = batchSize;
        this.jdbcTemplate = jdbcTemplate;
    }

    @Override
    public int[][] batchInsertMeasurements(final List<Measurement> measurements) {
        final int measurementsSize = measurements.size();
        log.info("Writing {} measurement(s).", measurementsSize);
        final int[][] result = jdbcTemplate.batchUpdate(
                INSERT_MEASUREMENTS_QUERY,
                measurements,
                batchSize, (preparedStatement, measurement) -> {
                    preparedStatement.setString(1, measurement.getChannelId());
                    preparedStatement.setTimestamp(2, Timestamp.valueOf(LocalDateTime.ofInstant(measurement.getTimestamp(), ZoneId.of("UTC"))));
                    preparedStatement.setBigDecimal(3, measurement.getMeasurement());
                });
        log.info("Successfully inserted {} measurement(s).", measurementsSize);
        return result;
    }
}
