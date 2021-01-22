CREATE SCHEMA if not exists metadata;
CREATE SCHEMA if not exists timeseries;

DROP TABLE IF EXISTS timeseries.messages cascade;
DROP TABLE IF EXISTS timeseries.measurements cascade;
DROP TABLE IF EXISTS timeseries.substitutions cascade;
DROP TABLE IF EXISTS metadata.channels cascade;
DROP TABLE IF EXISTS metadata.thing_nodes_to_things cascade;
DROP TABLE IF EXISTS metadata.things cascade;
DROP TABLE IF EXISTS metadata.thing_nodes cascade;

CREATE TABLE metadata.thing_nodes
(
    id          varchar(127) primary key,
    parent_id   varchar(127) REFERENCES metadata.thing_nodes (id),
    name        varchar(127) not null,
    description varchar(255)
);

CREATE TABLE metadata.things
(
    id          varchar(127) primary key,
    name        varchar(127) not null,
    description varchar(255)
);

CREATE TABLE metadata.thing_nodes_to_things
(
    thing_node_id varchar(127) not null REFERENCES metadata.thing_nodes (id),
    thing_id      varchar(127) not null REFERENCES metadata.things (id),
    primary key (thing_node_id, thing_id)
);

CREATE TABLE metadata.channels
(
    id          varchar(127) primary key,
    thing_id    varchar(127) REFERENCES metadata.things (id),
    name        varchar(127) not null,
    description varchar(255),
    unit        varchar(10),
    is_writable bool
);

CREATE TABLE timeseries.messages
(
    channel_id varchar(127) not null references metadata.channels (id),
    timestamp  timestamp,
    message    varchar(255) not null,
    PRIMARY KEY (timestamp, channel_id)
);

CREATE TABLE timeseries.measurements
(
    channel_id  varchar(127)   not null references metadata.Channels (id),
    timestamp   timestamp,
    measurement numeric(12, 6) not null,
    PRIMARY KEY (timestamp, channel_id)
);

CREATE TABLE timeseries.substitutions
(
    channel_id  varchar(127)   not null references metadata.Channels (id),
    timestamp   timestamp,
    measurement numeric(12, 6) not null,
    PRIMARY KEY (timestamp, channel_id)
);

-- Insert devices structure

INSERT INTO metadata.thing_nodes (id, parent_id, name, description)
VALUES ('h4o', null, 'H4O Devices', 'hetida4office Global Device Hierarchy');

INSERT INTO metadata.things(id, name, description)
VALUES ('h4o/s1', 'Sensor 1', 'Sensor 1 (Prototyp FG)');

INSERT INTO metadata.thing_nodes_to_things(thing_node_id, thing_id)
VALUES ('h4o', 'h4o/s1');

INSERT INTO metadata.channels(id, thing_id, name, description, unit, is_writable)
VALUES ('h4o/s1/co2', 'h4o/s1', 'H4O-S1: CO2', 'H4O Sensor 1: CO2', 'ppm', true);
INSERT INTO metadata.channels(id, thing_id, name, description, unit, is_writable)
VALUES ('h4o/s1/temp', 'h4o/s1', 'H4O-S1: Temperature', 'H4O Sensor 1: Temperature', '°C', true);
INSERT INTO metadata.channels(id, thing_id, name, description, unit, is_writable)
VALUES ('h4o/s1/press', 'h4o/s1', 'H4O-S1: Pressure', 'H4O Sensor 1: Pressure', 'hPa', true);
INSERT INTO metadata.channels(id, thing_id, name, description, unit, is_writable)
VALUES ('h4o/s1/hum', 'h4o/s1', 'H4O-S1: Humidity', 'H4O Sensor 1: Humidity (rel)', '%', true);