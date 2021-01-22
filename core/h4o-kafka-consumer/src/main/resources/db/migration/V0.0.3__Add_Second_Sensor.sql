INSERT INTO metadata.things(id, name, description)
VALUES ('h4o/s2', 'Sensor 2', 'Sensor 2 (Prototyp ZB)');

INSERT INTO metadata.thing_nodes_to_things(thing_node_id, thing_id)
VALUES ('h4o', 'h4o/s2');

INSERT INTO metadata.channels(id, thing_id, name, description, unit, is_writable)
VALUES ('h4o/s2/co2', 'h4o/s2', 'H4O-S2: CO2', 'H4O Sensor 2: CO2', 'ppm', true),
       ('h4o/s2/temp', 'h4o/s2', 'H4O-S2: Temperature', 'H4O Sensor 2: Temperature', '°C', true),
       ('h4o/s2/press', 'h4o/s2', 'H4O-S2: Pressure', 'H4O Sensor 2: Pressure', 'hPa', true),
       ('h4o/s2/hum', 'h4o/s2', 'H4O-S2: Humidity', 'H4O Sensor 2: Humidity (rel)', '%', true),
       ('h4o/s2/bat', 'h4o/s2', 'H4O-S2: Battery', 'H4O Sensor 2: Battery Voltage', 'V', true);
