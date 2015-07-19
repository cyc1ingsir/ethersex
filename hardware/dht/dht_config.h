static const char sensor_name_keller[] PROGMEM = "keller";

dht_sensor_t dht_sensors[] = {
	{
		.port = &PORTD,
		.pin = PD6,
		.name = sensor_name_keller
	}
};

