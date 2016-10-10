// MESSAGE EXTERNAL_DATA PACKING

#define MAVLINK_MSG_ID_EXTERNAL_DATA 227

MAVPACKED(
typedef struct __mavlink_external_data_t {
 float voltage; /*< Voltage of currently used battery in volts.*/
 float current; /*< Current leaving currently used battery in amps.*/
 float air_temperature; /*< Air temperature in celsius.*/
 float water_temperature; /*< Water temperature in celsius.*/
 float humidity; /*< Percentage of humidity in air.*/
 uint32_t battery_status; /*< Which battery is used. (1 or 2)*/
 float latitude; /*< Latitude of the microprocessor when the data was received.*/
 float longitude; /*< Longitude of the microprocessor when the data was received.*/
 float altitude; /*< Altitude of the microprocessor when the data was received.*/
}) mavlink_external_data_t;

#define MAVLINK_MSG_ID_EXTERNAL_DATA_LEN 36
#define MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN 36
#define MAVLINK_MSG_ID_227_LEN 36
#define MAVLINK_MSG_ID_227_MIN_LEN 36

#define MAVLINK_MSG_ID_EXTERNAL_DATA_CRC 68
#define MAVLINK_MSG_ID_227_CRC 68



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_EXTERNAL_DATA { \
	227, \
	"EXTERNAL_DATA", \
	9, \
	{  { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_external_data_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_external_data_t, current) }, \
         { "air_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_external_data_t, air_temperature) }, \
         { "water_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_external_data_t, water_temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_external_data_t, humidity) }, \
         { "battery_status", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_external_data_t, battery_status) }, \
         { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_external_data_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_external_data_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_external_data_t, altitude) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_EXTERNAL_DATA { \
	"EXTERNAL_DATA", \
	9, \
	{  { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_external_data_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_external_data_t, current) }, \
         { "air_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_external_data_t, air_temperature) }, \
         { "water_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_external_data_t, water_temperature) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_external_data_t, humidity) }, \
         { "battery_status", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_external_data_t, battery_status) }, \
         { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_external_data_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_external_data_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_external_data_t, altitude) }, \
         } \
}
#endif

/**
 * @brief Pack a external_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param voltage Voltage of currently used battery in volts.
 * @param current Current leaving currently used battery in amps.
 * @param air_temperature Air temperature in celsius.
 * @param water_temperature Water temperature in celsius.
 * @param humidity Percentage of humidity in air.
 * @param battery_status Which battery is used. (1 or 2)
 * @param latitude Latitude of the microprocessor when the data was received.
 * @param longitude Longitude of the microprocessor when the data was received.
 * @param altitude Altitude of the microprocessor when the data was received.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_external_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float voltage, float current, float air_temperature, float water_temperature, float humidity, uint32_t battery_status, float latitude, float longitude, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXTERNAL_DATA_LEN];
	_mav_put_float(buf, 0, voltage);
	_mav_put_float(buf, 4, current);
	_mav_put_float(buf, 8, air_temperature);
	_mav_put_float(buf, 12, water_temperature);
	_mav_put_float(buf, 16, humidity);
	_mav_put_uint32_t(buf, 20, battery_status);
	_mav_put_float(buf, 24, latitude);
	_mav_put_float(buf, 28, longitude);
	_mav_put_float(buf, 32, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN);
#else
	mavlink_external_data_t packet;
	packet.voltage = voltage;
	packet.current = current;
	packet.air_temperature = air_temperature;
	packet.water_temperature = water_temperature;
	packet.humidity = humidity;
	packet.battery_status = battery_status;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EXTERNAL_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_CRC);
}

/**
 * @brief Pack a external_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param voltage Voltage of currently used battery in volts.
 * @param current Current leaving currently used battery in amps.
 * @param air_temperature Air temperature in celsius.
 * @param water_temperature Water temperature in celsius.
 * @param humidity Percentage of humidity in air.
 * @param battery_status Which battery is used. (1 or 2)
 * @param latitude Latitude of the microprocessor when the data was received.
 * @param longitude Longitude of the microprocessor when the data was received.
 * @param altitude Altitude of the microprocessor when the data was received.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_external_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float voltage,float current,float air_temperature,float water_temperature,float humidity,uint32_t battery_status,float latitude,float longitude,float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXTERNAL_DATA_LEN];
	_mav_put_float(buf, 0, voltage);
	_mav_put_float(buf, 4, current);
	_mav_put_float(buf, 8, air_temperature);
	_mav_put_float(buf, 12, water_temperature);
	_mav_put_float(buf, 16, humidity);
	_mav_put_uint32_t(buf, 20, battery_status);
	_mav_put_float(buf, 24, latitude);
	_mav_put_float(buf, 28, longitude);
	_mav_put_float(buf, 32, altitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN);
#else
	mavlink_external_data_t packet;
	packet.voltage = voltage;
	packet.current = current;
	packet.air_temperature = air_temperature;
	packet.water_temperature = water_temperature;
	packet.humidity = humidity;
	packet.battery_status = battery_status;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_EXTERNAL_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_CRC);
}

/**
 * @brief Encode a external_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param external_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_external_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_external_data_t* external_data)
{
	return mavlink_msg_external_data_pack(system_id, component_id, msg, external_data->voltage, external_data->current, external_data->air_temperature, external_data->water_temperature, external_data->humidity, external_data->battery_status, external_data->latitude, external_data->longitude, external_data->altitude);
}

/**
 * @brief Encode a external_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param external_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_external_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_external_data_t* external_data)
{
	return mavlink_msg_external_data_pack_chan(system_id, component_id, chan, msg, external_data->voltage, external_data->current, external_data->air_temperature, external_data->water_temperature, external_data->humidity, external_data->battery_status, external_data->latitude, external_data->longitude, external_data->altitude);
}

/**
 * @brief Send a external_data message
 * @param chan MAVLink channel to send the message
 *
 * @param voltage Voltage of currently used battery in volts.
 * @param current Current leaving currently used battery in amps.
 * @param air_temperature Air temperature in celsius.
 * @param water_temperature Water temperature in celsius.
 * @param humidity Percentage of humidity in air.
 * @param battery_status Which battery is used. (1 or 2)
 * @param latitude Latitude of the microprocessor when the data was received.
 * @param longitude Longitude of the microprocessor when the data was received.
 * @param altitude Altitude of the microprocessor when the data was received.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_external_data_send(mavlink_channel_t chan, float voltage, float current, float air_temperature, float water_temperature, float humidity, uint32_t battery_status, float latitude, float longitude, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXTERNAL_DATA_LEN];
	_mav_put_float(buf, 0, voltage);
	_mav_put_float(buf, 4, current);
	_mav_put_float(buf, 8, air_temperature);
	_mav_put_float(buf, 12, water_temperature);
	_mav_put_float(buf, 16, humidity);
	_mav_put_uint32_t(buf, 20, battery_status);
	_mav_put_float(buf, 24, latitude);
	_mav_put_float(buf, 28, longitude);
	_mav_put_float(buf, 32, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_DATA, buf, MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_CRC);
#else
	mavlink_external_data_t packet;
	packet.voltage = voltage;
	packet.current = current;
	packet.air_temperature = air_temperature;
	packet.water_temperature = water_temperature;
	packet.humidity = humidity;
	packet.battery_status = battery_status;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_DATA, (const char *)&packet, MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_CRC);
#endif
}

/**
 * @brief Send a external_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_external_data_send_struct(mavlink_channel_t chan, const mavlink_external_data_t* external_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_external_data_send(chan, external_data->voltage, external_data->current, external_data->air_temperature, external_data->water_temperature, external_data->humidity, external_data->battery_status, external_data->latitude, external_data->longitude, external_data->altitude);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_DATA, (const char *)external_data, MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_EXTERNAL_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_external_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float voltage, float current, float air_temperature, float water_temperature, float humidity, uint32_t battery_status, float latitude, float longitude, float altitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, voltage);
	_mav_put_float(buf, 4, current);
	_mav_put_float(buf, 8, air_temperature);
	_mav_put_float(buf, 12, water_temperature);
	_mav_put_float(buf, 16, humidity);
	_mav_put_uint32_t(buf, 20, battery_status);
	_mav_put_float(buf, 24, latitude);
	_mav_put_float(buf, 28, longitude);
	_mav_put_float(buf, 32, altitude);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_DATA, buf, MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_CRC);
#else
	mavlink_external_data_t *packet = (mavlink_external_data_t *)msgbuf;
	packet->voltage = voltage;
	packet->current = current;
	packet->air_temperature = air_temperature;
	packet->water_temperature = water_temperature;
	packet->humidity = humidity;
	packet->battery_status = battery_status;
	packet->latitude = latitude;
	packet->longitude = longitude;
	packet->altitude = altitude;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_DATA, (const char *)packet, MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE EXTERNAL_DATA UNPACKING


/**
 * @brief Get field voltage from external_data message
 *
 * @return Voltage of currently used battery in volts.
 */
static inline float mavlink_msg_external_data_get_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field current from external_data message
 *
 * @return Current leaving currently used battery in amps.
 */
static inline float mavlink_msg_external_data_get_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field air_temperature from external_data message
 *
 * @return Air temperature in celsius.
 */
static inline float mavlink_msg_external_data_get_air_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field water_temperature from external_data message
 *
 * @return Water temperature in celsius.
 */
static inline float mavlink_msg_external_data_get_water_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field humidity from external_data message
 *
 * @return Percentage of humidity in air.
 */
static inline float mavlink_msg_external_data_get_humidity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field battery_status from external_data message
 *
 * @return Which battery is used. (1 or 2)
 */
static inline uint32_t mavlink_msg_external_data_get_battery_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field latitude from external_data message
 *
 * @return Latitude of the microprocessor when the data was received.
 */
static inline float mavlink_msg_external_data_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field longitude from external_data message
 *
 * @return Longitude of the microprocessor when the data was received.
 */
static inline float mavlink_msg_external_data_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field altitude from external_data message
 *
 * @return Altitude of the microprocessor when the data was received.
 */
static inline float mavlink_msg_external_data_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Decode a external_data message into a struct
 *
 * @param msg The message to decode
 * @param external_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_external_data_decode(const mavlink_message_t* msg, mavlink_external_data_t* external_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	external_data->voltage = mavlink_msg_external_data_get_voltage(msg);
	external_data->current = mavlink_msg_external_data_get_current(msg);
	external_data->air_temperature = mavlink_msg_external_data_get_air_temperature(msg);
	external_data->water_temperature = mavlink_msg_external_data_get_water_temperature(msg);
	external_data->humidity = mavlink_msg_external_data_get_humidity(msg);
	external_data->battery_status = mavlink_msg_external_data_get_battery_status(msg);
	external_data->latitude = mavlink_msg_external_data_get_latitude(msg);
	external_data->longitude = mavlink_msg_external_data_get_longitude(msg);
	external_data->altitude = mavlink_msg_external_data_get_altitude(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_EXTERNAL_DATA_LEN? msg->len : MAVLINK_MSG_ID_EXTERNAL_DATA_LEN;
        memset(external_data, 0, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN);
	memcpy(external_data, _MAV_PAYLOAD(msg), len);
#endif
}
