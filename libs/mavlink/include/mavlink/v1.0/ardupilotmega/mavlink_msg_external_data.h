// MESSAGE EXTERNAL_DATA PACKING

#define MAVLINK_MSG_ID_EXTERNAL_DATA 227

MAVPACKED(
typedef struct __mavlink_external_data_t {
 float rpm1; /*< RPM Sensor1*/
 float rpm2; /*< RPM Sensor2*/
}) mavlink_external_data_t;

#define MAVLINK_MSG_ID_EXTERNAL_DATA_LEN 8
#define MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN 8
#define MAVLINK_MSG_ID_227_LEN 8
#define MAVLINK_MSG_ID_227_MIN_LEN 8

#define MAVLINK_MSG_ID_EXTERNAL_DATA_CRC 116
#define MAVLINK_MSG_ID_227_CRC 116



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_EXTERNAL_DATA { \
	227, \
	"EXTERNAL_DATA", \
	2, \
	{  { "rpm1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_external_data_t, rpm1) }, \
         { "rpm2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_external_data_t, rpm2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_EXTERNAL_DATA { \
	"EXTERNAL_DATA", \
	2, \
	{  { "rpm1", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_external_data_t, rpm1) }, \
         { "rpm2", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_external_data_t, rpm2) }, \
         } \
}
#endif

/**
 * @brief Pack a external_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rpm1 RPM Sensor1
 * @param rpm2 RPM Sensor2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_external_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float rpm1, float rpm2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXTERNAL_DATA_LEN];
	_mav_put_float(buf, 0, rpm1);
	_mav_put_float(buf, 4, rpm2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN);
#else
	mavlink_external_data_t packet;
	packet.rpm1 = rpm1;
	packet.rpm2 = rpm2;

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
 * @param rpm1 RPM Sensor1
 * @param rpm2 RPM Sensor2
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_external_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float rpm1,float rpm2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXTERNAL_DATA_LEN];
	_mav_put_float(buf, 0, rpm1);
	_mav_put_float(buf, 4, rpm2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN);
#else
	mavlink_external_data_t packet;
	packet.rpm1 = rpm1;
	packet.rpm2 = rpm2;

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
	return mavlink_msg_external_data_pack(system_id, component_id, msg, external_data->rpm1, external_data->rpm2);
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
	return mavlink_msg_external_data_pack_chan(system_id, component_id, chan, msg, external_data->rpm1, external_data->rpm2);
}

/**
 * @brief Send a external_data message
 * @param chan MAVLink channel to send the message
 *
 * @param rpm1 RPM Sensor1
 * @param rpm2 RPM Sensor2
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_external_data_send(mavlink_channel_t chan, float rpm1, float rpm2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_EXTERNAL_DATA_LEN];
	_mav_put_float(buf, 0, rpm1);
	_mav_put_float(buf, 4, rpm2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_DATA, buf, MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_CRC);
#else
	mavlink_external_data_t packet;
	packet.rpm1 = rpm1;
	packet.rpm2 = rpm2;

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
    mavlink_msg_external_data_send(chan, external_data->rpm1, external_data->rpm2);
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
static inline void mavlink_msg_external_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float rpm1, float rpm2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, rpm1);
	_mav_put_float(buf, 4, rpm2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_DATA, buf, MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_CRC);
#else
	mavlink_external_data_t *packet = (mavlink_external_data_t *)msgbuf;
	packet->rpm1 = rpm1;
	packet->rpm2 = rpm2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_EXTERNAL_DATA, (const char *)packet, MAVLINK_MSG_ID_EXTERNAL_DATA_MIN_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN, MAVLINK_MSG_ID_EXTERNAL_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE EXTERNAL_DATA UNPACKING


/**
 * @brief Get field rpm1 from external_data message
 *
 * @return RPM Sensor1
 */
static inline float mavlink_msg_external_data_get_rpm1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field rpm2 from external_data message
 *
 * @return RPM Sensor2
 */
static inline float mavlink_msg_external_data_get_rpm2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
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
	external_data->rpm1 = mavlink_msg_external_data_get_rpm1(msg);
	external_data->rpm2 = mavlink_msg_external_data_get_rpm2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_EXTERNAL_DATA_LEN? msg->len : MAVLINK_MSG_ID_EXTERNAL_DATA_LEN;
        memset(external_data, 0, MAVLINK_MSG_ID_EXTERNAL_DATA_LEN);
	memcpy(external_data, _MAV_PAYLOAD(msg), len);
#endif
}
