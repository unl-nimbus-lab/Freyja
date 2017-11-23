/*
aj: custom packet for velocity control and logging
*/

/* packet type == 5 */

#define PRY_HEADER_VAL (0x7B)
#define DESCEND_HEADER_VAL (0x7C)
#define MOTORS_IDLE_HEADER_VAL (0x7D)
#define MOTORS_OFF_HEADER_VAL (0x7E)
#define MOTORS_ON_HEADER_VAL (0x7F)
#define AJ_VELOCITY_HEADER_VAL (0xA7)
enum {
  PKT_HEADER_IDX = 0,
  PKT_LAT_3_IDX,
	PKT_LAT_2_IDX,
	PKT_LAT_1_IDX,
	PKT_LAT_0_IDX,
	PKT_LON_3_IDX,
	PKT_LON_2_IDX,
	PKT_LON_1_IDX,
	PKT_LON_0_IDX,
	PKT_FUSION_HEIGHT_3_IDX,
	PKT_FUSION_HEIGHT_2_IDX,
	PKT_FUSION_HEIGHT_1_IDX,
	PKT_FUSION_HEIGHT_0_IDX,
  PKT_SPEED_X_3_IDX,
	PKT_SPEED_X_2_IDX,
	PKT_SPEED_X_1_IDX,
	PKT_SPEED_X_0_IDX,
	PKT_SPEED_Y_3_IDX,
	PKT_SPEED_Y_2_IDX,
	PKT_SPEED_Y_1_IDX,
	PKT_SPEED_Y_0_IDX,
	PKT_BEST_SPEED_X_1_IDX,
	PKT_BEST_SPEED_X_0_IDX,
	PKT_BEST_SPEED_Y_1_IDX,
	PKT_BEST_SPEED_Y_0_IDX,
	PKT_HEADING_1_IDX,
	PKT_HEADING_0_IDX,
	PKT_CHKSUM_IDX_1,
	PKT_CHKSUM_IDX_0,
	PKT_LEN
};

enum
{
  PRY_HEADER_IDX = 0,
  PRY_PITCH_HIGH,
  PRY_PITCH_LOW,
  PRY_ROLL_HIGH,
  PRY_ROLL_LOW,
  PRY_YAW_HIGH,
  PRY_YAW_LOW,
  PRY_THRUST_HIGH,
  PRY_THRUST_LOW,
  PRY_CTRL,
  PRY_CRC_HIGH,
  PRY_CRC_LOW,
  PRY_PKT_LEN
};

/* Descend packet command structure */
enum
{
	DESCEND_HEADER_IDX = 0,
	DESCEND_CRC_HIGH,
	DESCEND_CRC_LOW,
	DESCEND_CMND_LEN
};

/* Idle motor command structure */
enum
{
	MOTORS_IDLE_HEADER_IDX = 0,
	MOTORS_IDLE_CRC_HIGH,
	MOTORS_IDLE_CRC_LOW,
	MOTORS_IDLE_CMND_LEN
};

/* Motor off command structure */
enum
{
	MOTORS_OFF_HEADER_IDX = 0,
	MOTORS_OFF_CRC_HIGH,
	MOTORS_OFF_CRC_LOW,
	MOTORS_OFF_CMND_LEN
};

/* Motor on command structure */
enum
{
	MOTORS_ON_HEADER_IDX = 0,
	MOTORS_ON_CRC_HIGH,
	MOTORS_ON_CRC_LOW,
	MOTORS_ON_CMND_LEN
};

const int16_t PITCH_MAX = 2047;
const int16_t PITCH_MIN = -2047;
const int16_t ROLL_MAX = 2047;
const int16_t ROLL_MIN = -2047;
const int16_t YAW_MAX = 2047;
const int16_t YAW_MIN = -2047;
const int16_t THRUST_MAX = 4097;
const int16_t THRUST_MIN = 0;
