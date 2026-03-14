#include "bsp_fdcan.h"
#include <stdint.h>

#define DATA_LENGTH 8 /* Classical CAN */

/* Physical CAN peripheral: fdcan_instance */
struct fdcan_instance {
	FDCAN_HandleTypeDef *hfdcan;
	uint8_t rx_buff[DATA_LENGTH];
};

struct can_filter_config {
	uint32_t rx_id;
	uint32_t rx_mask;
	enum can_id_type id_type;
	FDCAN_HandleTypeDef *hfdcan;
};

int can_debug = 0;

#define NUM_FDCAN_INSTANCE (sizeof(fdcan_inst) / sizeof(struct fdcan_instance))

/*
 * Maximum number of devices for a single CAN peripheral.
 * This is a experimental limit (125hz each device).
 */
#define FDCAN_MAX_DEVICE_PER_INST 12

/* Hardware filter limits (as configured in STM32CubeMX) */
#define MAX_STD_FILTER_CNT 16
#define MAX_EXT_FILTER_CNT 8

static uint8_t filter_configured = 0u;

static struct fdcan_instance fdcan_inst[] = {
    {.hfdcan = &hfdcan1, .rx_buff = {0}},
    {.hfdcan = &hfdcan2, .rx_buff = {0}},
    {.hfdcan = &hfdcan3, .rx_buff = {0}},
};

static struct can_filter_config fdcan1_config[] = {
    {.rx_id = 0x201, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan1},
    {.rx_id = 0x202, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan1},
    {.rx_id = 0x203, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan1},
    {.rx_id = 0x204, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan1},
    {.rx_id = 0x205, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan1},
    {.rx_id = 0x206, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan1},
    {.rx_id = 0x207, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan1},
    {.rx_id = 0x208, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan1},
    {.rx_id = 0x301, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan1},
    {.rx_id = 0x00B, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan1},
};

static struct can_filter_config fdcan2_config[] = {
    {.rx_id = 0x201, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan2},
    {.rx_id = 0x202, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan2},
    {.rx_id = 0x203, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan2},
    {.rx_id = 0x204, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan2},
    {.rx_id = 0x205, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan2},
    {.rx_id = 0x206, .rx_mask = 0x7FF, .id_type = CAN_ID_STD, .hfdcan = &hfdcan2},
};

static struct can_filter_config fdcan3_config[] = {
    {.rx_id = 0x201, .rx_mask = 0x0FF, .id_type = CAN_ID_EXT, .hfdcan = &hfdcan3},
};

static int16_t get_instance_index(FDCAN_HandleTypeDef *hfdcan)
{
	for (int i = 0; i < NUM_FDCAN_INSTANCE; i++) {
		if (hfdcan == fdcan_inst[i].hfdcan)
			return i;
	}
	return -1;
}

static uint8_t *get_rx_buff(FDCAN_HandleTypeDef *hfdcan)
{
	int16_t idx = get_instance_index(hfdcan);

	if (idx != -1)
		return (uint8_t *)fdcan_inst[idx].rx_buff;
	else
		return NULL;
}

/**
 * add_filter() - Helper function for can_config_filter()
 * @conf: pointer to the filter configuration
 *
 * Configure a filter for a FDCAN instance.
 * All mesages matching are reported to RxFIFO0.
 *
 * Return: HAL_OK if successful and HAL_ERROR otherwise.
 */
static HAL_StatusTypeDef add_filter(struct can_filter_config const *conf)
{
	/* TODO: Log Messages & Error Case */

	FDCAN_FilterTypeDef filter;
	static uint8_t filter_idx_std[NUM_FDCAN_INSTANCE] = {0};
	static uint8_t filter_idx_ext[NUM_FDCAN_INSTANCE] = {0};

	if (conf == NULL)
		return HAL_ERROR;

	int16_t idx = get_instance_index(conf->hfdcan);
	if (idx == -1)
		return HAL_ERROR;

	/* STM32H7 FDCAN hardware filter index handling
	 *
	 * Standard and extended ID filters are stored in separate RAM areas.
	 * This means a standard filter and an extended filter can use
	 * the same FilterIndex value without conflict.
	 */
	if (conf->id_type == CAN_ID_STD) {
		if (filter_idx_std[idx] >= MAX_STD_FILTER_CNT)
			return HAL_ERROR;
		filter.IdType = FDCAN_STANDARD_ID;
		filter.FilterIndex = filter_idx_std[idx]++;
	} else {
		if (filter_idx_ext[idx] >= MAX_EXT_FILTER_CNT)
			return HAL_ERROR;
		filter.IdType = FDCAN_EXTENDED_ID;
		filter.FilterIndex = filter_idx_ext[idx]++;
	}

	/*
	 * RxBufferIndex and IsCalibrationMsg are omitted, that is because
	 * both parameters will be ignored if FilterConfig is different from
	 * FDCAN_FILTER_TO_BUFFER.
	 */
	filter.FilterType = FDCAN_FILTER_MASK; /* Matching: FilterID1 & FilterID2 */
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; /* Always reports to RxFIFO0 */
	filter.FilterID1 = conf->rx_id;
	filter.FilterID2 = conf->rx_mask;

	if (HAL_FDCAN_ConfigFilter(conf->hfdcan, &filter) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}

/**
 * can_config_filter() - Configure FDCAN filters
 * @conf: array of filter configuration
 * @num: total number of filters ( length of array )
 *
 * Configure all filters for every FDCAN instances.
 * All mesages matching are reported to RxFIFO0.
 *
 * NOTE: Filter configuration continues even if individual filters fail.
 * This ensures maximum device availability - errors are reported via return
 * value but do not stop the process.
 *
 * Context: Should be called only once and before can_init().
 * Calling can_config_filter() multiple times or after can_init() may render
 * bugs.
 *
 * Return: HAL_OK if all filters configured successfully,
 * HAL_ERROR if any filter fails or device limit exceeded.
 */
static HAL_StatusTypeDef can_config_filter(struct can_filter_config const *conf,
					   uint16_t num)
{
	/* TODO: Log Messages & Error Case */
	/*
	 * HAL_FDCAN_Start() in can_init() will set hfdcan->State to be
	 * HAL_FDCAN_STATE_BUSY. HAL_FDCAN_ConfigFilter() in add_filter()
	 * accepts both HAL_FDCAN_STATE_READY and HAL_FDCAN_STATE_BUSY states,
	 * but HAL_FDCAN_ConfigFilter() is better to be prior to
	 * HAL_FDCAN_Start().
	 *
	 * Although not recommended, calling this function multiple times or
	 * after can_init() is currently allowed and functional, so no checks
	 * are performed here.
	 */
	filter_configured = 1; /* set labels for can_init() */

	if (conf == NULL || num == 0)
		return HAL_ERROR;

	HAL_StatusTypeDef result = HAL_OK;

	for (int i = 0; i < num; i++) {
		if (add_filter(&conf[i]) != HAL_OK)
			result = HAL_ERROR;
	}

	return result;
}

/**
 * can_init() - Initialize all FDCAN instances and start reception.
 *
 * This function configures the global filter, activates the receive FIFO0
 * interrupt, and starts all FDCAN peripherals.
 * The global filter is set to reject all remote frames.
 * For data frames, unmatched messages are rejected if can_config_filter() has
 * been called, otherwise it accepts all data frame messages.
 *
 * If HAL_FDCAN_Start() fails, Error_Handler() is invoked ( while(1) ).
 * Failure on other HAL function will result in HAL_ERROR return,
 * but will not invoke Error_Handler().
 *
 * Context: Intended to be called once during system initialization,
 * after the calling of can_config_filter() and before CAN communications.
 *
 * Return: HAL_OK on success, HAL_ERROR if any non-start HAL function fails.
 * Does NOT return if HAL_FDCAN_Start() fails (calls Error_Handler).
 */
HAL_StatusTypeDef can_init(void)
{
	/* The recommended initialization order is:
	 * 1. Configure individual filters (HAL_FDCAN_ConfigFilter)
	 * 2. Configure the global filter (HAL_FDCAN_ConfigGlobalFilter)
	 * 3. Activate reception interrupt (HAL_FDCAN_ActivateNotification)
	 * 4. Start the FDCAN peripheral (HAL_FDCAN_Start)
	 *
	 * Reasons:
	 * HAL_FDCAN_ConfigGlobalFilter requires hfdcan->State to be
	 * HAL_FDCAN_STATE_READY, while HAL_FDCAN_Start sets state to
	 * HAL_FDCAN_STATE_BUSY. HAL_FDCAN_ConfigFilter and
	 * HAL_FDCAN_ActivateNotification accept both states.
	 */

	/* Configure filters */
	can_config_filter(fdcan1_config, 10);
	can_config_filter(fdcan2_config, 6);
	can_config_filter(fdcan3_config, 1);

	uint8_t result = HAL_OK;
	FDCAN_HandleTypeDef *hfdcan;
	/* Decide acception strategy based on whether filters are configured. */
	uint32_t unmatch = filter_configured ? FDCAN_REJECT : FDCAN_ACCEPT_IN_RX_FIFO0;

	for (int i = 0; i < NUM_FDCAN_INSTANCE; i++) {
		hfdcan = fdcan_inst[i].hfdcan;

		if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, unmatch, unmatch,
						 FDCAN_REJECT_REMOTE,
						 FDCAN_REJECT_REMOTE) != HAL_OK)
			result = HAL_ERROR;
		if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
						   0) != HAL_OK)
			result = HAL_ERROR;
		if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
			result = HAL_ERROR;
			Error_Handler(); /* while(1) */
		}
	}

	return result;
}

/**
 * can_transmit() - Transmit a CAN message
 * @hfdcan: Pointer to FDCAN handle
 * @id: CAN identifier (standard or extended)
 * @type: Type of identifier (CAN_ID_STD or CAN_ID_EXT)
 * @buff: Pointer to 8-byte data buffer to transmit
 *
 * Transmits a classical CAN message using the specified FDCAN peripheral.
 * The data buffer must contain exactly 8 bytes.
 *
 * Return: HAL_OK on success, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef can_transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
			       enum can_id_type type, uint8_t *buff)
{
	/* TODO: Log Messages & Error Cases */
	/* NOTE: This function will be absorted in future OOP design */
	if (hfdcan == NULL || buff == NULL)
		return HAL_ERROR;

	if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
		return HAL_BUSY; /* whether FIFO is full */

	FDCAN_TxHeaderTypeDef tx_header = {
	    .Identifier = id,
	    .IdType = (type == CAN_ID_STD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID,
	    .TxFrameType = FDCAN_DATA_FRAME, /* Only use data frame */
	    .DataLength = FDCAN_DLC_BYTES_8, /* Classical CAN */
	    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
	    .BitRateSwitch = FDCAN_BRS_OFF,
	    .FDFormat = FDCAN_CLASSIC_CAN,
	    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
	    .MessageMarker = 0,
	};

	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, buff);
}

__weak void fdcan1_data_interpret(FDCAN_RxHeaderTypeDef *header, uint8_t *buff);
__weak void fdcan2_data_interpret(FDCAN_RxHeaderTypeDef *header, uint8_t *buff);
__weak void fdcan3_data_interpret(FDCAN_RxHeaderTypeDef *header, uint8_t *buff);

/**
 * HAL_FDCAN_RxFifo0Callback() - FDCAN Rx FIFO 0 callback (weak function
 * override)
 * @hfdcan: Pointer to FDCAN handle
 * @RxFifo0ITs: Rx FIFO 0 interrupt status
 *
 * This callback is invoked by the HAL when a new message arrives in RxFIFO0.
 * It retrieves the received message using HAL_FDCAN_GetRxMessage and forwards
 * it to the user-registered callback for processing.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	can_debug++;

	/* TODO: Log Message & Error Cases */
	if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
		FDCAN_RxHeaderTypeDef rx_header;
		uint8_t *buff = get_rx_buff(hfdcan);
		if (buff == NULL)
			return;

		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, buff) !=
		    HAL_OK)
			return;

		if (hfdcan == &hfdcan1) {
			fdcan1_data_interpret(&rx_header, buff);
		} else if (hfdcan == &hfdcan2) {
			fdcan2_data_interpret(&rx_header, buff);
		} else if (hfdcan == &hfdcan3) {
			fdcan3_data_interpret(&rx_header, buff);
		}

		
	}
}
