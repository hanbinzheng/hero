#include "bsp_fdcan.h"

/* Classical can has a maximum length of 8, here choose 8 as default */
#define DATA_LENGTH 8

struct fdcan_instance {
	FDCAN_HandleTypeDef *hfdcan;
	uint8_t rx_buff[DATA_LENGTH];
};

/* TODO enable expansibility, now hard-coded */
static const struct fdcan_instance fdcan_inst[] = {
	{.hfdcan = &hfdcan1, .rx_buff = {0}},
	{.hfdcan = &hfdcan2, .rx_buff = {0}},
	{.hfdcan = &hfdcan3, .rx_buff = {0}},
};
/* total number of fdcan instance available on STM32 H723 */
#define NUM_FDCAN_INSTANCE (sizeof(fdcan_inst) / sizeof(struct fdcan_instance))

/*
 * Maximum number of devices on a single CAN bus ( CAN instance ).
 * This is a experimental limit (125hz each device).
 */
#define FDCAN_MAX_DEVICE_PER_INST 16

/* Hardware filter limits (as configured in STM32CubeMX) */
#define MAX_STD_FILTER_CNT 16
#define MAX_EXT_FILTER_CNT 8

static can_rx_callback_t rx_callback = NULL;

/* Check whether can_config_filter() has been called */
static uint8_t filter_configured = 0u;

/*
 * Helper function to determine the can instance.
 * Return -1 if it fails.
 */
static int16_t get_instance_index(FDCAN_HandleTypeDef *hfdcan)
{
	for (int i = 0; i < NUM_FDCAN_INSTANCE; i++) {
		if (hfdcan == fdcan_inst[i].hfdcan)
			return i;
	}
	return -1;
}

/**
 * add_filter() - The helper function for can_config_filter()
 * @conf: pointer to a filter configuration
 *
 * Configure one filter for a FDCAN instance.
 * All mesages matching are reported to RxFIFO0.
 *
 * Return: HAL_OK if successful and HAL_ERROR if any failure.
 */
static HAL_StatusTypeDef add_filter(struct can_filter_config const *conf)
{
	/* TODO: Add log messages for all failure and error case. */

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
		/* check the number of devices */
		if (filter_idx_std[idx] >= MAX_STD_FILTER_CNT)
			return HAL_ERROR;

		filter.IdType = FDCAN_STANDARD_ID;
		filter.FilterIndex = filter_idx_std[idx]++; /* starts from zero */

	} else {
		if (filter_idx_ext[idx] >= MAX_EXT_FILTER_CNT)
			return HAL_ERROR;

		filter.IdType = FDCAN_EXTENDED_ID;
		filter.FilterIndex = filter_idx_ext[idx]++;
	}

	/*
	 * Our configuration omits RxBufferIndex and IsCalibrationMsg
	 * Both parameters will be ignored if FilterConfig is different from
	 * FDCAN_FILTER_TO_BUFFER.
	 */
	filter.FilterType = FDCAN_FILTER_MASK;		   /* Matching: FilterID1 & FilterID2 */
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; /* Always reports to RxFIFO0 */
	filter.FilterID1 = conf->rx_id;
	filter.FilterID2 = conf->rx_mask;

	/* Configure the fdcan filter.
	 * HAL_FDCAN_ConfigFilter only returns HAL_OK or HAL_ERROR.
	 */
	if (HAL_FDCAN_ConfigFilter(conf->hfdcan, &filter) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}

/**
 * can_config_filter() - Configure fdcan filters
 * @conf: array of filter configuration
 * @num: total number of filters ( length of array )
 *
 * Configure all filters for every FDCAN instances.
 * All mesages matching are reported to RxFIFO0.
 *
 * NOTE: The configuration will continue to finish even failures are met.
 * This is to keep devices as much as possible.
 *
 * Context: Should be called only once and must be called before can_init()
 * Calling can_config_filter() multiple times or after can_init() may render
 * implicit bugs, thus this function is recommended to be called only once
 * before can_init().
 *
 * Return: HAL_ERROR if any configuration fails,
 * or the number of devices exceeds the upper limit.
 */
HAL_StatusTypeDef can_config_filter(struct can_filter_config const *conf, uint16_t num)
{
	/*
	 * HAL_FDCAN_Start() in can_init() will set hfdcan->State to be HAL_FDCAN_STATE_BUSY.
	 * HAL_FDCAN_ConfigFilter() in add_filter() accepts both
	 * HAL_FDCAN_STATE_READY and HAL_FDCAN_STATE_BUSY states,
	 * but HAL_FDCAN_ConfigFilter() is better to be prior to HAL_FDCAN_Start().
	 *
	 * Although calling can_config_filter() multiple times or after can_init() is not
	 * recommended, it is still acceptable, thus here no o inspection was conducted.
	 */
	filter_configured = 1; /* set labels for can_init() */

	if (conf == NULL || num == 0) {
		/* TODO: Add error log message. */
		return HAL_ERROR;
	}

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
 * interrupt, and starts each FDCAN peripherals.
 * The global filter is set to reject all non‑matching messages and remote
 * frames if can_config_filter() has been called, otherwise it will accept all messages.
 *
 * If HAL_FDCAN_Start() fails, Error_Handler() is invoked ( while(1) ),
 * failure on other HAL function will result in HAL_ERROR return,
 * but will not invoke Error_Handler().
 *
 * It must be called after individual filters have been configured
 * (by can_config_filter()) and before any CAN communication takes place. The
 * underlying FDCAN handles are assumed to have been properly initialized.
 *
 * The global filter is set to reject all non‑matching messages and remote
 * frames if can_config_filter() has been called, otherwise it will accept all messages.
 *
 * Context: Intended to be called once during system initialization,
 * after the calling of can_config_filter().
 *
 * Return: HAL_OK if successful, HAL_ERROR if HAL_FDCAN_Start() succeeds but
 * another HAL function fails, and it will not return if HAL_FDCAN_Start() fails
 */
HAL_StatusTypeDef can_init(void)
{
	/* The recommended initialization order, which respects the HAL state, is:
	 *
	 * 1. Configure individual filters via can_config_filter()
	 * (HAL_FDCAN_ConfigFilter)
	 * 2. Configure the global filter (HAL_FDCAN_ConfigGlobalFilter)
	 * 3. Activate reception interrupt (HAL_FDCAN_ActivateNotification)
	 * 4. Start the FDCAN peripheral (HAL_FDCAN_Start)
	 *
	 * Reasons:
	 * HAL_FDCAN_ConfigGlobalFilter requires hfdcan->State = HAL_FDCAN_STATE_READY,
	 * while HAL_FDCAN_Start sets state to HAL_FDCAN_STATE_BUSY.
	 * HAL_FDCAN_ConfigFilter and HAL_FDCAN_ActivateNotification accept both states.
	 */
	uint8_t result = HAL_OK;

	for (int i = 0; i < NUM_FDCAN_INSTANCE; i++) {
		/*
		 * Check whether can_config_filter() has been called.
		 * If filters has been configured, reject all unmatched information
		 */
		uint32_t unmatch = filter_configured ? FDCAN_REJECT : FDCAN_ACCEPT_IN_RX_FIFO0;
		FDCAN_HandleTypeDef *hfdcan = fdcan_inst[i].hfdcan;

		/* Configure global filters, and reject all remote frames */
		if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, unmatch, unmatch, FDCAN_REJECT_REMOTE,
										 FDCAN_REJECT_REMOTE) != HAL_OK)
			result = HAL_ERROR;
		/* Activate RxFIFO0 interrupt */
		if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) !=
			HAL_OK)
			result = HAL_ERROR;
		/* Start the FDCAN periphery */
		if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
			result = HAL_ERROR;
			Error_Handler(); /* while(1) */
		}
	}

	return result;
}

/**
 * can_transmit - Transmit a CAN message
 * @hfdcan: Pointer to FDCAN handle
 * @id: CAN identifier (standard or extended)
 * @type: Type of identifier (CAN_ID_STD or CAN_ID_EXT)
 * @buff: Pointer to 8-byte data buffer to transmit
 *
 * Transmits a classical CAN message using the specified FDCAN peripheral.
 * The data buffer must contain exactly 8 bytes. The function blocks until
 * the message is placed in the transmit mailbox or an error occurs.
 *
 * Return: HAL_OK on success, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef can_transmit(FDCAN_HandleTypeDef *hfdcan, uint32_t id,
							   enum can_id_type type, uint8_t *buff)
{
	/* TODO: Add log message */
	if (hfdcan == NULL || buff == NULL)
		return HAL_ERROR;

	if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
		return HAL_BUSY; /* whether FIFO is full */

	/* configure transimission messages */
	FDCAN_TxHeaderTypeDef tx_header = {
		.Identifier = id,
		.IdType = (type == CAN_ID_STD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID,
		.TxFrameType = FDCAN_DATA_FRAME, /* Only use data frame */
		.DataLength = FDCAN_DLC_BYTES_8, /* Classical CAN has data length of 8 */
		.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
		.BitRateSwitch = FDCAN_BRS_OFF, /* bit rate switch is FDCAN features */
		.FDFormat = FDCAN_CLASSIC_CAN,	/* Classical CAN */
		.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
		.MessageMarker = 0,
	};

	/* Transmit CAN message */
	return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, buff);
}

/**
 * can_register_rx_callback - Register the global CAN receive callback
 * @callback: Function pointer to be called when CAN message is received
 *
 * Registers a single callback function that will be invoked for all received
 * CAN messages. The callback must parse the received data immediately, as the
 * rx_buff points to a temporary buffer that will be overwritten by subsequent
 * messages. If this callback function is registered multiple times, UB occurs.
 *
 * Context: The registered callback is called from HAL_FDCAN_RxFifo0Callback
 * interrupt context with the received CAN ID and pointer to the 8-byte data buffer.
 *
 * Return: None
 */
void can_register_rx_callback(can_rx_callback_t callback)
{
	/* TODO: Add log messages and add logic to handle error */
	if (callback == NULL)
		return;

	if (rx_callback != NULL) {
		/* TODO: Add warning log and furure logic */
	}

	rx_callback = callback;
}

/*
 * Helper function to get reception buffer
 */
static uint8_t *get_rx_buff(FDCAN_HandleTypeDef *hfdcan)
{
	int16_t idx = get_instance_index(hfdcan);

	if (idx != -1)
		return (uint8_t *)fdcan_inst[idx].rx_buff;
	else
		return NULL;
}

/**
 * HAL_FDCAN_RxFifo0Callback - FDCAN Rx FIFO 0 callback (weak function override)
 * @hfdcan: Pointer to FDCAN handle
 * @RxFifo0ITs: Rx FIFO 0 interrupt status
 *
 * This callback is invoked by the HAL when a new message arrives in RxFIFO0.
 * It retrieves the received message using HAL_FDCAN_GetRxMessage and forwards
 * it to the user-registered callback for processing.
 *
 * Note: This function runs in interrupt context. The user callback must
 * be registered previously, execute quickly and avoid blocking operations.
 * Receive buffers are reused for each message - data must be processed
 * or copied immediately.
 *
 * Return: None
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	/* TODO: Add log message to handle error cases. */
	if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {

		FDCAN_RxHeaderTypeDef rx_header;
		uint8_t *buff = get_rx_buff(hfdcan);
		if (buff == NULL || rx_callback == NULL)
			return;

		/* Get rx message */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, buff) != HAL_OK)
			return;

		/* call rx callback function to process received data */
		rx_callback(&rx_header, buff);
	}
}
