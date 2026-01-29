#include "at86rf.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

uint16_t options = 0;
uint8_t seq_nr = 0;
int frame_len = 0;
int idle_state0 = 0;
int idle_state = 0;
//volatile uint32_t last_connected_change_ms = 0;

uint8_t readRegister(uint8_t reg) {
	uint8_t value = 0;

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	__NOP();
	__NOP();

	uint8_t cmd = reg | 0x80;                   // 0b10xx_xxxx — чтение регистра
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);

	HAL_SPI_Receive(&hspi1, &value, 1, 100);    // просто приём, без dummy

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	return value;
}

void writeRegister(const uint8_t addr, const uint8_t value) {
	uint8_t writeCommand = addr | AT86RF2XX_ACCESS_REG | AT86RF2XX_ACCESS_WRITE;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &writeCommand, sizeof(writeCommand),
			HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &value, sizeof(value), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

}

uint8_t get_status(void) {
	return readRegister(0x01);
}

void set_csma_max_retries(int8_t retries) {
	retries = (retries > 5) ? 5 : retries; /* valid values: 0-5 */
	retries = (retries < 0) ? 7 : retries; /* max < 0 => disable CSMA (set to 7) */

	uint8_t tmp = readRegister(AT86RF2XX_REG__XAH_CTRL_0);
	tmp &= ~(AT86RF2XX_XAH_CTRL_0__MAX_CSMA_RETRIES);
	tmp |= (retries << 1);
	writeRegister(AT86RF2XX_REG__XAH_CTRL_0, tmp);
}

void set_csma_backoff_exp(uint8_t min, uint8_t max) {
	max = (max > 8) ? 8 : max;
	min = (min > max) ? max : min;

	writeRegister(AT86RF2XX_REG__CSMA_BE, (max << 4) | (min));
}

void set_csma_seed(uint8_t entropy[2]) {
	if (entropy == NULL) {
		return;
	}

	writeRegister(AT86RF2XX_REG__CSMA_SEED_0, entropy[0]);

	uint8_t tmp = readRegister(AT86RF2XX_REG__CSMA_SEED_1);
	tmp &= ~(AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1);
	tmp |= entropy[1] & AT86RF2XX_CSMA_SEED_1__CSMA_SEED_1;
	writeRegister(AT86RF2XX_REG__CSMA_SEED_1, tmp);
}

void set_option(uint16_t option, int state) {
	uint8_t tmp;

	/* set option field */
	if (state) {
		options |= option;
		/* trigger option specific actions */
		switch (option) {
		case AT86RF2XX_OPT_CSMA:
			/* Initialize CSMA seed with hardware address */
			set_csma_seed(0b101011);
			set_csma_max_retries(4);
			set_csma_backoff_exp(3, 5);
			break;
		case AT86RF2XX_OPT_PROMISCUOUS:
			/* disable auto ACKs in promiscuous mode */
			tmp = readRegister(AT86RF2XX_REG__CSMA_SEED_1);
			tmp |= AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK;
			writeRegister(AT86RF2XX_REG__CSMA_SEED_1, tmp);
			/* enable promiscuous mode */
			tmp = readRegister(AT86RF2XX_REG__XAH_CTRL_1);
			tmp |= AT86RF2XX_XAH_CTRL_1__AACK_PROM_MODE;
			writeRegister(AT86RF2XX_REG__XAH_CTRL_1, tmp);
			break;
		case AT86RF2XX_OPT_AUTOACK:
			tmp = readRegister(AT86RF2XX_REG__CSMA_SEED_1);
			tmp &= ~(AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK);
			writeRegister(AT86RF2XX_REG__CSMA_SEED_1, tmp);
			break;
		case AT86RF2XX_OPT_TELL_RX_START:
			tmp = readRegister(AT86RF2XX_REG__IRQ_MASK);
			tmp |= AT86RF2XX_IRQ_STATUS_MASK__RX_START;
			writeRegister(AT86RF2XX_REG__IRQ_MASK, tmp);
			break;
		default:
			break;
		}
	} else {
		options &= ~(option);
		/* trigger option specific actions */
		switch (option) {
		case AT86RF2XX_OPT_CSMA:
			/* setting retries to -1 means CSMA disabled */
			set_csma_max_retries(-1);
			break;
		case AT86RF2XX_OPT_PROMISCUOUS:
			/* disable promiscuous mode */
			tmp = readRegister(AT86RF2XX_REG__XAH_CTRL_1);
			tmp &= ~(AT86RF2XX_XAH_CTRL_1__AACK_PROM_MODE);
			writeRegister(AT86RF2XX_REG__XAH_CTRL_1, tmp);
			/* re-enable AUTOACK only if the option is set */
			if (options & AT86RF2XX_OPT_AUTOACK) {
				tmp = readRegister(AT86RF2XX_REG__CSMA_SEED_1);
				tmp &= ~(AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK);
				writeRegister(AT86RF2XX_REG__CSMA_SEED_1, tmp);
			}
			break;
		case AT86RF2XX_OPT_AUTOACK:
			tmp = readRegister(AT86RF2XX_REG__CSMA_SEED_1);
			tmp |= AT86RF2XX_CSMA_SEED_1__AACK_DIS_ACK;
			writeRegister(AT86RF2XX_REG__CSMA_SEED_1, tmp);
			break;
		case AT86RF2XX_OPT_TELL_RX_START:
			tmp = readRegister(AT86RF2XX_REG__IRQ_MASK);
			tmp &= ~AT86RF2XX_IRQ_STATUS_MASK__RX_START;
			writeRegister(AT86RF2XX_REG__IRQ_MASK, tmp);
			break;
		default:
			break;
		}
	}
}

void _set_state(uint8_t state_) {
	writeRegister(AT86RF2XX_REG__TRX_STATE, state_);
	while (get_status() != state_)
		;
}

void set_state(uint8_t state_) {
	_set_state(state_);
	uint8_t old_state = get_status();

	if (state_ == old_state) {
		return;
	}
	/* make sure there is no ongoing transmission, or state transition already
	 * in progress */
	while (old_state == AT86RF2XX_STATE_BUSY_RX_AACK
			|| old_state == AT86RF2XX_STATE_BUSY_TX_ARET
			|| old_state == AT86RF2XX_STATE_IN_PROGRESS) {
		old_state = get_status();
	}

	/* we need to go via PLL_ON if we are moving between RX_AACK_ON <-> TX_ARET_ON */
	if ((old_state == AT86RF2XX_STATE_RX_AACK_ON
			&& state_ == AT86RF2XX_STATE_TX_ARET_ON)
			|| (old_state == AT86RF2XX_STATE_TX_ARET_ON
					&& state_ == AT86RF2XX_STATE_RX_AACK_ON)) {
		_set_state(AT86RF2XX_STATE_PLL_ON);
	}
	/* check if we need to wake up from sleep mode */
	else if (old_state == AT86RF2XX_STATE_SLEEP) {
		//assert_awake();
	}
}

void set_addr_short(uint16_t addr) {
	uint8_t addr_short[2];
	addr_short[0] = addr >> 8;
	addr_short[1] = addr & 0xff;
	writeRegister(AT86RF2XX_REG__SHORT_ADDR_0, addr_short[0]);
	writeRegister(AT86RF2XX_REG__SHORT_ADDR_1, addr_short[1]);
}

void set_addr_long(uint64_t addr) {
	uint8_t addr_long[8];
	for (int i = 0; i < 8; i++) {
		addr_long[i] = (addr >> ((7 - i) * 8));
		writeRegister((AT86RF2XX_REG__IEEE_ADDR_0 + i), addr_long[i]);
	}
}

void set_pan(uint16_t pan) {
	//pan = pan_;
	writeRegister(AT86RF2XX_REG__PAN_ID_0, (uint8_t) pan);
	writeRegister(AT86RF2XX_REG__PAN_ID_1, (pan >> 8));
}

void set_chan(uint8_t channel) {
	uint8_t tmp;

	if (channel < AT86RF2XX_MIN_CHANNEL || channel > AT86RF2XX_MAX_CHANNEL) {
		return;
	}
	//chan = channel;
	tmp = readRegister(AT86RF2XX_REG__PHY_CC_CCA);
	tmp &= ~(AT86RF2XX_PHY_CC_CCA_MASK__CHANNEL);
	tmp |= (channel & AT86RF2XX_PHY_CC_CCA_MASK__CHANNEL);
	writeRegister(AT86RF2XX_REG__PHY_CC_CCA, tmp);
}

void at86rf233_init(uint8_t mode) {

	HAL_GPIO_WritePin(SLP_GPIO_Port, SLP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	uint8_t part_num = readRegister(AT86RF2XX_REG__PART_NUM);
	if (part_num != 0xb) {

		return -1;
	}

	hardware_reset();
	reset_state_machine();

	seq_nr = 0;
	options = 0;

	// Enable promiscuous mode:
	set_addr_short(0x1);
	set_pan(0x0023);
	set_addr_long(0x2222334445666768);

	set_chan(AT86RF2XX_DEFAULT_CHANNEL);

	writeRegister(0x05, 0x0); // tx power

	/* set default options */
	set_option(AT86RF2XX_OPT_PROMISCUOUS, 1);
	set_option(AT86RF2XX_OPT_AUTOACK, 1);
	set_option(AT86RF2XX_OPT_CSMA, 1);
	set_option(AT86RF2XX_OPT_TELL_RX_START, 1);
	set_option(AT86RF2XX_OPT_TELL_RX_END, 1);

	writeRegister(AT86RF2XX_REG__TRX_CTRL_2,
			AT86RF2XX_TRX_CTRL_2_MASK__RX_SAFE_MODE);

	readRegister(0x1C);

	/* disable clock output to save power */
	uint8_t tmp = readRegister(AT86RF2XX_REG__TRX_CTRL_0);
	tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_CTRL);
	tmp &= ~(AT86RF2XX_TRX_CTRL_0_MASK__CLKM_SHA_SEL);
	tmp |= (AT86RF2XX_TRX_CTRL_0_CLKM_CTRL__OFF);
	writeRegister(AT86RF2XX_REG__TRX_CTRL_0, tmp);

	/* enable interrupts */
	writeRegister(AT86RF2XX_REG__IRQ_MASK, AT86RF2XX_IRQ_STATUS_MASK__TRX_END);

	/* clear interrupt flags */
	readRegister(AT86RF2XX_REG__IRQ_STATUS);

	//set_state(6); //16 - RX_ACACK 6 - rx 25 - tx
	set_state(22);
	////////////////////////////////////////////////////////////

	/*uint8_t val = readRegister(0x04); //TX_AUTO_CRC_ON_1
	 val = val | 0b10000;
	 writeRegister(0x04, val);

	 val = readRegister(0x2C); //MAX_FRAME_RETRIES, MAX_CSMA_RETRIES
	 val = val | 0b110110;
	 writeRegister(0x2C, val);


	 writeRegister(0x2C, 0xEA);//CSMA_SEED
	 val = readRegister(0x2E);
	 val = val | 0b110;
	 writeRegister(0x2E, val);

	 //max min be
	 val = readRegister(0x2F);
	 val = val | 0b110001;
	 writeRegister(0x2F, val);


	 readRegister(0x1C);

	 */
	HAL_Delay(1);
}

void force_trx_off() {
	writeRegister(AT86RF2XX_REG__TRX_STATE, AT86RF2XX_TRX_STATE__FORCE_TRX_OFF);
	while (get_status() != AT86RF2XX_STATE_TRX_OFF)
		;
}

void reset_state_machine() {
	uint8_t old_state;

	//assert_awake();

	/* Wait for any state transitions to complete before forcing TRX_OFF */
	do {
		old_state = get_status();
	} while (old_state == AT86RF2XX_STATE_IN_PROGRESS);

	force_trx_off();
}

void hardware_reset(void) {

	/* trigger hardware reset */

	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(AT86RF2XX_RESET_PULSE_WIDTH);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(AT86RF2XX_RESET_PULSE_WIDTH);
}

void sram_write(const uint8_t offset, const uint8_t *data, const size_t len) {
	uint8_t writeCommand = AT86RF2XX_ACCESS_SRAM | AT86RF2XX_ACCESS_WRITE;
	CSRESET
	;
	HAL_SPI_Transmit(&hspi1, &writeCommand, sizeof(writeCommand),
			HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &offset, sizeof(offset), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, data, sizeof(data), HAL_MAX_DELAY);

	// SPI.transfer((char)offset);
	//for (int b=0; b<len; b++) {
	// SPI.transfer(data[b]);
	//}
	CSSET
	;
}

void tx_prepare(void) {
	uint8_t state;
	/* make sure ongoing transmissions are finished */
	do {
		state = get_status();
	} while (state == AT86RF2XX_STATE_BUSY_TX_ARET);

	/* if receiving cancel */
	if (state == AT86RF2XX_STATE_BUSY_RX_AACK) {
		//force_trx_off();
		idle_state = AT86RF2XX_STATE_RX_AACK_ON;
	} else if (state != AT86RF2XX_STATE_TX_ARET_ON) {
		idle_state = state;
	}
	writeRegister(0x02, 0x02);
	frame_len = IEEE802154_FCS_LEN;
}

size_t tx_load(uint8_t *data, size_t len, size_t offset) {
	frame_len += (uint8_t) len;
	sram_write(offset + 1, data, len);
	return offset + len;
}

void tx_exec() {
	/* write frame length field in FIFO */
	sram_write(0, &(frame_len), 1);
	/* trigger sending of pre-loaded frame */
	writeRegister(AT86RF2XX_REG__TRX_STATE, AT86RF2XX_TRX_STATE__TX_START);
	/*if (at86rf2xx.event_cb && (at86rf2xx.options & AT86RF2XX_OPT_TELL_TX_START)) {
	 at86rf2xx.event_cb(NETDEV_EVENT_TX_STARTED, NULL);
	 }*/
}
int send(uint8_t *data, size_t len) {
	tx_prepare();
	tx_load(data, len, 0);
	tx_exec();
	return len;
}

void fb_read(uint8_t *data, const size_t len) {
	uint8_t readCommand = AT86RF2XX_ACCESS_FB | AT86RF2XX_ACCESS_READ;
	CSRESET
	;
	HAL_SPI_Transmit(&hspi1, &readCommand, sizeof(readCommand), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, 0x1, sizeof(readCommand), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, 0x1, sizeof(readCommand), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, 0x1, sizeof(readCommand), HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
	//for (int b=0; b<len; b++) {
	//data[b] = SPI.transfer(0x00);
	//}
	//digitalWrite(cs_pin, HIGH);
	CSSET
	;
}

size_t rx_len(void) {
	uint8_t phr;
	fb_read(&phr, 1);

	/* ignore MSB (refer p.80) and substract length of FCS field */
	return phr;
}

void sram_read(const uint8_t offset, uint8_t *data, const size_t len) {
	uint8_t readCommand = AT86RF2XX_ACCESS_SRAM | AT86RF2XX_ACCESS_READ;
	CSRESET
	;
	HAL_SPI_Transmit(&hspi1, &readCommand, sizeof(readCommand), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &offset, sizeof(offset), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &offset, sizeof(offset), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &offset, sizeof(offset), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &offset, sizeof(offset), HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
	//for (int b=0; b<len; b++) {
	//data[b] = SPI.transfer(0x00);
	//}
	CSSET
	;
	//digitalWrite(cs_pin, HIGH);
}

void rx_read(uint8_t *data, size_t len, size_t offset) {
	/* when reading from SRAM, the different chips from the AT86RF2xx family
	 * behave differently: the AT86F233, the AT86RF232 and the ATRF86212B return
	 * frame length field (PHR) at position 0 and the first data byte at
	 * position 1.
	 * The AT86RF231 does not return the PHR field and return
	 * the first data byte at position 0.
	 */
#ifndef MODULE_AT86RF231
	sram_read(offset + 1, data, len);
#else
     sram_read(offset, data, len);
 #endif
}

void at86rf2xx_receive_data() {
	/*  print the length of the frame
	 *  (including the header)
	 */
	size_t pkt_len = rx_len();
	//if(tval==0)tval=pkt_len;
	//if(pkt_len==6)HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
	/*  Print the frame, byte for byte  */
	uint8_t data[pkt_len];
	rx_read(data, pkt_len, 0);
	//if(strcmp("cheeel", data))
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);

	/* How many frames is this so far?  */

}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

void sleepMode(void) {

	__HAL_RCC_PWR_CLK_ENABLE();

	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IRQ_Pin)
    {
        last_connected_change_ms = HAL_GetTick();

        uint8_t irq = readRegister(AT86RF2XX_REG__IRQ_STATUS);

        if (irq & AT86RF2XX_IRQ_STATUS_MASK__TRX_END)
        {
            uint8_t rssi_raw = readRegister(0x07);

        }
    }
}
*/
