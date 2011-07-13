#include "mcu_periph/i2c.h"

#include <stm32/rcc.h>
#include <stm32/gpio.h>
#include <stm32/flash.h>
#include <stm32/misc.h>

//#include "led.h" 

/////////// DEBUGGING //////////////

static inline void LED1_ON(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_6 , Bit_SET );
}

static inline void LED1_OFF(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_6 , !Bit_SET );
}

static inline void LED2_ON()
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_7 , Bit_SET );
}

static inline void LED2_OFF(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_7 , !Bit_SET );
}

static inline void LED_INIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  LED1_OFF();
  LED2_OFF();
}


static inline void LED_ERROR(uint8_t nr)
{
  for (int i=0;i<20;i++)
  {
    LED1_ON();
    LED1_OFF();
    if (nr == i)
      LED2_OFF();
    else
      LED2_ON();
    LED2_OFF();    
  }
}

static inline void LED_STROBE2(void)
{
LED2_ON();
LED2_OFF();
LED1_ON();
LED1_OFF();
LED2_ON();
LED2_OFF();
LED1_ON();
LED1_OFF();
LED1_OFF();
LED1_OFF();
LED1_OFF();

}

//////////////////////////////////////

#ifdef DEBUG_I2C
#define SPURIOUS_INTERRUPT(_periph, _status, _event) { while(1); }
#define OUT_OF_SYNC_STATE_MACHINE(_periph, _status, _event) { while(1); }
#else
#define SPURIOUS_INTERRUPT(_periph, _status, _event) { }
#define OUT_OF_SYNC_STATE_MACHINE(_periph, _status, _event) { }
#endif

#ifdef USE_I2C1
static I2C_InitTypeDef  I2C1_InitStruct = {
      .I2C_Mode = I2C_Mode_I2C,
      .I2C_DutyCycle = I2C_DutyCycle_2,
      .I2C_OwnAddress1 = 0x00,
      .I2C_Ack = I2C_Ack_Enable,
      .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
      .I2C_ClockSpeed = 200000
};
#endif

#ifdef USE_I2C2
static I2C_InitTypeDef  I2C2_InitStruct = {
      .I2C_Mode = I2C_Mode_I2C,
      .I2C_DutyCycle = I2C_DutyCycle_2,
      .I2C_OwnAddress1 = 0x00,
      .I2C_Ack = I2C_Ack_Enable,
      .I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
      .I2C_ClockSpeed = 400000
};
#endif


#ifdef USE_I2C2



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// IDLE CHECK

static bool_t PPRZ_I2C_IS_IDLE(struct i2c_periph* periph)
{
  return I2C_GetFlagStatus(periph->reg_addr, I2C_FLAG_BUSY) == RESET;
}

// (RE)START

static inline void PPRZ_I2C_SEND_START(struct i2c_periph *periph)
{
  periph->idx_buf = 0;
  I2C_GenerateSTART(periph->reg_addr, ENABLE);
  I2C_ITConfig(periph->reg_addr, I2C_IT_EVT | I2C_IT_ERR, ENABLE);
  I2C_ITConfig(periph->reg_addr, I2C_IT_BUF, DISABLE);
}

static void PPRZ_I2C_START_NEXT_TRANSACTION(struct i2c_periph* periph) 
{
  /* if we have no more transaction to process, stop here */
  if (periph->trans_extract_idx == periph->trans_insert_idx)
  {
    // Should we disable just in case? normally not. So if more interrupts are 
    // triggered there is a problem and we want to know.
    // I2C_ITConfig(periph->reg_addr, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, DISABLE);
    periph->status = I2CIdle;
  }
  /* if not, start next transaction */
  else
  {
    periph->status = I2CStartRequested;
    PPRZ_I2C_SEND_START(periph);
  }
}

static inline void PPRZ_I2C_RESTART(struct i2c_periph *periph)
{
//LED2_ON();
//        I2C_GenerateSTOP(periph->reg_addr, ENABLE);
//      I2C_SendData(periph->reg_addr, 0);
//  I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_BTF);
  periph->status = I2CRestartRequested;
  PPRZ_I2C_SEND_START(periph);
}

// STOP

static inline void PPRZ_I2C_HAS_FINISHED(struct i2c_periph *periph, struct i2c_transaction *trans, enum I2CTransactionStatus _status)
{
  // Finish Current
  trans->status = _status;
 
  // When finished successfully the I2C_FLAG_MLS will be cleared after the stop condition was issued.
  // However: we do not need to wait for it to go the the next step, but if no stop condition was 
  // sent yet than we are still talking to the same slave...
  // When we are here all paths to this function with success have already issued a STOP, the others not.
  // Man: p722:  Stop generation after the current byte transfer or after the current Start condition is sent.
  if (_status != I2CTransSuccess)
  {
    // TODO: we might need to do much more here: see reset functions of antoine...
    I2C_GenerateSTOP(periph->reg_addr, ENABLE);
  }  

  // Jump to the next
  periph->trans_extract_idx++;
  if (periph->trans_extract_idx >= I2C_TRANSACTION_QUEUE_LEN)
    periph->trans_extract_idx = 0;

  PPRZ_I2C_START_NEXT_TRANSACTION(periph);
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// Status Register 1

#define I2C_SR1_BIT_SB			(1<<0)		// Start Condition Met
#define I2C_SR1_BIT_ADDR		(1<<1)		// Address Sent
#define I2C_SR1_BIT_BTF			(1<<2)		// SCL held low
#define I2C_SR1_BIT_RXNE		(1<<6)		// Data Read Available
#define I2C_SR1_BIT_TXE			(1<<7)		// TX buffer space available

#define I2C_SR1_BIT_ERR_BUS		(1<<8)		// Misplaced Start/Stop
#define I2C_SR1_BIT_ERR_AF		(1<<10)		// Ack Failure

#define I2C_SR1_BITS_ERR		(I2C_SR_BIT_ERR_BUS|I2C_SR_BIT_ERR_AF)

// Status Register 2

#define I2C_SR2_BIT_TRA			(1<<2)		// Transmitting
#define I2C_SR2_BIT_BUSY		(1<<1)		// Busy
#define I2C_SR2_BIT_MSL			(1<<0)		// Master Selected

// Control Register 1

#define I2C_CR1_BIT_PE			(1<<0)		// Peripheral Enable
#define I2C_CR1_BIT_START		(1<<8)		// Generate a Start
#define I2C_CR1_BIT_STOP		(1<<9)		// Generate a Stop
#define I2C_CR1_BIT_ACK			(1<<10)		// ACK / NACK
#define I2C_CR1_BIT_POS			(1<<11)		// Ack will control not the next but secondnext received byte
#define I2C_CR1_BIT_SWRST		(1<<15)		// Clear Busy Condition when no stop was detected

// Control Register 2

#define I2C_CR2_BIT_ITERREN		(1<<8)		// Error Interrupt
#define I2C_CR2_BIT_ITEVTEN		(1<<9)		// Event Interrupt
#define I2C_CR2_BIT_ITBUFEN		(1<<10)		// Buffer Interrupt


// Bit Control

#define BIT_X_IS_SET_IN_REG(X,REG)	(((REG) & (X)) == (X))

// STM32 I2C Transaction Types

enum STMI2CTransactionType {
  I2CSend1,
  I2CSend2,
  I2CSendMany,
  I2CGet1,
  I2CGet2,
  I2CGetMany
};

enum STMI2CTransactionEndType {
  STMI2C_StopAndClose,
  STMI2C_StopAndNewStart,
  STMI2C_ReStart,
};

static inline void stmi2c_send1(I2C_TypeDef *regs, uint8_t last_transaction)
{
  volatile uint16_t SR1 = regs->SR1;

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_SB, SR1 ) )
  {
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
    regs->DR = 0x3C;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_ADDR, SR1) )
  {
    // Now read SR2 to clear the ADDR
    volatile uint16_t SR2 = regs->SR2;
    if (! BIT_X_IS_SET_IN_REG(I2C_SR2_BIT_TRA, SR2)) {}

    // Send Bytes
    regs->DR = 0x03;
    regs->CR1 |= I2C_CR1_BIT_STOP;

    // BTF is set as soon as the shift register is empty. 
    // BTF is cleared A) when writing data to DR or B) when a start/stop condition OCCURRED (not was requested)
    // Dummy Data to avoid BTF
    regs->DR = 0x00;

    // After the stop: start again
    if (! last_transaction )
    {
      regs->CR1 |= I2C_CR1_BIT_START;
    }
  }
}

static inline void stmi2c_send2(I2C_TypeDef *regs, uint8_t last_transaction)
{
  volatile uint16_t SR1 = regs->SR1;

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_SB, SR1 ) )
  {
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
    regs->DR = 0x3C;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_ADDR, SR1) )
  {
    // Now read SR2 to clear the ADDR
    volatile uint16_t SR2 = regs->SR2;

    if (! BIT_X_IS_SET_IN_REG(I2C_SR2_BIT_TRA, SR2))
    {
    }


    // Send First 2 bytes
    regs->DR = 0x00;
    regs->DR = 0x18;
    regs->CR2 |= I2C_CR2_BIT_ITBUFEN;
  }
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_TXE, SR1) )
  {
    regs->CR1 |= I2C_CR1_BIT_STOP;
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
    // Also provide some dummy data in DR to silent the BTF interrupt
    regs->DR = 0x00;

    // After the stop: start again
    if (! last_transaction )
    {
      regs->CR1 |= I2C_CR1_BIT_START;
    }
  }
}

static inline void stmi2c_read1(I2C_TypeDef *regs, uint8_t last_transaction)
{
  volatile uint16_t SR1 = regs->SR1;

  // Start Condition Was Just Generated
  if (BIT_X_IS_SET_IN_REG( I2C_SR1_BIT_SB, SR1 ) )
  {
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;
    regs->DR = 0x3C + 1;
  }
  // Address Was Sent
  else if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_ADDR, SR1) )
  {
    // First Clear the ACK bit
    regs->CR1 &= ~ I2C_CR1_BIT_ACK;

    // Only after setting ACK, read SR2 to clear the ADDR (next byte will start arriving)
    volatile uint16_t SR2 = regs->SR2;
      
    // Enable the RXNE to get the result
    regs->CR2 &= ~ I2C_CR2_BIT_ITBUFEN;

    // Program A Stop After
    regs->CR1 |= I2C_CR1_BIT_STOP;

    // And start again
    if (! last_transaction )
    {
      regs->CR1 |= I2C_CR1_BIT_START;
    }
  }
}

static inline void i2c_event(struct i2c_periph *periph)
{
  /*	
	There are 7 possible reasons to get here:

	If IT_EV_FEN
	-------------------------

	We are always interested in all IT_EV_FEV: all are required.

	1) SB		// Start Condition Success in Master mode
	2) ADDR		// Address sent received Acknoledge
	[3 ADDR10]	// -- 10bit address stuff
	[4 STOPF]	// -- only for slaves: master has no stop interrupt
	5) BTF		// I2C has stopped working (it is waiting for new data, all buffers are tx_empty/rx_full)

	// Beware: using the buffered I2C has some interesting properties:
	  -in receive mode: BTF only occurs after the 2nd received byte: after the first byte is received it is 
           in RD but the I2C can still receive a second byte. Only when the 2nd byte is received while the RxNE is 1
	   then a BTF occurs (I2C can not continue receiving bytes or they will get lost)
	  -in transmitmode: when writing a byte to WD, you instantly get a new TxE interrupt while the first is not
	   transmitted yet. The byte was pushed to the I2C shift register and the buffer is ready for more. You can already
	   fill new data in the buffer while the first is still being transmitted for max performance transmission.
        
        // Beware: besides buffering there is also event sheduling. You can send 2 bytes to the buffer, ask for a stop and 
           a new start in one go. 

          -thanks to / because of this buffering and event sheduling there is not 1 interrupt per start / byte / stop
           This also means you must think more in advance and a transaction could be popped from the stack even before it is
           actually completely transmitted. But then you would not know the result yet so you have to keep it until the result
           is known.
	    
	// Beware: the order in which Status is read determines how flags are cleared. You should not just read SR1 & SR2 every time

	If IT_EV_FEN AND IT_EV_BUF
	--------------------------

	Buffer event are not always wanted and are tipically switched on during longer data transfers. It highly depends on the data size.

	6) RxNE
	7) TxE

	--------------------------------------------------------------------------------------------------
	// This driver uses only a subset of the pprz_i2c_states for several reasons:
	// -we have less interrupts than the I2CStatus states (for efficiency)
	// -STM32 has such a powerfull I2C engine with plenty of status register flags that
            only little extra status information needs to be stored. 

	enum I2CStatus {
	  I2CIdle,			// Dummy: Actual I2C Peripheral idle detection is safer with the 
					// hardware status register flag I2C_FLAG_BUSY.

	  I2CStartRequested,		// EV5: used to differentiate S en Sr
	  I2CRestartRequested,		// EV5: used to differentiate S en Sr

	  I2CSendingByte,		// Not used: using the hardware status reg I2C_FLAG_TRA
	  I2CReadingByte,
	  I2CAddrWrSent,		// Since we can do many things at once and we
	  I2CAddrRdSent,		// have buffered sending, these states
	  I2CSendingLastByte,		// do not correspond to the real state of the
	  I2CReadingLastByte,		// STM I2C driver so they are not used
	  I2CStopRequested,		

	  I2CComplete,			// Used to provide the result
	  I2CFailed
	};

	---------

	The STM waits indefinately (holding SCL low) for user interaction:
	a) after a master-start (waiting for address)
	b) after an address (waiting for data)
	   not during data sending when using buffered
	c) after the last byte is transmitted (waiting for either stop or restart)
	   not during data receiving when using buffered
	   not after the last byte is received

	The STM I2C stalls indefinately when a stop condition was attempted that
	did not succeed. The BUSY flag remains on

   */


  ///////////////////////////////////////////////////////////////////////////////////
  // Reading the status:
  // - Caution: this clears several flags and can start transmissions etc... 
  // - Certain flags like STOP / (N)ACK need to be guaranteed to be set before
  //   the transmission of the byte is finished. At higher clock rates that can be
  //   quite fast: so we allow no other interrupt to be triggered in between
  //   reading the status and setting all needed flags

  // Direct Access to the I2C Registers
  static volatile uint8_t stage = 0;

  I2C_TypeDef *regs = (I2C_TypeDef *) periph->reg_addr;

  // Do not read SR2 yet as it might start the reading while an (n)ack bit might be needed first
  
  LED1_ON();
  LED1_OFF();

  stmi2c_read1(regs, FALSE);


  return;

  // Referring to manual: 
  // -Doc ID 13902 Rev 11


  // Check to make sure that user space has an active transaction pending
  if (periph->trans_extract_idx == periph->trans_insert_idx)
  {
    // no transaction?
    periph->errors->unexpected_event_cnt++;
    return;
  }

  struct i2c_transaction* trans = periph->trans[periph->trans_extract_idx];


  LED1_ON();

  //I2C_TypeDef *regs = (I2C_TypeDef *) periph->reg_addr;

  //__disable_irq();
  uint32_t event = I2C_GetLastEvent(periph->reg_addr);

  //uint32_t event = (((uint32_t)(regs->SR2)) << 16) + regs->SR1;

  if (event & I2C_FLAG_TXE)
  {
    LED2_ON();
  }


  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // START: Start Condition in Master Mode: 
  // STM Manual Ev5
  if (event & I2C_FLAG_SB)
  {
    // Periph was waiting for Start
    if (periph->status == I2CStartRequested)
    {
      // Send Read Slave Address
      if(trans->type == I2CTransRx)
      {
        I2C_Send7bitAddress(periph->reg_addr, trans->slave_addr, I2C_Direction_Receiver);
      }
      // Send Write Slave Address
      else
      {
        I2C_Send7bitAddress(periph->reg_addr, trans->slave_addr, I2C_Direction_Transmitter);
      }
      I2C_ITConfig(periph->reg_addr, I2C_IT_BUF, ENABLE);
    }
    // Waiting for Restart: Always Rx
    else if (periph->status == I2CRestartRequested)
    {
      I2C_Send7bitAddress(periph->reg_addr, trans->slave_addr, I2C_Direction_Receiver);
      I2C_ITConfig(periph->reg_addr, I2C_IT_BUF, ENABLE);
    }
    // Problem: this problem need to be triggerd as if the 
    // status was not OK then the buf size is also bad
    else
    {
      PPRZ_I2C_HAS_FINISHED(periph, trans, I2CTransFailed);
    }
    periph->status = I2CAddrWrSent;
  }

  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // TRANSMIT: Buffer Can accept the next byte for transmission
  // --> this means we HAVE TO fill the buffer and/or disable buf interrupts (otherwise this interrupt 
  //     will be triggered until a start/stop occurs which can be quite long = many spurrious interrupts)
  // STM Manual Ev8
  else if (event & I2C_FLAG_TXE) // only possible when TRA(nsmitter) and no flag tra/start/stop/addr
  {
    if (periph->status == I2CRestartRequested)
    {
      // Neglect this interrupt: We just issued the restart last session already
          PPRZ_I2C_RESTART(periph);
    }

    // Do we have more data? and there is buffer room? 
    // (neglect BTF: if it was set it just means we were too slow)
    else if (periph->idx_buf < trans->len_w)
    {
      I2C_SendData(periph->reg_addr, trans->buf[periph->idx_buf]);
      periph->idx_buf++;
      // Was this one the Last? -> Disable the buf interrupt (until next start) and wait for BTF
      // -we could gain a bit of efficiency by already starting the next action but for 
      //  code-readability we will wait until the last tx-byte is sent
      if (periph->idx_buf >= trans->len_w)
      {
        I2C_ITConfig(periph->reg_addr, I2C_IT_BUF, DISABLE);
        // If this is followed by a restart: then we need to set the startbit to avoid extra interrupts.
        if (trans->type != I2CTransTx) 
        {
        I2C_GenerateSTOP(periph->reg_addr, ENABLE);
        }
      } 
    }

    else if (event & I2C_FLAG_BTF)
    {
      // Ready -> Stop
      if (trans->type == I2CTransTx) 
      {
        // STM Manual Ev8_2 
        I2C_GenerateSTOP(periph->reg_addr, ENABLE);
        PPRZ_I2C_HAS_FINISHED(periph, trans, I2CTransSuccess);
      }
      // Rx/Trans -> Restart: 
      // Do not wait for BTF
    }
    // If we had no more data but got no BTF then there is a problem
    else
    {
      PPRZ_I2C_HAS_FINISHED(periph, trans, I2CTransFailed);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////
  // RECEIVE:
  // while receiving: the master needs to signal to the slave if more data is needed
  else if ((event & I2C_FLAG_ADDR) || (event & I2C_FLAG_RXNE))
  {
    // data is available every time RXNE is set. If BTF is set it means that 2 bytes are 
    // ready to read (one in shift register) and I2C has stopped until the buffer can accept new data.
    if (event & I2C_FLAG_RXNE) 
    {
      uint8_t read_byte =  I2C_ReceiveData(periph->reg_addr);
      if (periph->idx_buf < trans->len_r) 
      {
        trans->buf[periph->idx_buf] = read_byte;
        periph->idx_buf++;
      }
    }

    // This last byte has arrived
    if (periph->idx_buf >= trans->len_r) 
    {
      PPRZ_I2C_HAS_FINISHED(periph, trans, I2CTransSuccess);
    }
    // Tell the Slave it will be the last one
    else if (periph->idx_buf >= trans->len_r-1) 
    {
      I2C_AcknowledgeConfig(periph->reg_addr, DISABLE);         // give them a nack once it's done
      I2C_GenerateSTOP(periph->reg_addr, ENABLE);               // and follow with a stop
    }
    // Ask the Slave to send more
    else
    {
      I2C_AcknowledgeConfig(periph->reg_addr, ENABLE);
    }

  }

  // Now re-enable IRQ... it's been too long  
  // __enable_irq();



  LED2_OFF();
  LED1_OFF();


}

static inline void i2c_error(struct i2c_periph *periph)
{
  uint8_t err_nr = 0;
  periph->errors->er_irq_cnt;
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_AF)) {       /* Acknowledge failure */
    periph->errors->ack_fail_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_AF);
    err_nr = 1;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_BERR)) {     /* Misplaced Start or Stop condition */
    periph->errors->miss_start_stop_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_BERR);
    err_nr = 2;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_ARLO)) {     /* Arbitration lost */
    periph->errors->arb_lost_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_ARLO);
    //    I2C_AcknowledgeConfig(I2C2, DISABLE);
    //    uint8_t dummy __attribute__ ((unused)) = I2C_ReceiveData(I2C2);
    //    I2C_GenerateSTOP(I2C2, ENABLE);
    err_nr = 3;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_OVR)) {      /* Overrun/Underrun */
    periph->errors->over_under_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_OVR);
    err_nr = 4;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_PECERR)) {   /* PEC Error in reception */
    periph->errors->pec_recep_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_PECERR);
    err_nr = 5;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_TIMEOUT)) {  /* Timeout or Tlow error */
    periph->errors->timeout_tlow_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_TIMEOUT);
    err_nr = 6;
  }
  if (I2C_GetITStatus(periph->reg_addr, I2C_IT_SMBALERT)) { /* SMBus alert */
    periph->errors->smbus_alert_cnt++;
    I2C_ClearITPendingBit(periph->reg_addr, I2C_IT_SMBALERT);
    err_nr = 7;
  }


  LED_ERROR(err_nr);


  // Check to make sure that user space has an active transaction pending
  if (periph->trans_extract_idx == periph->trans_insert_idx)
  {
    // no transaction?
    periph->errors->unexpected_event_cnt++;
    err_nr = 8;
    return;
  }

  struct i2c_transaction* trans = periph->trans[periph->trans_extract_idx];
 //abort_and_reset(p);



  PPRZ_I2C_HAS_FINISHED(periph, trans, I2CTransFailed);


}

  
/*
  // Make sure the bus is free before resetting (p722)
  if (regs->SR2 & (I2C_FLAG_BUSY >> 16)) {
    // Reset the I2C block
    I2C_SoftwareResetCmd(periph->reg_addr, ENABLE);
    I2C_SoftwareResetCmd(periph->reg_addr, DISABLE);
  }
*/

#endif /* USE_I2C2 */




#ifdef USE_I2C1

struct i2c_errors i2c1_errors;

void i2c1_hw_init(void) {

  i2c1.reg_addr = I2C1;
  i2c1.init_struct = &I2C1_InitStruct;
  i2c1.scl_pin = GPIO_Pin_6;
  i2c1.sda_pin = GPIO_Pin_7;
  i2c1.errors = &i2c1_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c1_errors);

  // Extra
  LED_INIT();
}

void i2c1_ev_irq_handler(void) {
  i2c_event(&i2c1);
}

void i2c1_er_irq_handler(void) {
  i2c_error(&i2c1);
}

#endif /* USE_I2C1 */

#ifdef USE_I2C2

struct i2c_errors i2c2_errors;

void i2c2_hw_init(void) {

  i2c2.reg_addr = I2C2;
  i2c2.init_struct = &I2C2_InitStruct;
  i2c2.scl_pin = GPIO_Pin_10;
  i2c2.sda_pin = GPIO_Pin_11;
  i2c2.errors = &i2c2_errors;

  /* zeros error counter */
  ZEROS_ERR_COUNTER(i2c2_errors);

  /* reset peripheral to default state ( sometimes not achieved on reset :(  ) */
  //I2C_DeInit(I2C2);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef  NVIC_InitStructure;

  /* Configure and enable I2C2 event interrupt --------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C2 err interrupt ----------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable peripheral clocks -------------------------------------------------*/
  /* Enable I2C2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  /* Enable GPIOB clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = i2c2.scl_pin | i2c2.sda_pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  I2C_DeInit(I2C2);

  // enable peripheral
  I2C_Cmd(I2C2, ENABLE);

  I2C_Init(I2C2, i2c2.init_struct);

//  I2C_SoftwareResetCmd(I2C2, ENABLE);
//  I2C_SoftwareResetCmd(I2C2, DISABLE);

  // Reset and initialize I2C HW
  // enable error interrupts
  I2C_ITConfig(I2C2, I2C_IT_ERR, ENABLE);

//  i2c_reset_init(&i2c2);

}


void i2c2_ev_irq_handler(void) {
  i2c_event(&i2c2);
}

void i2c2_er_irq_handler(void) {
  i2c_error(&i2c2);
}

#endif /* USE_I2C2 */



/////////////////////////////////////////////////////////
// Implement Interface Functions

bool_t i2c_submit(struct i2c_periph* periph, struct i2c_transaction* t) {

  uint8_t temp;
  temp = periph->trans_insert_idx + 1;
  if (temp >= I2C_TRANSACTION_QUEUE_LEN) temp = 0;
  if (temp == periph->trans_extract_idx)
    return FALSE;                          // queue full

  t->status = I2CTransPending;

  __disable_irq();
  /* put transacation in queue */
  periph->trans[periph->trans_insert_idx] = t;
  periph->trans_insert_idx = temp;

  /* if peripheral is idle, start the transaction */
  // if (PPRZ_I2C_IS_IDLE(p))
  if (periph->status == I2CIdle)
    PPRZ_I2C_START_NEXT_TRANSACTION(periph);
  /* else it will be started by the interrupt handler when the previous transactions completes */
  __enable_irq();

  return TRUE;
}

bool_t i2c_idle(struct i2c_periph* periph)
{
  return PPRZ_I2C_IS_IDLE(periph);
  //return periph->status == I2CIdle;
}


