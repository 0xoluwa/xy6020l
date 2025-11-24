/**
 * @file xy6020l.h
 * @brief UART control access to XY6020L DCDC
 *
 * This library provides an embedded and simplified ModBus implementation.
 * The data type of interfaces to physical values are in decimal, scaled almost in 0.01.
 *
 * Tested only on a Arduino Pro Micro clone from China
 *
 * Special thanks to user g-radmac for his discovery of the UART protocol!
 * References: 
 *   https://forum.allaboutcircuits.com/threads/exploring-programming-a-xy6020l-power-supply-via-modbus.197022/
 *   https://www.simplymodbus.ca/FAQ.htm
 *   
 * 
 * @author Jens Gleissberg
 * @date 2024
 * @license GNU Lesser General Public License v3.0 or later
 *
 */

#ifndef xy6020l_h
#define xy6020l_h

#include "Arduino.h"

// the XY6020 provides 31 holding registers
#define NB_HREGS 31
#define NB_MEMREGS 14

// Holding Register index
// set voltage
#define HREG_IDX_CV 0  
// set current
#define HREG_IDX_CC 1
// actual voltage  0,01 V
#define HREG_IDX_ACT_V 2
// actual current   0,01 A
#define HREG_IDX_ACT_C 3
// actual output power  0,1 W
#define HREG_IDX_ACT_P 4
// input voltage  0,01 V
#define HREG_IDX_IN_V 5
// output charge  0,001 Ah
#define HREG_IDX_OUT_CHRG 6
#define HREG_IDX_OUT_CHRG_HIGH 7
// output energy  0,001 Wh
#define HREG_IDX_OUT_ENERGY 8
#define HREG_IDX_OUT_ENERGY_HIGH 9
// on time  [h]   ??
#define HREG_IDX_ON_HOUR 0x0A
// on time  [min]  
#define HREG_IDX_ON_MIN 0x0B
// on time  [s]  
#define HREG_IDX_ON_SEC 0x0C
// temperature  0,1 °C / Fahrenheit ?
#define HREG_IDX_TEMP 0x0D
#define HREG_IDX_TEMP_EXD 0x0E
// key lock changes
#define HREG_IDX_LOCK 0x0F
#define HREG_IDX_PROTECT 0x10
#define HREG_IDX_CVCC 0x11
// output on
#define HREG_IDX_OUTPUT_ON 0x12
#define HREG_IDX_FC 0x13
#define HREG_IDX_MODEL 0x16
#define HREG_IDX_VERSION 0x17
#define HREG_IDX_SLAVE_ADD 0x18
#define HREG_IDX_BAUDRATE 0x19
#define HREG_IDX_TEMP_OFS 0x1A
#define HREG_IDX_TEMP_EXT_OFS 0x1B
#define HREG_IDX_MEMORY 0x1D
// Memory register
#define HREG_IDX_M0 0x50
#define HREG_IDX_M_OFFSET 0x10
#define HREG_IDX_M_VSET 0
#define HREG_IDX_M_ISET 1
#define HREG_IDX_M_SLVP 2
#define HREG_IDX_M_SOVP 3
#define HREG_IDX_M_SOCP 4
#define HREG_IDX_M_SOPP 5
#define HREG_IDX_M_SOHPH 6
#define HREG_IDX_M_SOHPM 7
#define HREG_IDX_M_SOAHL 8
#define HREG_IDX_M_SOAHH 9
#define HREG_IDX_M_SOWHL 10
#define HREG_IDX_M_SOWHH 11
#define HREG_IDX_M_SOTP  12
#define HREG_IDX_M_SINI  13


#define TX_RING_BUFFER_SIZE 16
typedef struct {
  uint8_t mHregIdx;
  uint16_t mValue;
} txRingEle;

class TxRingBuffer
{
  private:
    txRingEle* mpIn;
    txRingEle* mpOut;
    int mIn;
    txRingEle mTxBuf[TX_RING_BUFFER_SIZE];
  public:
    TxRingBuffer();
    bool IsEmpty() { return (mIn<1);};
    bool IsFull() { return (mIn>=TX_RING_BUFFER_SIZE);}
    bool AddTx(txRingEle* pTxEle);
    bool AddTx(uint8_t hRegIdx, uint16_t value);
    bool GetTx(txRingEle& pTxEle);
};

typedef struct {
    uint8_t Nr;
    uint16_t VSet;
    uint16_t ISet;
    uint16_t sLVP;
    uint16_t sOVP;
    uint16_t sOCP;
    uint16_t sOPP;
    uint16_t sOHPh;
    uint16_t sOHPm;
    unsigned long sOAH;
    unsigned long sOWH;
    uint16_t sOTP;
    uint16_t sINI;
} tMemory;

/**
 * @class xy6020l
 * @brief Class for controlling the XY6020L DCDC converter
 */
/** @brief option flags */
#define XY6020_OPT_SKIP_SAME_HREG_VALUE 1
#define XY6020_OPT_NO_HREG_UPDATE 2

class xy6020l 
{
  public:
    /**
     * @brief Constructor requires an interface to serial port
     * @param serial Stream object reference (i.e., Serial1)
     * @param adr slave address of the xy device, can be change by setSlaveAdd command
     * @param txPeriod minimum period to wait for next tx message, at times < 50 ms the XY6020 does not send answers
     */
    xy6020l(Stream& serial, uint8_t adr=1, uint8_t txPeriod=50, uint8_t options=XY6020_OPT_SKIP_SAME_HREG_VALUE );
    /**
     * @brief Task method that must be called in loop() function of the main program cyclically.
     * It automatically triggers the reading of the Holding Registers each PERIOD_READ_ALL_HREGS ms.
     */
    void task(void);

    /** @brief requests to read all Hold Registers, reception from XY6020 can be checked via HRegUpdated and data access via read/get methods 
        @return false if tx buffer is full 
    */
    bool ReadAllHRegs(void);
    /** @brief true if the Hold Regs are read after read all register command, asynchron access */
    bool HRegUpdated(void);

    /// @name XY6020L application layer: HReg register access
    /// @{
    // 
    /** @brief voltage setpoint, LSB: 0.01 V , R/W  */
    uint16_t getCV(void) { return (uint16_t)hRegs[ HREG_IDX_CV]; };
    bool setCV( uint16_t cv) { return mTxRingBuffer.AddTx(HREG_IDX_CV, cv);};

    /** @brief constant current  setpoint, LSB: 0.01 A , R/W  */
    uint16_t getCC() { return (uint16_t)hRegs[ HREG_IDX_CC]; };
    bool setCC( uint16_t cc) { return mTxRingBuffer.AddTx(HREG_IDX_CC, cc);};

    /** @brief actual input voltage , LSB: 0.01 V, readonly  */
    uint16_t getInV() { return (uint16_t)hRegs[ HREG_IDX_IN_V ]; };
    /** @brief actual voltage at output, LSB: 0.01 V, readonly  */
    uint16_t getActV() { return (uint16_t)hRegs[ HREG_IDX_ACT_V]; };
    /** @brief actual current at output, LSB: 0.01 A, readonly  */
    uint16_t getActC() { return (uint16_t)hRegs[ HREG_IDX_ACT_C]; };
    /** @brief actual power at output, LSB: 0.01 W, readonly  */
    uint16_t getActP() { return (uint16_t)hRegs[ HREG_IDX_ACT_P]; };
    /** @brief actual charge from output, LSB: 0.001 Ah, readonly, with high uint16_t because not tested yet  */
    uint16_t getCharge() { return (uint16_t)hRegs[ HREG_IDX_OUT_CHRG ]  ; };
    /** @brief actual energy provided from output, LSB: 0.001 Wh, readonly, with high uint16_t because not tested yet  */
    uint16_t getEnergy() { return (uint16_t)hRegs[ HREG_IDX_OUT_ENERGY ]  ; };
    /** @brief actual output time, LSB: 1 h, readonly */
    uint16_t getHour() { return (uint16_t)hRegs[ HREG_IDX_ON_HOUR ]  ; };
    /** @brief actual output time, LSB: 1 min, readonly */
    uint16_t getMin() { return (uint16_t)hRegs[ HREG_IDX_ON_MIN ]  ; };
    /** @brief actual output time, LSB: 1 secs, readonly */
    uint16_t getSec() { return (uint16_t)hRegs[ HREG_IDX_ON_SEC ]  ; };

    /** @brief dcdc temperature, LSB: 0.1°C/F, readonly */
    uint16_t getTemp() { return (uint16_t)hRegs[ HREG_IDX_TEMP ]  ; };
    /** @brief external temperature, LSB: 0.1°C/F, readonly */
    uint16_t getTempExt() { return (uint16_t)hRegs[ HREG_IDX_TEMP_EXD ]; };

    /** @brief lock switch, true = on, R/W   */
    bool getLockOn() { return hRegs[ HREG_IDX_LOCK]>0?true:false; };
    bool setLockOn(bool onState) { return mTxRingBuffer.AddTx(HREG_IDX_LOCK, onState?1:0);};

    /** @brief lock switch, true = on, R/W   */
    uint16_t getProtect() { return hRegs[ HREG_IDX_PROTECT]; };
    bool setProtect(uint16_t state) { return setHReg(HREG_IDX_PROTECT, state );};

    /** @brief returns if CC is active , true = on, read only   */
    bool isCC() { return hRegs[ HREG_IDX_CVCC]>0?true:false; };
    /** @brief returns if CV is active , true = on, read only   */
    bool isCV() { return hRegs[ HREG_IDX_CVCC]<1?true:false; };

    /** @brief output switch, true = on, R/W   */
    bool getOutputOn() { return hRegs[ HREG_IDX_OUTPUT_ON]>0?true:false; };
    bool setOutput(bool onState) { return mTxRingBuffer.AddTx(HREG_IDX_OUTPUT_ON, onState?1:0);};

    /** @brief set the temperature unit to °C, read not implemended because no use  */
    bool setTempAsCelsius(void)  { return setHReg(HREG_IDX_FC, 0);};
    /** @brief set the temperature unit to Fahrenheit, read not implemended because no use  */
    bool setTempAsFahrenheit(void)  { return setHReg(HREG_IDX_FC, 1);};

    /** @brief returns the product number, readonly */
    uint16_t getModel(void)  { return (uint16_t)hRegs[ HREG_IDX_MODEL ]  ; };
    /** @brief returns the version number, readonly */
    uint16_t getVersion(void)  { return (uint16_t)hRegs[ HREG_IDX_VERSION ]  ; };

    /** @brief slave address, R/W, take effect after reset of XY6020L !  */
    uint16_t getSlaveAdd(void) { return (uint16_t)hRegs[ HREG_IDX_SLAVE_ADD]; };
    bool setSlaveAdd( uint16_t add);

    /** @brief baud rate , W, no read option because on use  
        @todo: provide enum for rate number to avoid random/unsupported number */
    bool setBaudrate( uint16_t rate) { return setHReg(HREG_IDX_BAUDRATE, rate);};

    /** @brief internal temperature offset, R/W  */
    uint16_t getTempOfs(void) { return (uint16_t)hRegs[ HREG_IDX_TEMP_OFS]; };
    bool setTempOfs( uint16_t tempOfs) { return setHReg(HREG_IDX_TEMP_OFS, tempOfs );};

    /** @brief external temperature offset, R/W  */
    uint16_t getTempExtOfs(void) { return (uint16_t)hRegs[ HREG_IDX_TEMP_EXT_OFS]; };
    bool setTempExtOfs( uint16_t tempOfs) { return setHReg(HREG_IDX_TEMP_EXT_OFS, tempOfs );};

    /** @brief Presets, R/W  */
    uint16_t getPreset(void) { return (uint16_t)hRegs[ HREG_IDX_MEMORY]; };
    bool setPreset( uint16_t preset) { return setHReg(HREG_IDX_MEMORY, preset );};
    /// @}
    
    bool TxBufEmpty(void) { return ((mTxBufIdx<=0)&&(mTxRingBuffer.IsEmpty()));};
    void SetMemory(tMemory& mem);
    bool GetMemory(tMemory* pMem);
    void PrintMemory(tMemory& mem);

  private:
    uint8_t          mAdr;
    uint8_t          mOptions;
    Stream*       mSerial;
    uint8_t          mRxBufIdx;
    unsigned char mRxBuf[60];
    uint8_t          mRxState;
    bool          mRxThis;
    uint8_t          mRxSize;
    uint16_t          mRxFrameCnt;
    uint16_t          mRxFrameCntLast;
    uint8_t          mLastExceptionCode;

    enum          Response { None, Confirm, Data };
    Response      mResponse;
    /** @brief rx answer belongs to memory request data
     *   M0..M9 -> 0..9 ;  255 no memory   */
    uint8_t          mMemory; 
    long          mTs;
    long          mTO;
    long          mTLastTx;
    uint8_t          mCntTO;
    uint8_t          mTxPeriod;

    int           mTxBufIdx;
    unsigned char mTxBuf[40];
    TxRingBuffer  mTxRingBuffer;

    /** @brief buffer to cache hold regs after reading them at once and to check if update needed for writting regs */
    uint16_t          hRegs[NB_HREGS];
    /** @brief 1 cache for memory register */
    uint16_t          mMem[NB_MEMREGS];
    enum          MemoryState { Send, Wait };
    MemoryState   mMemoryState;
    uint16_t          mMemoryLastFrame;

    bool setHReg(uint8_t nr, uint16_t value);
    bool setHRegFromBuf(void);

    void CRCModBus(int datalen);
    void RxDecodeExceptions(uint8_t cnt);
    bool RxDecode03( uint8_t cnt);
    bool RxDecode06( uint8_t cnt);
    bool RxDecode16( uint8_t cnt);
    void SendReadHReg( uint16_t startReg, uint16_t nbRegs);
    void setMemoryRegs(uint8_t HRegIdx);
};
#endif