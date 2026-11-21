#include "w25qxx.h"
#include "spi.h"
#include <string.h>
#include <stdio.h>


#define W25Q_CMD_WRITE_ENABLE   0x06
#define W25Q_CMD_READ_STATUS     0x05
#define W25Q_CMD_READ_DATA       0x03
#define W25Q_CMD_PAGE_PROGRAM    0x02
#define W25Q_CMD_SECTOR_ERASE    0x20
#define W25Q_CMD_JEDEC_ID        0x9F
#define W25Q_BUSY_BIT            0x01

extern void Debug_Print(char *str);

/* ── 适配：原代码使用 hspi4 (SPI4)，本板 Flash 挂载在 SPI1: PB3/4/5, CS=PB6 ── */
#define W25Q_SPI_HANDLE hspi1

static GPIO_InitTypeDef   cs_init = {0};

/* Flash CS = SPI1_CS (PB6)，由 main.h 提供 _Port/_Pin 宏 */
static void cs_low(void)  { HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); }

static void wait_busy(void) {
    uint8_t tx[2]={W25Q_CMD_READ_STATUS,0xFF}, rx[2];
    do { cs_low(); HAL_SPI_TransmitReceive(&W25Q_SPI_HANDLE,tx,rx,2,100); cs_high(); } while(rx[1]&W25Q_BUSY_BIT);
}
static void write_enable(void) {
    uint8_t cmd=W25Q_CMD_WRITE_ENABLE;
    cs_low(); HAL_SPI_Transmit(&W25Q_SPI_HANDLE,&cmd,1,100); cs_high();
}

HAL_StatusTypeDef W25Qxx_Init(void) {
    /* ── 适配：重配 SPI1_CS (PB6) 为 GPIO 输出（覆盖 MspInit 的 AF 配置） ── */
    cs_init.Pin   = SPI1_CS_Pin;
    cs_init.Mode  = GPIO_MODE_OUTPUT_PP;
    cs_init.Pull  = GPIO_NOPULL;
    cs_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI1_CS_GPIO_Port, &cs_init);
    cs_high();
    HAL_Delay(5);

    uint8_t tx[4]={W25Q_CMD_JEDEC_ID,0xFF,0xFF,0xFF}, rx[4];
    cs_low(); HAL_SPI_TransmitReceive(&W25Q_SPI_HANDLE,tx,rx,4,100); cs_high();
    return (rx[1]==0||rx[1]==0xFF) ? HAL_ERROR : HAL_OK;
}

void W25Qxx_ReadID(uint8_t *id) {
    uint8_t tx[4]={W25Q_CMD_JEDEC_ID,0xFF,0xFF,0xFF}, rx[4];
    cs_low(); HAL_SPI_TransmitReceive(&W25Q_SPI_HANDLE,tx,rx,4,100); cs_high();
    id[0]=rx[1]; id[1]=rx[2]; id[2]=rx[3];
}

void W25Qxx_EraseSector(uint32_t addr) {
    uint8_t c[4]={W25Q_CMD_SECTOR_ERASE, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
    write_enable();
    cs_low(); HAL_SPI_Transmit(&W25Q_SPI_HANDLE,c,4,100); cs_high();
    wait_busy();
}

void W25Qxx_WritePage(uint32_t addr, const uint8_t *data, uint16_t len) {
    uint8_t h[4]={W25Q_CMD_PAGE_PROGRAM, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
    write_enable();
    cs_low(); HAL_SPI_Transmit(&W25Q_SPI_HANDLE,h,4,100); HAL_SPI_Transmit(&W25Q_SPI_HANDLE,(uint8_t*)data,len,500); cs_high();
    wait_busy();
}

void W25Qxx_ReadData(uint32_t addr, uint8_t *data, uint32_t len) {
    uint8_t c[4]={W25Q_CMD_READ_DATA, (uint8_t)(addr>>16), (uint8_t)(addr>>8), (uint8_t)addr};
    cs_low(); HAL_SPI_Transmit(&W25Q_SPI_HANDLE,c,4,100); HAL_SPI_Receive(&W25Q_SPI_HANDLE,data,(uint16_t)len,5000); cs_high();
}

/* 快照存储（后续使用） */
void W25Qxx_SaveScopeSnapshot(const ScopeSnapshot *snap, uint32_t snap_idx) {
    uint32_t a=FLASH_SCOPE_BASE+snap_idx*sizeof(ScopeSnapshot);
    uint32_t s=a&~(W25Q16_SECTOR_SIZE-1), e=(a+sizeof(ScopeSnapshot)-1)&~(W25Q16_SECTOR_SIZE-1);
    for(uint32_t x=s;x<=e;x+=W25Q16_SECTOR_SIZE) W25Qxx_EraseSector(x);
    uint8_t*b=(uint8_t*)snap;
    for(uint32_t o=0,r=sizeof(ScopeSnapshot);r>0;){ uint16_t c=r>W25Q16_PAGE_SIZE?W25Q16_PAGE_SIZE:(uint16_t)r; W25Qxx_WritePage(a+o,b+o,c); o+=c; r-=c; }
}
void W25Qxx_LoadScopeSnapshot(ScopeSnapshot *snap, uint32_t idx) { W25Qxx_ReadData(FLASH_SCOPE_BASE+idx*sizeof(ScopeSnapshot),(uint8_t*)snap,sizeof(ScopeSnapshot)); }
uint32_t W25Qxx_GetSnapshotCount(void) { return (W25Q16_SECTOR_SIZE*FLASH_SCOPE_MAX_SECTORS)/sizeof(ScopeSnapshot); }

void W25Qxx_Test(void) {
    uint8_t w[256],r[256]={0},id[3];
		char m[128];
		int e=0;
    W25Qxx_ReadID(id); 
		snprintf(m,sizeof(m),"\r\n[TEST] ID:%02X%02X%02X\r\n",id[0],id[1],id[2]); Debug_Print(m);
    for(int j=0;j<256;j++) w[j]=j;
    Debug_Print("[W]"); for(int j=0;j<16;j++){snprintf(m,sizeof(m),"%02X ",w[j]);Debug_Print(m);} Debug_Print("\r\n");
    snprintf(m,sizeof(m),"[E] 0x%08lX\r\n",0x100000UL); Debug_Print(m); W25Qxx_EraseSector(0x100000);
    snprintf(m,sizeof(m),"[W] 0x%08lX\r\n",0x100000UL); Debug_Print(m); W25Qxx_WritePage(0x100000,w,256);
    snprintf(m,sizeof(m),"[R] 0x%08lX\r\n",0x100000UL); Debug_Print(m); W25Qxx_ReadData(0x100000,r,256);
    for(int j=0;j<256;j++) if(w[j]!=r[j]){snprintf(m,sizeof(m)," MIS@0x%08lX w%02X r%02X\r\n",0x100000UL+j,w[j],r[j]);Debug_Print(m);e++;}
    Debug_Print("[R]"); for(int j=0;j<16;j++){snprintf(m,sizeof(m),"%02X ",r[j]);Debug_Print(m);} Debug_Print("\r\n");
    snprintf(m,sizeof(m),e?"FAIL %d\r\n":"PASS\r\n",e); Debug_Print(m);
}