#include "linux/types.h"


extern void RegReadA_lc898111(uint16_t addr, uint8_t* data);
extern void RegWriteA_lc898111(uint16_t addr, uint8_t data);

extern void RamReadA_lc898111(uint16_t addr, uint16_t* data);
extern void RamWriteA_lc898111(uint16_t addr, uint16_t data);

extern void RamRead32A_lc898111(uint16_t addr, unsigned long* data);
extern void RamWrite32A_lc898111(uint16_t addr, uint32_t data);

extern void WitTim_lc898111(unsigned short ms);

