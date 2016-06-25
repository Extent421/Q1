#define AP_CONNECT_DISABLED 0
#define AP_AUTOCONNECT 1

#define SETTINGS_VERSION 0




#define PD_VCLAMP 1<<19
#define PD_VAMP 1<<18
#define PD_IF_DEMOD 1<<17
#define PD_IFAF 1<<16
#define PD_RSSI_SQUELCH 1<<15
#define PD_REGBS 1<<14
#define PD_REGIF 1<<13
#define PD_BC 1<<12
#define PD_DIV4 1<<11
#define PD_5GVCO 1<<10
#define PD_SYN 1<<9
#define PD_AU6M 1<<8
#define PD_6M 1<<7
#define PD_AU6M5 1<<6
#define PD_6M5 1<<5
#define PD_REG1D8 1<<4
#define PD_IFABF 1<<3
#define PD_MIXER 1<<2
#define PD_DIV80 1<<1
#define PD_PLL1D8 1<<0



typedef struct pulseData {
    unsigned long start;
    unsigned long end;
};

