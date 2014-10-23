#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/usr/lib/gcc/xscale-elf/3.4.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 213
typedef long unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
#line 8
  int dummy;
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
#line 13
  int dummy;
}  ;
#line 14
struct __nesc_attr_one_nok {
#line 14
  int dummy;
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
#line 17
  int dummy;
}  ;
# 4 "/opt/tinyos-2.1.2/tos/chips/pxa27x/inttypes.h"
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef short int16_t;
typedef unsigned short uint16_t;

typedef int int32_t;
typedef unsigned int uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;

typedef int32_t intptr_t;
typedef uint32_t uintptr_t;
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;





static __inline uint8_t __nesc_ntoh_leuint8(const void * source)  ;




static __inline uint8_t __nesc_hton_leuint8(void * target, uint8_t value)  ;





static __inline int8_t __nesc_ntoh_int8(const void * source)  ;
#line 303
static __inline int8_t __nesc_hton_int8(void * target, int8_t value)  ;
#line 315
static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;






static __inline uint16_t __nesc_ntoh_leuint16(const void * source)  ;




static __inline uint16_t __nesc_hton_leuint16(void * target, uint16_t value)  ;
#line 340
static __inline uint32_t __nesc_ntoh_uint32(const void * source)  ;






static __inline uint32_t __nesc_hton_uint32(void * target, uint32_t value)  ;
#line 431
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char nxdata[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char nxdata[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char nxdata[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char nxdata[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 6 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/sys/lock.h" 3
typedef int _LOCK_T;
typedef int _LOCK_RECURSIVE_T;
# 14 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/sys/_types.h" 3
typedef long _off_t;
__extension__ 
#line 15
typedef long long _off64_t;


typedef int _ssize_t;
# 354 "/usr/lib/gcc/xscale-elf/3.4.3/include/stddef.h" 3
typedef unsigned int wint_t;
# 35 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/sys/_types.h" 3
#line 27
typedef struct __nesc_unnamed4242 {

  int __count;
  union __nesc_unnamed4243 {

    wint_t __wch;
    unsigned char __wchb[4];
  } __value;
} _mbstate_t;

typedef _LOCK_RECURSIVE_T _flock_t;


typedef void *_iconv_t;
# 19 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 40
struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _on_exit_args {
  void *_fnargs[32];
  void *_dso_handle[32];

  __ULong _fntypes;


  __ULong _is_cxa;
};









struct _atexit {
  struct _atexit *_next;
  int _ind;

  void (*_fns[32])(void );
  struct _on_exit_args _on_exit_args;
};









struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 166
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;






  char *_cookie;

  int (*_read)();
  int (*_write)();

  _fpos_t (*_seek)();
  int (*_close)();


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;


  struct _reent *_data;



  _flock_t _lock;
};
#line 259
typedef struct __sFILE __FILE;


struct _glue {

  struct _glue *_next;
  int _niobs;
  __FILE *_iobs;
};
#line 290
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};
#line 565
struct _reent {

  int _errno;




  __FILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  char *_current_locale;

  int __sdidinit;

  void (*__cleanup)();


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
      _mbstate_t _mblen_state;
      _mbstate_t _mbtowc_state;
      _mbstate_t _wctomb_state;
      char _l64a_buf[8];
      char _signal_buf[24];
      int _getdate_err;
      _mbstate_t _mbrlen_state;
      _mbstate_t _mbrtowc_state;
      _mbstate_t _mbsrtowcs_state;
      _mbstate_t _wcrtomb_state;
      _mbstate_t _wcsrtombs_state;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x2b4800e82c10);




  struct _glue __sglue;
  __FILE __sf[3];
};
#line 799
struct _reent;
struct _reent;
# 23 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/string.h" 3
int memcmp();
char *memcpy();

char *memset();
# 28 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/stdlib.h" 3
#line 24
typedef struct __nesc_unnamed4247 {

  int quot;
  int rem;
} div_t;





#line 30
typedef struct __nesc_unnamed4248 {

  long quot;
  long rem;
} ldiv_t;






#line 37
typedef struct __nesc_unnamed4249 {

  long long int quot;
  long long int rem;
} lldiv_t;
# 17 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/math.h" 3
union __dmath {

  __ULong i[2];
  double d;
};




union __dmath;
#line 72
typedef float float_t;
typedef double double_t;
#line 292
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 347
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 25 "/opt/tinyos-2.1.2/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4250 {
#line 26
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;






struct __nesc_attr_atmostonce {
};
#line 37
struct __nesc_attr_atleastonce {
};
#line 38
struct __nesc_attr_exactlyonce {
};
# 51 "/opt/tinyos-2.1.2/tos/types/TinyError.h"
enum __nesc_unnamed4251 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 94 "/opt/tinyos-2.1.2/tos/chips/pxa27x/pxa27xhardware.h"
extern void enableICache();
extern void initSyncFlash();

static __inline uint32_t _pxa27x_clzui(uint32_t i);





typedef uint32_t __nesc_atomic_t;


__inline __nesc_atomic_t __nesc_atomic_start(void )  ;
#line 122
__inline void __nesc_atomic_end(__nesc_atomic_t oldState)  ;
#line 140
static __inline void __nesc_enable_interrupt();
#line 154
static __inline void __nesc_disable_interrupt();
#line 180
typedef uint8_t mcu_power_t  ;
# 119 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/hardware.h"
enum __nesc_unnamed4252 {
  TOS_SLEEP_NONE = 0
};





enum __nesc_unnamed4253 {
  TOSH_period16 = 0x00, 
  TOSH_period32 = 0x01, 
  TOSH_period64 = 0x02, 
  TOSH_period128 = 0x03, 
  TOSH_period256 = 0x04, 
  TOSH_period512 = 0x05, 
  TOSH_period1024 = 0x06, 
  TOSH_period2048 = 0x07
};





const uint8_t TOSH_IRP_TABLE[40] = { 0x05, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0x04, 
0x01, 
0x03, 
0x02, 
0x08, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0x07, 
0xFF, 
0x06, 
0xFF, 
0x0A, 
0x00, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0x09, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF };
#line 263
static inline void TOSH_SET_PIN_DIRECTIONS(void );
# 9 "MoteToMote.h"
#line 4
typedef nx_struct MoteToMoteMsg {

  nx_uint16_t NodeId;
  nx_uint8_t Data;
} __attribute__((packed)) 
MoteToMoteMsg_t;

enum __nesc_unnamed4254 {

  AM_RADIO = 6
};
# 41 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4255 {
#line 41
  int notUsed;
} 
#line 41
TSecond;
typedef struct __nesc_unnamed4256 {
#line 42
  int notUsed;
} 
#line 42
TMilli;
typedef struct __nesc_unnamed4257 {
#line 43
  int notUsed;
} 
#line 43
T32khz;
typedef struct __nesc_unnamed4258 {
#line 44
  int notUsed;
} 
#line 44
TMicro;
# 43 "/opt/tinyos-2.1.2/tos/types/Leds.h"
enum __nesc_unnamed4259 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 39 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 93
#line 87
typedef nx_struct security_header_t {
  unsigned char __nesc_filler0[1];


  nx_uint32_t frameCounter;
  nx_uint8_t keyID[1];
} __attribute__((packed)) security_header_t;
#line 113
#line 95
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;







  nxle_uint8_t network;


  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 118
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 143
#line 128
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 





cc2420_metadata_t;





#line 146
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 179
enum __nesc_unnamed4260 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 28 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum cc2420_security_enums {
  CC2420_NO_SEC = 0, 
  CC2420_CBC_MAC = 1, 
  CC2420_CTR = 2, 
  CC2420_CCM = 3, 
  NO_SEC = 0, 
  CBC_MAC_4 = 1, 
  CBC_MAC_8 = 2, 
  CBC_MAC_16 = 3, 
  CTR = 4, 
  CCM_4 = 5, 
  CCM_8 = 6, 
  CCM_16 = 7
};


enum __nesc_unnamed4261 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/opt/tinyos-2.1.2/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4262 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4263 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 83 "/opt/tinyos-2.1.2/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4264 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4265 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4266 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 121
#line 109
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 123
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 131
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 136
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 60 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/platform_message.h"
#line 57
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 62
typedef union message_footer {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 66
typedef union message_metadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/opt/tinyos-2.1.2/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[28];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 40 "/opt/tinyos-2.1.2/tos/types/IeeeEui64.h"
enum __nesc_unnamed4267 {
#line 40
  IEEE_EUI64_LENGTH = 8
};


#line 42
typedef struct ieee_eui64 {
  uint8_t data[IEEE_EUI64_LENGTH];
} ieee_eui64_t;
# 47 "/opt/tinyos-2.1.2/tos/types/Ieee154.h"
typedef uint16_t ieee154_panid_t;
typedef uint16_t ieee154_saddr_t;
typedef ieee_eui64_t ieee154_laddr_t;







#line 51
typedef struct __nesc_unnamed4268 {
  uint8_t ieee_mode : 2;
  union __nesc_unnamed4269 {
    ieee154_saddr_t saddr;
    ieee154_laddr_t laddr;
  } ieee_addr;
} ieee154_addr_t;



enum __nesc_unnamed4270 {
  IEEE154_BROADCAST_ADDR = 0xffff, 
  IEEE154_LINK_MTU = 127
};

struct ieee154_frame_addr {
  ieee154_addr_t ieee_src;
  ieee154_addr_t ieee_dst;
  ieee154_panid_t ieee_dstpan;
};

enum __nesc_unnamed4271 {
  IEEE154_MIN_HDR_SZ = 6
};
#line 86
enum ieee154_fcf_enums {
  IEEE154_FCF_FRAME_TYPE = 0, 
  IEEE154_FCF_SECURITY_ENABLED = 3, 
  IEEE154_FCF_FRAME_PENDING = 4, 
  IEEE154_FCF_ACK_REQ = 5, 
  IEEE154_FCF_INTRAPAN = 6, 
  IEEE154_FCF_DEST_ADDR_MODE = 10, 
  IEEE154_FCF_SRC_ADDR_MODE = 14
};

enum ieee154_fcf_type_enums {
  IEEE154_TYPE_BEACON = 0, 
  IEEE154_TYPE_DATA = 1, 
  IEEE154_TYPE_ACK = 2, 
  IEEE154_TYPE_MAC_CMD = 3, 
  IEEE154_TYPE_MASK = 7
};

enum ieee154_fcf_addr_mode_enums {
  IEEE154_ADDR_NONE = 0, 
  IEEE154_ADDR_SHORT = 2, 
  IEEE154_ADDR_EXT = 3, 
  IEEE154_ADDR_MASK = 3
};
# 33 "/opt/tinyos-2.1.2/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 44 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/SSP.h"
#line 39
typedef enum __nesc_unnamed4272 {
  SSP_FORMAT_SPI = 0, 
  SSP_FORMAT_TISSP, 
  SSP_FORMAT_UWIRE, 
  SSP_FORMAT_PSP
} SSPFrameFormat_t;

typedef uint8_t SSPDataWidth_t;
typedef uint8_t SSPFifoLevel_t;




#line 49
typedef enum __nesc_unnamed4273 {
  UWIRE_8BIT, 
  UWIRE_16BIT
} SSPMicrowireTxSize_t;




#line 54
typedef enum __nesc_unnamed4274 {
  SSP_NORMALMODE, 
  SSP_NETWORKMODE
} SSPClkMode_t;
# 43 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420TimeSyncMessage.h"
typedef nx_uint32_t timesync_radio_t;





#line 45
typedef nx_struct timesync_footer_t {

  nx_am_id_t type;
  timesync_radio_t timestamp;
} __attribute__((packed)) timesync_footer_t;
typedef TMilli MoteToMoteC$Timer$precision_tag;
typedef TMilli PMICM$chargeMonitorTimer$precision_tag;
typedef TMilli /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$precision_tag;
typedef /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$precision_tag;
typedef /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$frequency_tag;
typedef /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$frequency_tag /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$precision_tag;
typedef uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$size_type;
typedef TMilli /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$frequency_tag;
typedef /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$frequency_tag /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$LocalTime$precision_tag;
typedef /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$frequency_tag /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$precision_tag;
typedef uint32_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$size_type;
enum CounterMilliC$__nesc_unnamed4275 {
  CounterMilliC$OST_CLIENT_ID = 1U
};
enum HilTimerMilliC$__nesc_unnamed4276 {
  HilTimerMilliC$OST_CLIENT_ID = 0U
};
enum CC2420ActiveMessageC$__nesc_unnamed4277 {
  CC2420ActiveMessageC$CC2420_AM_SEND_ID = 0U
};
typedef T32khz CC2420ControlP$StartupTimer$precision_tag;
typedef uint32_t CC2420ControlP$StartupTimer$size_type;
typedef uint16_t CC2420ControlP$ReadRssi$val_t;
typedef T32khz /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$frequency_tag;
typedef /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$frequency_tag /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$precision_tag;
typedef uint32_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$size_type;
enum /*AlarmMultiplexC.Alarm.Alarm32khzC*/Alarm32khzC$0$__nesc_unnamed4278 {
  Alarm32khzC$0$OST_CLIENT_ID = 2U
};
typedef T32khz /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$precision_tag;
typedef uint32_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$size_type;
typedef T32khz /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$frequency_tag;
typedef /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$frequency_tag /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$LocalTime$precision_tag;
typedef /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$frequency_tag /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Counter$precision_tag;
typedef uint32_t /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Counter$size_type;
enum Counter32khzC$__nesc_unnamed4279 {
  Counter32khzC$OST_CLIENT_ID = 3U
};
enum /*CC2420ControlC.Spi*/CC2420SpiC$0$__nesc_unnamed4280 {
  CC2420SpiC$0$CLIENT_ID = 0U
};
enum /*CC2420SpiWireC.HplCC2420SpiC*/HplCC2420SpiC$0$__nesc_unnamed4281 {
  HplCC2420SpiC$0$SPI_CLIENT_ID = 0U
};
enum /*CC2420ControlC.SyncSpiC*/CC2420SpiC$1$__nesc_unnamed4282 {
  CC2420SpiC$1$CLIENT_ID = 1U
};
enum /*CC2420ControlC.RssiResource*/CC2420SpiC$2$__nesc_unnamed4283 {
  CC2420SpiC$2$CLIENT_ID = 2U
};
typedef T32khz CC2420TransmitP$PacketTimeStamp$precision_tag;
typedef uint32_t CC2420TransmitP$PacketTimeStamp$size_type;
typedef T32khz CC2420TransmitP$BackoffTimer$precision_tag;
typedef uint32_t CC2420TransmitP$BackoffTimer$size_type;
enum /*CC2420TransmitC.Spi*/CC2420SpiC$3$__nesc_unnamed4284 {
  CC2420SpiC$3$CLIENT_ID = 3U
};
typedef T32khz CC2420ReceiveP$PacketTimeStamp$precision_tag;
typedef uint32_t CC2420ReceiveP$PacketTimeStamp$size_type;
typedef T32khz CC2420PacketP$PacketTimeStamp32khz$precision_tag;
typedef uint32_t CC2420PacketP$PacketTimeStamp32khz$size_type;
typedef T32khz CC2420PacketP$LocalTime32khz$precision_tag;
typedef TMilli CC2420PacketP$LocalTimeMilli$precision_tag;
typedef TMilli CC2420PacketP$PacketTimeStampMilli$precision_tag;
typedef uint32_t CC2420PacketP$PacketTimeStampMilli$size_type;
typedef T32khz /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$frequency_tag;
typedef /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$frequency_tag /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$LocalTime$precision_tag;
typedef /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$frequency_tag /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$Counter$precision_tag;
typedef uint32_t /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$Counter$size_type;
enum Counter32khz32C$__nesc_unnamed4285 {
  Counter32khz32C$OST_CLIENT_ID = 4U
};
typedef T32khz /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$precision_tag;
typedef /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$precision_tag;
typedef uint32_t /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$size_type;
enum /*CC2420ReceiveC.Spi*/CC2420SpiC$4$__nesc_unnamed4286 {
  CC2420SpiC$4$CLIENT_ID = 4U
};
typedef uint16_t RandomMlcgC$SeedInit$parameter;
enum CC2420TinyosNetworkC$__nesc_unnamed4287 {
  CC2420TinyosNetworkC$TINYOS_N_NETWORKS = 1U
};
enum AMQueueP$__nesc_unnamed4288 {
  AMQueueP$NUM_CLIENTS = 1U
};
# 113 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static void MoteToMoteC$AMControl$startDone(error_t error);
#line 138
static void MoteToMoteC$AMControl$stopDone(error_t error);
# 60 "/opt/tinyos-2.1.2/tos/interfaces/Boot.nc"
static void MoteToMoteC$Boot$booted(void );
# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void MoteToMoteC$AMSend$sendDone(
#line 103
message_t * msg, 






error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



MoteToMoteC$Receive$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void MoteToMoteC$Timer$fired(void );
# 32 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/PlatformReset.nc"
static void PlatformP$PlatformReset$reset(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t PlatformP$Init$init(void );
# 139 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void PlatformP$OST0M3$fired(void );
# 54 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static void PMICM$PI2C$interruptI2C(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t PMICM$Init$init(void );
# 56 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMIC.nc"
static error_t PMICM$PMIC$enableManualCharging(bool enable);
#line 51
static error_t PMICM$PMIC$setCoreVoltage(uint8_t trimValue);


static error_t PMICM$PMIC$getBatteryVoltage(uint8_t *val);
# 150 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void PMICM$PMICGPIO$interruptGPIOPin(void );
# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void PMICM$chargeMonitorTimer$fired(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$fired(void );
#line 83
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$default$fired(
# 48 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b4801136da0);
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(
# 48 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b4801136da0, 
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(
# 48 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b4801136da0);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$runTask(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$fired(void );
# 136 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$getNow(void );
#line 129
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$stop(void );
# 139 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$fired(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$runTask(void );
# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$size_type /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getNow(void );
#line 103
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$startAt(/*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$size_type t0, /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$size_type dt);
#line 116
static /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$size_type /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getAlarm(void );
#line 73
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$stop(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Init$init(void );
# 75 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xOSTimerM$OST4_11Irq$fired(void );
#line 75
static void HplPXA27xOSTimerM$OST0Irq$fired(void );
#line 75
static void HplPXA27xOSTimerM$OST1Irq$fired(void );
#line 75
static void HplPXA27xOSTimerM$OST2Irq$fired(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t HplPXA27xOSTimerM$Init$init(void );
# 75 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xOSTimerM$OST3Irq$fired(void );
# 71 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOSMR(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x2b480120a358, 
# 71 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
uint32_t val);
#line 64
static uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSCR(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x2b480120a358);
# 139 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$default$fired(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x2b480120a358);
# 78 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSMR(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x2b480120a358);
# 85 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOMCR(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x2b480120a358, 
# 85 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
uint32_t val);
#line 112
static bool HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x2b480120a358);
# 103 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static bool HplPXA27xOSTimerM$PXA27xOST$getOSSRbit(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x2b480120a358);
# 57 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOSCR(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x2b480120a358, 
# 57 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
uint32_t val);
#line 119
static void HplPXA27xOSTimerM$PXA27xOST$setOIERbit(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x2b480120a358, 
# 119 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
bool flag);
# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerWatchdog.nc"
static void HplPXA27xOSTimerM$PXA27xWD$enableWatchdog(void );
# 75 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xInterruptM$PXA27xIrq$default$fired(
# 51 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
uint8_t arg_0x2b480120bd08);
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xInterruptM$PXA27xIrq$enable(
# 51 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
uint8_t arg_0x2b480120bd08);
# 60 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static error_t HplPXA27xInterruptM$PXA27xIrq$allocate(
# 51 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
uint8_t arg_0x2b480120bd08);
# 139 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$fired(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Init$init(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$default$overflow(void );
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getICR(void );
#line 45
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setICR(uint32_t val);



static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getISR(void );

static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setISAR(uint32_t val);
#line 43
static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getIDBR(void );
#line 42
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setIDBR(uint32_t val);
# 75 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$fired(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$Init$init(void );
# 75 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xGPIOM$GPIOIrq0$fired(void );
#line 75
static void HplPXA27xGPIOM$GPIOIrq$fired(void );
# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static bool HplPXA27xGPIOM$HplPXA27xGPIOPin$getGPLRbit(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x2b4801368020);
# 134 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x2b4801368020, 
# 134 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
uint8_t func);
#line 101
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGFERbit(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x2b4801368020, 
# 101 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool flag);
#line 83
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGRERbit(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x2b4801368020, 
# 83 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool flag);
#line 52
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x2b4801368020, 
# 52 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool dir);
#line 72
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPCRbit(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x2b4801368020);
# 124 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static bool HplPXA27xGPIOM$HplPXA27xGPIOPin$clearGEDRbit(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x2b4801368020);
# 66 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPSRbit(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x2b4801368020);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t HplPXA27xGPIOM$Init$init(void );
# 132 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIO.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIO$default$fired(void );
# 75 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xGPIOM$GPIOIrq1$fired(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(
# 56 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b4800f867d8);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$default$runTask(
# 56 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b4800f867d8);
# 57 "/opt/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP$Scheduler$init(void );
#line 72
static void SchedulerBasicP$Scheduler$taskLoop(void );
#line 65
static bool SchedulerBasicP$Scheduler$runNextTask(void );
# 76 "/opt/tinyos-2.1.2/tos/interfaces/McuSleep.nc"
static void McuSleepC$McuSleep$sleep(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t LedsP$Init$init(void );
# 72 "/opt/tinyos-2.1.2/tos/interfaces/Leds.nc"
static void LedsP$Leds$led1On(void );
#line 94
static void LedsP$Leds$led2Off(void );
#line 89
static void LedsP$Leds$led2On(void );
# 58 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$disable(
# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014979a8);
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
static void HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$default$fired(
# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014979a8);
# 50 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableFallingEdge(
# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014979a8);
# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableRisingEdge(
# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014979a8);
# 68 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static void HalPXA27xGeneralIOM$GpioInterrupt$default$fired(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b5c60);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static error_t HalPXA27xGeneralIOM$GpioInterrupt$disable(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b5c60);
# 54 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static error_t HalPXA27xGeneralIOM$GpioInterrupt$enableFallingEdge(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b5c60);
# 53 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static error_t HalPXA27xGeneralIOM$GpioInterrupt$enableRisingEdge(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b5c60);
# 150 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$interruptGPIOPin(
# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b4d00);
# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void HalPXA27xGeneralIOM$GeneralIO$makeInput(
# 44 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b4801498228);
# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static bool HalPXA27xGeneralIOM$GeneralIO$get(
# 44 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b4801498228);
# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void HalPXA27xGeneralIOM$GeneralIO$makeOutput(
# 44 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b4801498228);
# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void HalPXA27xGeneralIOM$GeneralIO$set(
# 44 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b4801498228);
# 41 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void HalPXA27xGeneralIOM$GeneralIO$clr(
# 44 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b4801498228);
# 104 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static error_t CC2420CsmaP$SplitControl$start(void );
# 81 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420CsmaP$SubBackoff$requestInitialBackoff(message_t * msg);






static void CC2420CsmaP$SubBackoff$requestCongestionBackoff(message_t * msg);
# 73 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420CsmaP$CC2420Transmit$sendDone(message_t * p_msg, error_t error);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t CC2420CsmaP$Send$send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t CC2420CsmaP$Send$maxPayloadLength(void );
# 76 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420CsmaP$CC2420Power$startOscillatorDone(void );
#line 56
static void CC2420CsmaP$CC2420Power$startVRegDone(void );
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420CsmaP$Resource$granted(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void CC2420CsmaP$sendDone_task$runTask(void );
#line 75
static void CC2420CsmaP$stopDone_task$runTask(void );
#line 75
static void CC2420CsmaP$startDone_task$runTask(void );
# 93 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ControlP$CC2420Config$isAddressRecognitionEnabled(void );
#line 117
static bool CC2420ControlP$CC2420Config$isAutoAckEnabled(void );
#line 112
static bool CC2420ControlP$CC2420Config$isHwAutoAckDefault(void );
#line 66
static ieee_eui64_t CC2420ControlP$CC2420Config$getExtAddr(void );




static uint16_t CC2420ControlP$CC2420Config$getShortAddr(void );
#line 54
static error_t CC2420ControlP$CC2420Config$sync(void );
#line 77
static uint16_t CC2420ControlP$CC2420Config$getPanAddr(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void CC2420ControlP$StartupTimer$fired(void );
# 63 "/opt/tinyos-2.1.2/tos/interfaces/Read.nc"
static void CC2420ControlP$ReadRssi$default$readDone(error_t result, CC2420ControlP$ReadRssi$val_t val);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP$syncDone$runTask(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t CC2420ControlP$Init$init(void );
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420ControlP$SpiResource$granted(void );
#line 102
static void CC2420ControlP$SyncResource$granted(void );
# 71 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420ControlP$CC2420Power$startOscillator(void );
#line 90
static error_t CC2420ControlP$CC2420Power$rxOn(void );
#line 51
static error_t CC2420ControlP$CC2420Power$startVReg(void );
#line 63
static error_t CC2420ControlP$CC2420Power$stopVReg(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void CC2420ControlP$sync$runTask(void );
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420ControlP$Resource$release(void );
#line 88
static error_t CC2420ControlP$Resource$request(void );
# 68 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static void CC2420ControlP$InterruptCCA$fired(void );
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420ControlP$RssiResource$granted(void );
# 139 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$fired(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$lateAlarm$runTask(void );
# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$size_type /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$getNow(void );
#line 66
static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$start(/*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$size_type dt);






static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$stop(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Init$init(void );
# 54 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captureFallingEdge(void );
#line 66
static void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$disable(void );
#line 53
static error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captureRisingEdge(void );
# 68 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$fired(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$overflow(void );
# 139 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$fired(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Init$init(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Counter$size_type /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Counter$get(void );
# 82 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
static void CC2420SpiP$SpiPacket$sendDone(
#line 75
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 62 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420SpiP$Fifo$continueRead(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b480173f328, 
# 62 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 91
static void CC2420SpiP$Fifo$default$writeDone(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b480173f328, 
# 91 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 82
static cc2420_status_t CC2420SpiP$Fifo$write(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b480173f328, 
# 82 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420SpiP$Fifo$beginRead(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b480173f328, 
# 51 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length);
#line 71
static void CC2420SpiP$Fifo$default$readDone(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b480173f328, 
# 71 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 31 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP$ChipSpiResource$abortRelease(void );







static error_t CC2420SpiP$ChipSpiResource$attemptRelease(void );
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420SpiP$SpiResource$granted(void );
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420SpiP$Ram$write(
# 47 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint16_t arg_0x2b480173e4d8, 
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Ram.nc"
uint8_t offset, uint8_t * data, uint8_t length);
# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420SpiP$Reg$read(
# 48 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b480173d220, 
# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t *data);







static cc2420_status_t CC2420SpiP$Reg$write(
# 48 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b480173d220, 
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
uint16_t data);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420SpiP$Resource$release(
# 45 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b4801741158);
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420SpiP$Resource$immediateRequest(
# 45 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b4801741158);
# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420SpiP$Resource$request(
# 45 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b4801741158);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420SpiP$Resource$default$granted(
# 45 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b4801741158);
# 128 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool CC2420SpiP$Resource$isOwner(
# 45 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b4801741158);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void CC2420SpiP$grant$runTask(void );
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420SpiP$Strobe$strobe(
# 49 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b480173c020);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t StateImplP$Init$init(void );
# 56 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
static void StateImplP$State$toIdle(
# 67 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
uint8_t arg_0x2b48017a5d50);
# 66 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
static bool StateImplP$State$isState(
# 67 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
uint8_t arg_0x2b48017a5d50, 
# 66 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
uint8_t myState);
#line 61
static bool StateImplP$State$isIdle(
# 67 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
uint8_t arg_0x2b48017a5d50);
# 45 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
static error_t StateImplP$State$requestState(
# 67 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
uint8_t arg_0x2b48017a5d50, 
# 45 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
uint8_t reqState);





static void StateImplP$State$forceState(
# 67 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
uint8_t arg_0x2b48017a5d50, 
# 51 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
uint8_t reqState);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$Init$init(void );
# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$enqueue(resource_client_id_t id);
#line 53
static bool /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEmpty(void );








static bool /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEnqueued(resource_client_id_t id);







static resource_client_id_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$dequeue(void );
# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(
# 52 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b480180a020);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$immediateRequested(
# 52 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b480180a020);
# 65 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(
# 56 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b4801808538);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(
# 56 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b4801808538);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$release(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b48017dee30);
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$immediateRequest(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b48017dee30);
# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$request(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b48017dee30);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$default$granted(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b48017dee30);
# 128 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static bool /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$isOwner(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b48017dee30);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void );
# 70 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
static error_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$send(
# 58 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSpiPioM.nc"
uint8_t arg_0x2b4801861448, 
# 59 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
#line 82
static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$default$sendDone(
# 58 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSpiPioM.nc"
uint8_t arg_0x2b4801861448, 
# 75 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 70 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$interruptSSP(void );
# 45 "/opt/tinyos-2.1.2/tos/interfaces/SpiByte.nc"
static uint8_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiByte$write(uint8_t tx);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$Init$init(void );
# 70 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
static void HalPXA27xSSPControlP$SSP$interruptSSP(void );
# 150 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void IM2CC2420InitSpiP$RXD$interruptGPIOPin(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t IM2CC2420InitSpiP$Init$init(void );
# 150 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void IM2CC2420InitSpiP$SCLK$interruptGPIOPin(void );
#line 150
static void IM2CC2420InitSpiP$TXD$interruptGPIOPin(void );
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xSSPP$SSP2Irq$default$enable(void );









static void HplPXA27xSSPP$SSP3Irq$fired(void );
# 43 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
static void HplPXA27xSSPP$HplPXA27xSSP$setSSSR(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
uint8_t arg_0x2b48018f7cd8, 
# 43 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
uint32_t val);








static void HplPXA27xSSPP$HplPXA27xSSP$setSSTO(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
uint8_t arg_0x2b48018f7cd8, 
# 52 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
uint32_t val);
#line 49
static void HplPXA27xSSPP$HplPXA27xSSP$setSSDR(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
uint8_t arg_0x2b48018f7cd8, 
# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
uint32_t val);
#line 37
static void HplPXA27xSSPP$HplPXA27xSSP$setSSCR0(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
uint8_t arg_0x2b48018f7cd8, 
# 37 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
uint32_t val);






static uint32_t HplPXA27xSSPP$HplPXA27xSSP$getSSSR(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
uint8_t arg_0x2b48018f7cd8);
# 70 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
static void HplPXA27xSSPP$HplPXA27xSSP$default$interruptSSP(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
uint8_t arg_0x2b48018f7cd8);
# 50 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
static uint32_t HplPXA27xSSPP$HplPXA27xSSP$getSSDR(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
uint8_t arg_0x2b48018f7cd8);
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
static void HplPXA27xSSPP$HplPXA27xSSP$setSSCR1(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
uint8_t arg_0x2b48018f7cd8, 
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
uint32_t val);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t HplPXA27xSSPP$Init$init(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
uint8_t arg_0x2b48018f71b8);
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xSSPP$SSP1Irq$default$enable(void );









static void HplPXA27xDMAM$DMAIrq$fired(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t HplPXA27xDMAM$Init$init(void );
# 52 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMACntl.nc"
static uint32_t HplPXA27xDMAM$HplPXA27xDMACntl$getDINT(void );
# 81 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
static void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCSR(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x2b480198d670, 
# 81 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
uint32_t val);









static void HplPXA27xDMAM$HplPXA27xDMAChnl$default$interruptDMA(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x2b480198d670);
# 83 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
static void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCMD(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x2b480198d670, 
# 83 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
uint32_t val);
# 55 "/opt/tinyos-2.1.2/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC$amAddress(void );
# 50 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t ActiveMessageAddressC$ActiveMessageAddress$amAddress(void );




static am_group_t ActiveMessageAddressC$ActiveMessageAddress$amGroup(void );
# 48 "/opt/tinyos-2.1.2/tos/interfaces/LocalIeeeEui64.nc"
static ieee_eui64_t LocalIeeeEui64C$LocalIeeeEui64$getId(void );
# 66 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420TransmitP$RadioBackoff$setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420TransmitP$RadioBackoff$setInitialBackoff(uint16_t backoffTime);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static void CC2420TransmitP$CaptureSFD$captured(uint16_t time);
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void CC2420TransmitP$BackoffTimer$fired(void );
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP$CC2420Receive$receive(uint8_t type, message_t * message);
# 51 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420TransmitP$Send$send(message_t * p_msg, bool useCca);
# 24 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP$ChipSpiResource$releasing(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t CC2420TransmitP$Init$init(void );
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420TransmitP$SpiResource$granted(void );
# 95 "/opt/tinyos-2.1.2/tos/interfaces/StdControl.nc"
static error_t CC2420TransmitP$StdControl$start(void );









static error_t CC2420TransmitP$StdControl$stop(void );
# 91 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420TransmitP$TXFIFO$writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420TransmitP$TXFIFO$readDone(uint8_t * data, uint8_t length, error_t error);
# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ReceiveP$CC2420Config$syncDone(error_t error);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void CC2420ReceiveP$receiveDone_task$runTask(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP$CC2420Receive$sfd_dropped(void );
#line 49
static void CC2420ReceiveP$CC2420Receive$sfd(uint32_t time);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t CC2420ReceiveP$Init$init(void );
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420ReceiveP$SpiResource$granted(void );
# 91 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420ReceiveP$RXFIFO$writeDone(uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420ReceiveP$RXFIFO$readDone(uint8_t * data, uint8_t length, error_t error);
# 68 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static void CC2420ReceiveP$InterruptFIFOP$fired(void );
# 95 "/opt/tinyos-2.1.2/tos/interfaces/StdControl.nc"
static error_t CC2420ReceiveP$StdControl$start(void );









static error_t CC2420ReceiveP$StdControl$stop(void );
# 77 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void CC2420PacketP$CC2420Packet$setNetwork(message_t * p_msg, uint8_t networkId);
#line 75
static uint8_t CC2420PacketP$CC2420Packet$getNetwork(message_t * p_msg);
# 70 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
static void CC2420PacketP$PacketTimeStamp32khz$clear(
#line 66
message_t * msg);
#line 78
static void CC2420PacketP$PacketTimeStamp32khz$set(
#line 73
message_t * msg, 




CC2420PacketP$PacketTimeStamp32khz$size_type value);
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420PacketP$CC2420PacketBody$getHeader(message_t * msg);










static cc2420_metadata_t * CC2420PacketP$CC2420PacketBody$getMetadata(message_t * msg);
# 58 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420PacketP$PacketTimeSyncOffset$get(
#line 53
message_t * msg);
#line 50
static bool CC2420PacketP$PacketTimeSyncOffset$isSet(
#line 46
message_t * msg);
# 139 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$fired(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$Init$init(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 52 "/opt/tinyos-2.1.2/tos/interfaces/Random.nc"
static uint16_t RandomMlcgC$Random$rand16(void );
#line 46
static uint32_t RandomMlcgC$Random$rand32(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t RandomMlcgC$Init$init(void );
# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static void UniqueSendP$SubSend$sendDone(
#line 96
message_t * msg, 



error_t error);
#line 75
static error_t UniqueSendP$Send$send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t UniqueSendP$Send$maxPayloadLength(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t UniqueSendP$Init$init(void );
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP$SubReceive$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t UniqueReceiveP$Init$init(void );
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP$DuplicateReceive$default$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP$SubSend$sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP$SubReceive$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void CC2420TinyosNetworkP$grantTask$runTask(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP$ActiveSend$send(
#line 67
message_t * msg, 







uint8_t len);
#line 125
static 
#line 123
void * 

CC2420TinyosNetworkP$ActiveSend$getPayload(
#line 122
message_t * msg, 


uint8_t len);
#line 112
static uint8_t CC2420TinyosNetworkP$ActiveSend$maxPayloadLength(void );
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP$BareReceive$default$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP$Resource$release(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x2b4801d92280);
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP$Resource$immediateRequest(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x2b4801d92280);
# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420TinyosNetworkP$Resource$request(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x2b4801d92280);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP$Resource$default$granted(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x2b4801d92280);
# 125 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static 
#line 123
void * 

CC2420TinyosNetworkP$BareSend$getPayload(
#line 122
message_t * msg, 


uint8_t len);
#line 100
static void CC2420TinyosNetworkP$BareSend$default$sendDone(
#line 96
message_t * msg, 



error_t error);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$Init$init(void );
# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$enqueue(resource_client_id_t id);
#line 53
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void );








static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(resource_client_id_t id);







static resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void );
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP$SubReceive$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static void CC2420ActiveMessageP$SubSend$sendDone(
#line 96
message_t * msg, 



error_t error);
# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ActiveMessageP$CC2420Config$syncDone(error_t error);
# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP$RadioBackoff$default$requestCca(
# 54 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e14d60, 
# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
#line 81
static void CC2420ActiveMessageP$RadioBackoff$default$requestInitialBackoff(
# 54 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e14d60, 
# 81 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);






static void CC2420ActiveMessageP$RadioBackoff$default$requestCongestionBackoff(
# 54 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e14d60, 
# 88 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/SendNotifier.nc"
static void CC2420ActiveMessageP$SendNotifier$default$aboutToSend(
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e14258, 
# 59 "/opt/tinyos-2.1.2/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP$SubBackoff$requestCca(message_t * msg);
#line 81
static void CC2420ActiveMessageP$SubBackoff$requestInitialBackoff(message_t * msg);






static void CC2420ActiveMessageP$SubBackoff$requestCongestionBackoff(message_t * msg);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
static uint8_t CC2420ActiveMessageP$Packet$payloadLength(
#line 74
message_t * msg);
#line 126
static 
#line 123
void * 


CC2420ActiveMessageP$Packet$getPayload(
#line 121
message_t * msg, 




uint8_t len);
#line 106
static uint8_t CC2420ActiveMessageP$Packet$maxPayloadLength(void );
#line 94
static void CC2420ActiveMessageP$Packet$setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t CC2420ActiveMessageP$AMSend$send(
# 48 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e197d8, 
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP$Snoop$default$receive(
# 50 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e17480, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
#line 78
static 
#line 74
message_t * 



CC2420ActiveMessageP$Receive$default$receive(
# 49 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e18908, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 68 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
static am_addr_t CC2420ActiveMessageP$AMPacket$address(void );









static am_addr_t CC2420ActiveMessageP$AMPacket$destination(
#line 74
message_t * amsg);
#line 103
static void CC2420ActiveMessageP$AMPacket$setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 147
static am_id_t CC2420ActiveMessageP$AMPacket$type(
#line 143
message_t * amsg);
#line 162
static void CC2420ActiveMessageP$AMPacket$setType(
#line 158
message_t * amsg, 



am_id_t t);
#line 136
static bool CC2420ActiveMessageP$AMPacket$isForMe(
#line 133
message_t * amsg);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420ActiveMessageP$RadioResource$granted(void );
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static void /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$sendDone(
#line 96
message_t * msg, 



error_t error);
# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$sendDone(
# 48 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2b4801ec0488, 
# 103 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$send(
# 46 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b4801ec1318, 
# 67 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 100
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$default$sendDone(
# 46 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b4801ec1318, 
# 96 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$runTask(void );
#line 75
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask$runTask(void );
# 104 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static error_t MoteToMoteC$AMControl$start(void );
# 126 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
static 
#line 123
void * 


MoteToMoteC$Packet$getPayload(
#line 121
message_t * msg, 




uint8_t len);
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t MoteToMoteC$AMSend$send(am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 72 "/opt/tinyos-2.1.2/tos/interfaces/Leds.nc"
static void MoteToMoteC$Leds$led1On(void );
#line 94
static void MoteToMoteC$Leds$led2Off(void );
#line 89
static void MoteToMoteC$Leds$led2On(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void MoteToMoteC$Timer$startPeriodic(uint32_t dt);
# 26 "MoteToMoteC.nc"
uint8_t MoteToMoteC$dummyVar1 = 123;

static inline void MoteToMoteC$Boot$booted(void );




bool MoteToMoteC$RadioBusy = FALSE;
message_t MoteToMoteC$packet;

static inline void MoteToMoteC$Timer$fired(void );
#line 52
static inline void MoteToMoteC$AMSend$sendDone(message_t *msg, error_t error);







static inline void MoteToMoteC$AMControl$startDone(error_t error);
#line 72
static inline void MoteToMoteC$AMControl$stopDone(error_t error);




static inline message_t *MoteToMoteC$Receive$receive(message_t *msg, void *payload, uint8_t len);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t PlatformP$InitL2$init(void );
#line 62
static error_t PlatformP$InitL0$init(void );
#line 62
static error_t PlatformP$InitL3$init(void );
# 71 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void PlatformP$OST0M3$setOSMR(uint32_t val);
#line 64
static uint32_t PlatformP$OST0M3$getOSCR(void );
#line 112
static bool PlatformP$OST0M3$clearOSSRbit(void );






static void PlatformP$OST0M3$setOIERbit(bool flag);
# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerWatchdog.nc"
static void PlatformP$PXA27xWD$enableWatchdog(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t PlatformP$InitL1$init(void );
# 52 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/PlatformP.nc"
static inline error_t PlatformP$Init$init(void );
#line 117
static inline void PlatformP$PlatformReset$reset(void );






static inline void PlatformP$OST0M3$fired(void );
# 32 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/PlatformReset.nc"
static void PMICM$PlatformReset$reset(void );
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static uint32_t PMICM$PI2C$getICR(void );
#line 45
static void PMICM$PI2C$setICR(uint32_t val);



static uint32_t PMICM$PI2C$getISR(void );

static void PMICM$PI2C$setISAR(uint32_t val);
#line 43
static uint32_t PMICM$PI2C$getIDBR(void );
#line 42
static void PMICM$PI2C$setIDBR(uint32_t val);
# 134 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void PMICM$PMICGPIO$setGAFRpin(uint8_t func);
#line 101
static void PMICM$PMICGPIO$setGFERbit(bool flag);
#line 52
static void PMICM$PMICGPIO$setGPDRbit(bool dir);
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void PMICM$chargeMonitorTimer$startPeriodic(uint32_t dt);
#line 78
static void PMICM$chargeMonitorTimer$stop(void );
# 101 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
bool PMICM$gotReset;


static error_t PMICM$readPMIC(uint8_t address, uint8_t *value, uint8_t numBytes);
#line 151
static error_t PMICM$writePMIC(uint8_t address, uint8_t value);
#line 171
static inline void PMICM$startLDOs(void );
#line 219
static inline error_t PMICM$Init$init(void );
#line 256
static inline void PMICM$PI2C$interruptI2C(void );
#line 271
static inline void PMICM$PMICGPIO$interruptGPIOPin(void );
#line 300
static inline error_t PMICM$PMIC$setCoreVoltage(uint8_t trimValue);
#line 334
static error_t PMICM$getPMICADCVal(uint8_t channel, uint8_t *val);
#line 355
static inline error_t PMICM$PMIC$getBatteryVoltage(uint8_t *val);
#line 372
static inline error_t PMICM$PMIC$enableManualCharging(bool enable);
#line 407
static inline void PMICM$chargeMonitorTimer$fired(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$postTask(void );
# 136 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow(void );
#line 129
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt);
#line 78
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$stop(void );




static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$fired(
# 48 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x2b4801136da0);
#line 71
enum /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$__nesc_unnamed4289 {
#line 71
  VirtualizeTimerC$0$updateFromTimer = 0U
};
#line 71
typedef int /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$__nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer];
#line 53
enum /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$__nesc_unnamed4290 {

  VirtualizeTimerC$0$NUM_TIMERS = 3U, 
  VirtualizeTimerC$0$END_OF_LIST = 255
};








#line 59
typedef struct /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$__nesc_unnamed4291 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer_t;

/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$m_timers[/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$NUM_TIMERS];




static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$fireTimers(uint32_t now);
#line 100
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
#line 139
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$fired(void );




static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt);









static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(uint8_t num);
#line 204
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$postTask(void );
# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getNow(void );
#line 103
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type dt);
#line 116
static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getAlarm(void );
#line 73
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$stop(void );
# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$fired(void );
# 74 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$__nesc_unnamed4292 {
#line 74
  AlarmToTimerC$0$fired = 1U
};
#line 74
typedef int /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$__nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired];
#line 55
uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_dt;
bool /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot);
#line 71
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$fired(void );
#line 93
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$getNow(void );
# 71 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSMR(uint32_t val);
#line 64
static uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSCR(void );
#line 78
static uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSMR(void );






static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOMCR(uint32_t val);
#line 112
static bool /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$clearOSSRbit(void );
#line 103
static bool /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSSRbit(void );
#line 57
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSCR(uint32_t val);
#line 119
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(bool flag);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTInit$init(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$postTask(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$fired(void );
# 56 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
enum /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$__nesc_unnamed4293 {
#line 56
  HalPXA27xAlarmM$0$lateAlarm = 2U
};
#line 56
typedef int /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$__nesc_sillytask_lateAlarm[/*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm];
#line 53
bool /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning;
uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT;

static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$runTask(void );






static inline error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Init$init(void );
#line 117
static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$stop(void );
#line 132
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$startAt(uint32_t t0, uint32_t dt);
#line 152
static inline uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getNow(void );



static inline uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getAlarm(void );



static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$fired(void );
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xOSTimerM$OST4_11Irq$enable(void );
#line 60
static error_t HplPXA27xOSTimerM$OST4_11Irq$allocate(void );




static void HplPXA27xOSTimerM$OST0Irq$enable(void );
#line 60
static error_t HplPXA27xOSTimerM$OST0Irq$allocate(void );




static void HplPXA27xOSTimerM$OST1Irq$enable(void );
#line 60
static error_t HplPXA27xOSTimerM$OST1Irq$allocate(void );




static void HplPXA27xOSTimerM$OST2Irq$enable(void );
#line 60
static error_t HplPXA27xOSTimerM$OST2Irq$allocate(void );




static void HplPXA27xOSTimerM$OST3Irq$enable(void );
#line 60
static error_t HplPXA27xOSTimerM$OST3Irq$allocate(void );
# 139 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$fired(
# 39 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x2b480120a358);
#line 53
bool HplPXA27xOSTimerM$gfInitialized = FALSE;

static inline void HplPXA27xOSTimerM$DispatchOSTInterrupt(uint8_t id);





static error_t HplPXA27xOSTimerM$Init$init(void );
#line 87
static void HplPXA27xOSTimerM$PXA27xOST$setOSCR(uint8_t chnl_id, uint32_t val);









static uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSCR(uint8_t chnl_id);










static void HplPXA27xOSTimerM$PXA27xOST$setOSMR(uint8_t chnl_id, uint32_t val);





static inline uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSMR(uint8_t chnl_id);






static void HplPXA27xOSTimerM$PXA27xOST$setOMCR(uint8_t chnl_id, uint32_t val);
#line 138
static bool HplPXA27xOSTimerM$PXA27xOST$getOSSRbit(uint8_t chnl_id);










static bool HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(uint8_t chnl_id);
#line 163
static void HplPXA27xOSTimerM$PXA27xOST$setOIERbit(uint8_t chnl_id, bool flag);
#line 186
static inline void HplPXA27xOSTimerM$PXA27xWD$enableWatchdog(void );









static inline void HplPXA27xOSTimerM$OST0Irq$fired(void );




static inline void HplPXA27xOSTimerM$OST1Irq$fired(void );




static inline void HplPXA27xOSTimerM$OST2Irq$fired(void );




static inline void HplPXA27xOSTimerM$OST3Irq$fired(void );




static inline void HplPXA27xOSTimerM$OST4_11Irq$fired(void );
#line 233
static void HplPXA27xOSTimerM$PXA27xOST$default$fired(uint8_t chnl_id);
# 75 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xInterruptM$PXA27xIrq$fired(
# 51 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
uint8_t arg_0x2b480120bd08);







static inline uint32_t HplPXA27xInterruptM$getICHP(void );








void hplarmv_irq(void ) __attribute((interrupt("IRQ")))   ;
#line 85
void hplarmv_fiq(void ) __attribute((interrupt("FIQ")))   ;



static uint8_t HplPXA27xInterruptM$usedPriorities = 0;




static error_t HplPXA27xInterruptM$allocate(uint8_t id, bool level, uint8_t priority);
#line 162
static void HplPXA27xInterruptM$enable(uint8_t id);
#line 188
static inline error_t HplPXA27xInterruptM$PXA27xIrq$allocate(uint8_t id);




static inline void HplPXA27xInterruptM$PXA27xIrq$enable(uint8_t id);
#line 227
static inline void HplPXA27xInterruptM$PXA27xIrq$default$fired(uint8_t id);
# 71 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSMR(uint32_t val);
#line 85
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOMCR(uint32_t val);
#line 112
static bool /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$clearOSSRbit(void );
#line 57
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSCR(uint32_t val);
#line 119
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOIERbit(bool flag);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTInit$init(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$overflow(void );
# 56 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline error_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Init$init(void );
#line 91
static inline void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$fired(void );
#line 104
static inline void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$default$overflow(void );
# 54 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$interruptI2C(void );
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$enable(void );
#line 60
static error_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$allocate(void );
# 52 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
bool /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$m_fInit = FALSE;

static inline error_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$Init$init(void );
#line 90
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setIDBR(uint32_t val);








static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getIDBR(void );







static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setICR(uint32_t val);








static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getICR(void );
#line 132
static inline uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getISR(void );







static inline void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setISAR(uint32_t val);
#line 157
static inline void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$fired(void );
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xGPIOM$GPIOIrq0$enable(void );
#line 60
static error_t HplPXA27xGPIOM$GPIOIrq0$allocate(void );




static void HplPXA27xGPIOM$GPIOIrq$enable(void );
#line 60
static error_t HplPXA27xGPIOM$GPIOIrq$allocate(void );
# 150 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x2b4801368020);
# 132 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIO.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIO$fired(void );
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xGPIOM$GPIOIrq1$enable(void );
#line 60
static error_t HplPXA27xGPIOM$GPIOIrq1$allocate(void );
# 58 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
bool HplPXA27xGPIOM$gfInitialized = FALSE;

static inline error_t HplPXA27xGPIOM$Init$init(void );
#line 80
static bool HplPXA27xGPIOM$HplPXA27xGPIOPin$getGPLRbit(uint8_t pin);




static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(uint8_t pin, bool dir);
#line 101
static inline void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPSRbit(uint8_t pin);





static inline void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPCRbit(uint8_t pin);





static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGRERbit(uint8_t pin, bool flag);
#line 129
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGFERbit(uint8_t pin, bool flag);
#line 150
static bool HplPXA27xGPIOM$HplPXA27xGPIOPin$clearGEDRbit(uint8_t pin);







static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(uint8_t pin, uint8_t func);
#line 259
static inline void HplPXA27xGPIOM$HplPXA27xGPIO$default$fired(void );



static inline void HplPXA27xGPIOM$GPIOIrq$fired(void );
#line 307
static inline void HplPXA27xGPIOM$GPIOIrq0$fired(void );




static inline void HplPXA27xGPIOM$GPIOIrq1$fired(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t RealMainP$SoftwareInit$init(void );
# 60 "/opt/tinyos-2.1.2/tos/interfaces/Boot.nc"
static void RealMainP$Boot$booted(void );
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t RealMainP$PlatformInit$init(void );
# 57 "/opt/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
static void RealMainP$Scheduler$init(void );
#line 72
static void RealMainP$Scheduler$taskLoop(void );
#line 65
static bool RealMainP$Scheduler$runNextTask(void );
# 63 "/opt/tinyos-2.1.2/tos/system/RealMainP.nc"
int main(void )   ;
# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(
# 56 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x2b4800f867d8);
# 76 "/opt/tinyos-2.1.2/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP$McuSleep$sleep(void );
# 61 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP$__nesc_unnamed4294 {

  SchedulerBasicP$NUM_TASKS = 15U, 
  SchedulerBasicP$NO_TASK = 255
};

uint8_t SchedulerBasicP$m_head;
uint8_t SchedulerBasicP$m_tail;
uint8_t SchedulerBasicP$m_next[SchedulerBasicP$NUM_TASKS];








static __inline uint8_t SchedulerBasicP$popTask(void );
#line 97
static inline bool SchedulerBasicP$isWaiting(uint8_t id);




static inline bool SchedulerBasicP$pushTask(uint8_t id);
#line 124
static inline void SchedulerBasicP$Scheduler$init(void );









static bool SchedulerBasicP$Scheduler$runNextTask(void );
#line 149
static inline void SchedulerBasicP$Scheduler$taskLoop(void );
#line 170
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id);




static void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id);
# 53 "/opt/tinyos-2.1.2/tos/chips/pxa27x/McuSleepC.nc"
static inline void McuSleepC$McuSleep$sleep(void );
# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void LedsP$Led0$makeOutput(void );
#line 40
static void LedsP$Led0$set(void );





static void LedsP$Led1$makeOutput(void );
#line 40
static void LedsP$Led1$set(void );
static void LedsP$Led1$clr(void );




static void LedsP$Led2$makeOutput(void );
#line 40
static void LedsP$Led2$set(void );
static void LedsP$Led2$clr(void );
# 56 "/opt/tinyos-2.1.2/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void );
#line 89
static inline void LedsP$Leds$led1On(void );
#line 104
static inline void LedsP$Leds$led2On(void );




static inline void LedsP$Leds$led2Off(void );
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
static void HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$fired(
# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014979a8);
# 68 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static void HalPXA27xGeneralIOM$GpioInterrupt$fired(
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b5c60);
# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static bool HalPXA27xGeneralIOM$HplPXA27xGPIOPin$getGPLRbit(
# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b4d00);
# 101 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGFERbit(
# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b4d00, 
# 101 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool flag);
#line 83
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGRERbit(
# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b4d00, 
# 83 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool flag);
#line 52
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPDRbit(
# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b4d00, 
# 52 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool dir);
#line 72
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPCRbit(
# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b4d00);
# 124 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static bool HalPXA27xGeneralIOM$HplPXA27xGPIOPin$clearGEDRbit(
# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b4d00);
# 66 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPSRbit(
# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x2b48014b4d00);




static void HalPXA27xGeneralIOM$GeneralIO$set(uint8_t pin);





static void HalPXA27xGeneralIOM$GeneralIO$clr(uint8_t pin);
#line 77
static inline bool HalPXA27xGeneralIOM$GeneralIO$get(uint8_t pin);





static inline void HalPXA27xGeneralIOM$GeneralIO$makeInput(uint8_t pin);










static inline void HalPXA27xGeneralIOM$GeneralIO$makeOutput(uint8_t pin);










static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableRisingEdge(uint8_t pin);







static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableFallingEdge(uint8_t pin);
#line 129
static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$disable(uint8_t pin);








static inline error_t HalPXA27xGeneralIOM$GpioInterrupt$enableRisingEdge(uint8_t pin);



static inline error_t HalPXA27xGeneralIOM$GpioInterrupt$enableFallingEdge(uint8_t pin);



static inline error_t HalPXA27xGeneralIOM$GpioInterrupt$disable(uint8_t pin);



static inline void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$interruptGPIOPin(uint8_t pin);







static inline void HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$default$fired(uint8_t pin);



static inline void HalPXA27xGeneralIOM$GpioInterrupt$default$fired(uint8_t pin);
# 113 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
static void CC2420CsmaP$SplitControl$startDone(error_t error);
#line 138
static void CC2420CsmaP$SplitControl$stopDone(error_t error);
# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420CsmaP$RadioBackoff$requestCca(message_t * msg);
#line 81
static void CC2420CsmaP$RadioBackoff$requestInitialBackoff(message_t * msg);






static void CC2420CsmaP$RadioBackoff$requestCongestionBackoff(message_t * msg);
#line 66
static void CC2420CsmaP$SubBackoff$setCongestionBackoff(uint16_t backoffTime);
#line 60
static void CC2420CsmaP$SubBackoff$setInitialBackoff(uint16_t backoffTime);
# 51 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static error_t CC2420CsmaP$CC2420Transmit$send(message_t * p_msg, bool useCca);
# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static void CC2420CsmaP$Send$sendDone(
#line 96
message_t * msg, 



error_t error);
# 52 "/opt/tinyos-2.1.2/tos/interfaces/Random.nc"
static uint16_t CC2420CsmaP$Random$rand16(void );
# 95 "/opt/tinyos-2.1.2/tos/interfaces/StdControl.nc"
static error_t CC2420CsmaP$SubControl$start(void );









static error_t CC2420CsmaP$SubControl$stop(void );
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420CsmaP$CC2420PacketBody$getHeader(message_t * msg);










static cc2420_metadata_t * CC2420CsmaP$CC2420PacketBody$getMetadata(message_t * msg);
# 71 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Power.nc"
static error_t CC2420CsmaP$CC2420Power$startOscillator(void );
#line 90
static error_t CC2420CsmaP$CC2420Power$rxOn(void );
#line 51
static error_t CC2420CsmaP$CC2420Power$startVReg(void );
#line 63
static error_t CC2420CsmaP$CC2420Power$stopVReg(void );
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420CsmaP$Resource$release(void );
#line 88
static error_t CC2420CsmaP$Resource$request(void );
# 66 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
static bool CC2420CsmaP$SplitControlState$isState(uint8_t myState);
#line 45
static error_t CC2420CsmaP$SplitControlState$requestState(uint8_t reqState);





static void CC2420CsmaP$SplitControlState$forceState(uint8_t reqState);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t CC2420CsmaP$sendDone_task$postTask(void );
#line 67
static error_t CC2420CsmaP$stopDone_task$postTask(void );
#line 67
static error_t CC2420CsmaP$startDone_task$postTask(void );
# 74 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
enum CC2420CsmaP$__nesc_unnamed4295 {
#line 74
  CC2420CsmaP$startDone_task = 3U
};
#line 74
typedef int CC2420CsmaP$__nesc_sillytask_startDone_task[CC2420CsmaP$startDone_task];
enum CC2420CsmaP$__nesc_unnamed4296 {
#line 75
  CC2420CsmaP$stopDone_task = 4U
};
#line 75
typedef int CC2420CsmaP$__nesc_sillytask_stopDone_task[CC2420CsmaP$stopDone_task];
enum CC2420CsmaP$__nesc_unnamed4297 {
#line 76
  CC2420CsmaP$sendDone_task = 5U
};
#line 76
typedef int CC2420CsmaP$__nesc_sillytask_sendDone_task[CC2420CsmaP$sendDone_task];
#line 58
enum CC2420CsmaP$__nesc_unnamed4298 {
  CC2420CsmaP$S_STOPPED, 
  CC2420CsmaP$S_STARTING, 
  CC2420CsmaP$S_STARTED, 
  CC2420CsmaP$S_STOPPING, 
  CC2420CsmaP$S_TRANSMITTING
};

message_t * CC2420CsmaP$m_msg;

error_t CC2420CsmaP$sendErr = SUCCESS;


bool CC2420CsmaP$ccaOn;






static inline void CC2420CsmaP$shutdown(void );


static error_t CC2420CsmaP$SplitControl$start(void );
#line 122
static inline error_t CC2420CsmaP$Send$send(message_t *p_msg, uint8_t len);
#line 173
static inline uint8_t CC2420CsmaP$Send$maxPayloadLength(void );
#line 205
static inline void CC2420CsmaP$CC2420Transmit$sendDone(message_t *p_msg, error_t err);




static inline void CC2420CsmaP$CC2420Power$startVRegDone(void );



static inline void CC2420CsmaP$Resource$granted(void );



static inline void CC2420CsmaP$CC2420Power$startOscillatorDone(void );




static inline void CC2420CsmaP$SubBackoff$requestInitialBackoff(message_t *msg);






static inline void CC2420CsmaP$SubBackoff$requestCongestionBackoff(message_t *msg);
#line 244
static inline void CC2420CsmaP$sendDone_task$runTask(void );
#line 257
static inline void CC2420CsmaP$startDone_task$runTask(void );







static inline void CC2420CsmaP$stopDone_task$runTask(void );









static inline void CC2420CsmaP$shutdown(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
static void CC2420ControlP$CC2420Config$syncDone(error_t error);
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP$RXCTRL1$write(uint16_t data);
# 48 "/opt/tinyos-2.1.2/tos/interfaces/LocalIeeeEui64.nc"
static ieee_eui64_t CC2420ControlP$LocalIeeeEui64$getId(void );
# 66 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void CC2420ControlP$StartupTimer$start(CC2420ControlP$StartupTimer$size_type dt);
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP$MDMCTRL0$write(uint16_t data);
# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP$RSTN$makeOutput(void );
#line 40
static void CC2420ControlP$RSTN$set(void );
static void CC2420ControlP$RSTN$clr(void );
# 63 "/opt/tinyos-2.1.2/tos/interfaces/Read.nc"
static void CC2420ControlP$ReadRssi$readDone(error_t result, CC2420ControlP$ReadRssi$val_t val);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t CC2420ControlP$syncDone$postTask(void );
# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP$RSSI$read(uint16_t *data);







static cc2420_status_t CC2420ControlP$TXCTRL$write(uint16_t data);
#line 63
static cc2420_status_t CC2420ControlP$IOCFG0$write(uint16_t data);
# 50 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ControlP$ActiveMessageAddress$amAddress(void );




static am_group_t CC2420ControlP$ActiveMessageAddress$amGroup(void );
# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420ControlP$CSN$makeOutput(void );
#line 40
static void CC2420ControlP$CSN$set(void );
static void CC2420ControlP$CSN$clr(void );




static void CC2420ControlP$VREN$makeOutput(void );
#line 40
static void CC2420ControlP$VREN$set(void );
static void CC2420ControlP$VREN$clr(void );
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP$SXOSCON$strobe(void );
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420ControlP$SpiResource$release(void );
#line 88
static error_t CC2420ControlP$SpiResource$request(void );
#line 120
static error_t CC2420ControlP$SyncResource$release(void );
#line 88
static error_t CC2420ControlP$SyncResource$request(void );
# 76 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Power.nc"
static void CC2420ControlP$CC2420Power$startOscillatorDone(void );
#line 56
static void CC2420ControlP$CC2420Power$startVRegDone(void );
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420ControlP$IOCFG1$write(uint16_t data);
#line 63
static cc2420_status_t CC2420ControlP$FSCTRL$write(uint16_t data);
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP$SRXON$strobe(void );
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420ControlP$Resource$granted(void );
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420ControlP$IEEEADR$write(uint8_t offset, uint8_t * data, uint8_t length);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ControlP$InterruptCCA$disable(void );
#line 53
static error_t CC2420ControlP$InterruptCCA$enableRisingEdge(void );
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420ControlP$RssiResource$release(void );
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ControlP$SRFOFF$strobe(void );
# 125 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
enum CC2420ControlP$__nesc_unnamed4299 {
#line 125
  CC2420ControlP$sync = 6U
};
#line 125
typedef int CC2420ControlP$__nesc_sillytask_sync[CC2420ControlP$sync];
enum CC2420ControlP$__nesc_unnamed4300 {
#line 126
  CC2420ControlP$syncDone = 7U
};
#line 126
typedef int CC2420ControlP$__nesc_sillytask_syncDone[CC2420ControlP$syncDone];
#line 90
#line 84
typedef enum CC2420ControlP$__nesc_unnamed4301 {
  CC2420ControlP$S_VREG_STOPPED, 
  CC2420ControlP$S_VREG_STARTING, 
  CC2420ControlP$S_VREG_STARTED, 
  CC2420ControlP$S_XOSC_STARTING, 
  CC2420ControlP$S_XOSC_STARTED
} CC2420ControlP$cc2420_control_state_t;

uint8_t CC2420ControlP$m_channel;

uint8_t CC2420ControlP$m_tx_power;

uint16_t CC2420ControlP$m_pan;

uint16_t CC2420ControlP$m_short_addr;

ieee_eui64_t CC2420ControlP$m_ext_addr;

bool CC2420ControlP$m_sync_busy;


bool CC2420ControlP$autoAckEnabled;


bool CC2420ControlP$hwAutoAckDefault;


bool CC2420ControlP$addressRecognition;


bool CC2420ControlP$hwAddressRecognition;

CC2420ControlP$cc2420_control_state_t CC2420ControlP$m_state = CC2420ControlP$S_VREG_STOPPED;



static void CC2420ControlP$writeFsctrl(void );
static void CC2420ControlP$writeMdmctrl0(void );
static void CC2420ControlP$writeId(void );
static inline void CC2420ControlP$writeTxctrl(void );





static inline error_t CC2420ControlP$Init$init(void );
#line 188
static inline error_t CC2420ControlP$Resource$request(void );







static inline error_t CC2420ControlP$Resource$release(void );







static inline error_t CC2420ControlP$CC2420Power$startVReg(void );
#line 216
static inline error_t CC2420ControlP$CC2420Power$stopVReg(void );







static inline error_t CC2420ControlP$CC2420Power$startOscillator(void );
#line 268
static inline error_t CC2420ControlP$CC2420Power$rxOn(void );
#line 298
static inline ieee_eui64_t CC2420ControlP$CC2420Config$getExtAddr(void );



static uint16_t CC2420ControlP$CC2420Config$getShortAddr(void );







static inline uint16_t CC2420ControlP$CC2420Config$getPanAddr(void );
#line 323
static inline error_t CC2420ControlP$CC2420Config$sync(void );
#line 355
static inline bool CC2420ControlP$CC2420Config$isAddressRecognitionEnabled(void );
#line 382
static inline bool CC2420ControlP$CC2420Config$isHwAutoAckDefault(void );






static inline bool CC2420ControlP$CC2420Config$isAutoAckEnabled(void );









static inline void CC2420ControlP$SyncResource$granted(void );
#line 413
static inline void CC2420ControlP$SpiResource$granted(void );




static inline void CC2420ControlP$RssiResource$granted(void );
#line 431
static void CC2420ControlP$StartupTimer$fired(void );









static inline void CC2420ControlP$InterruptCCA$fired(void );
#line 465
static inline void CC2420ControlP$sync$runTask(void );



static inline void CC2420ControlP$syncDone$runTask(void );









static void CC2420ControlP$writeFsctrl(void );
#line 496
static void CC2420ControlP$writeMdmctrl0(void );
#line 515
static void CC2420ControlP$writeId(void );
#line 533
static inline void CC2420ControlP$writeTxctrl(void );
#line 545
static inline void CC2420ControlP$ReadRssi$default$readDone(error_t error, uint16_t data);
# 71 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOSMR(uint32_t val);
#line 64
static uint32_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$getOSCR(void );
#line 85
static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOMCR(uint32_t val);
#line 112
static bool /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$clearOSSRbit(void );
#line 103
static bool /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$getOSSRbit(void );
#line 57
static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOSCR(uint32_t val);
#line 119
static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOIERbit(bool flag);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTInit$init(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$lateAlarm$postTask(void );
# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$fired(void );
# 56 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
enum /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$__nesc_unnamed4302 {
#line 56
  HalPXA27xAlarmM$1$lateAlarm = 8U
};
#line 56
typedef int /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$__nesc_sillytask_lateAlarm[/*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$lateAlarm];
#line 53
bool /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mfRunning;
uint32_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mMinDeltaT;

static inline void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$lateAlarm$runTask(void );






static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Init$init(void );
#line 93
static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$start(uint32_t dt);
#line 117
static inline void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$stop(void );
#line 152
static inline uint32_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$getNow(void );







static inline void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$fired(void );
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captured(uint16_t time);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$disable(void );
#line 54
static error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$enableFallingEdge(void );
#line 53
static error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$enableRisingEdge(void );
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$size_type /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$get(void );
# 59 "/opt/tinyos-2.1.2/tos/lib/gpio/SoftCaptureP.nc"
static inline error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captureRisingEdge(void );



static inline error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captureFallingEdge(void );



static inline void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$disable(void );




static inline void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$fired(void );







static inline void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$overflow(void );
# 71 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOSMR(uint32_t val);
#line 64
static uint32_t /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$getOSCR(void );
#line 85
static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOMCR(uint32_t val);
#line 112
static bool /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$clearOSSRbit(void );
#line 57
static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOSCR(uint32_t val);
#line 119
static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOIERbit(bool flag);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTInit$init(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Counter$overflow(void );
# 56 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline error_t /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Init$init(void );
#line 72
static inline uint32_t /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Counter$get(void );
#line 91
static inline void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$fired(void );
# 70 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
static error_t CC2420SpiP$SpiPacket$send(
#line 59
uint8_t * txBuf, 

uint8_t * rxBuf, 








uint16_t len);
# 91 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static void CC2420SpiP$Fifo$writeDone(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b480173f328, 
# 91 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
#line 71
static void CC2420SpiP$Fifo$readDone(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b480173f328, 
# 71 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
uint8_t * data, uint8_t length, error_t error);
# 24 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420SpiP$ChipSpiResource$releasing(void );
# 45 "/opt/tinyos-2.1.2/tos/interfaces/SpiByte.nc"
static uint8_t CC2420SpiP$SpiByte$write(uint8_t tx);
# 56 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
static void CC2420SpiP$WorkingState$toIdle(void );




static bool CC2420SpiP$WorkingState$isIdle(void );
#line 45
static error_t CC2420SpiP$WorkingState$requestState(uint8_t reqState);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420SpiP$SpiResource$release(void );
#line 97
static error_t CC2420SpiP$SpiResource$immediateRequest(void );
#line 88
static error_t CC2420SpiP$SpiResource$request(void );
#line 128
static bool CC2420SpiP$SpiResource$isOwner(void );
#line 102
static void CC2420SpiP$Resource$granted(
# 45 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
uint8_t arg_0x2b4801741158);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t CC2420SpiP$grant$postTask(void );
# 88 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
enum CC2420SpiP$__nesc_unnamed4303 {
#line 88
  CC2420SpiP$grant = 9U
};
#line 88
typedef int CC2420SpiP$__nesc_sillytask_grant[CC2420SpiP$grant];
#line 63
enum CC2420SpiP$__nesc_unnamed4304 {
  CC2420SpiP$RESOURCE_COUNT = 5U, 
  CC2420SpiP$NO_HOLDER = 0xFF
};


enum CC2420SpiP$__nesc_unnamed4305 {
  CC2420SpiP$S_IDLE, 
  CC2420SpiP$S_BUSY
};


uint16_t CC2420SpiP$m_addr;


uint8_t CC2420SpiP$m_requests = 0;


uint8_t CC2420SpiP$m_holder = CC2420SpiP$NO_HOLDER;


bool CC2420SpiP$release;


static error_t CC2420SpiP$attemptRelease(void );







static inline void CC2420SpiP$ChipSpiResource$abortRelease(void );






static inline error_t CC2420SpiP$ChipSpiResource$attemptRelease(void );




static error_t CC2420SpiP$Resource$request(uint8_t id);
#line 126
static error_t CC2420SpiP$Resource$immediateRequest(uint8_t id);
#line 149
static error_t CC2420SpiP$Resource$release(uint8_t id);
#line 178
static inline bool CC2420SpiP$Resource$isOwner(uint8_t id);





static inline void CC2420SpiP$SpiResource$granted(void );




static cc2420_status_t CC2420SpiP$Fifo$beginRead(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 209
static inline error_t CC2420SpiP$Fifo$continueRead(uint8_t addr, uint8_t *data, 
uint8_t len);



static inline cc2420_status_t CC2420SpiP$Fifo$write(uint8_t addr, uint8_t *data, 
uint8_t len);
#line 260
static cc2420_status_t CC2420SpiP$Ram$write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len);
#line 287
static inline cc2420_status_t CC2420SpiP$Reg$read(uint8_t addr, uint16_t *data);
#line 305
static cc2420_status_t CC2420SpiP$Reg$write(uint8_t addr, uint16_t data);
#line 318
static cc2420_status_t CC2420SpiP$Strobe$strobe(uint8_t addr);










static inline void CC2420SpiP$SpiPacket$sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error);








static error_t CC2420SpiP$attemptRelease(void );
#line 358
static inline void CC2420SpiP$grant$runTask(void );








static inline void CC2420SpiP$Resource$default$granted(uint8_t id);


static inline void CC2420SpiP$Fifo$default$readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error);


static inline void CC2420SpiP$Fifo$default$writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error);
# 74 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
uint8_t StateImplP$state[4U];

enum StateImplP$__nesc_unnamed4306 {
  StateImplP$S_IDLE = 0
};


static inline error_t StateImplP$Init$init(void );
#line 96
static error_t StateImplP$State$requestState(uint8_t id, uint8_t reqState);
#line 111
static inline void StateImplP$State$forceState(uint8_t id, uint8_t reqState);






static inline void StateImplP$State$toIdle(uint8_t id);







static inline bool StateImplP$State$isIdle(uint8_t id);






static bool StateImplP$State$isState(uint8_t id, uint8_t myState);
# 49 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
enum /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$__nesc_unnamed4307 {
#line 49
  FcfsResourceQueueC$1$NO_ENTRY = 0xFF
};
uint8_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$resQ[1U];
uint8_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qHead = /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;
uint8_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qTail = /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;

static inline error_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$Init$init(void );




static inline bool /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEmpty(void );



static inline bool /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$dequeue(void );
#line 82
static inline error_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$enqueue(resource_client_id_t id);
# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(
# 52 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b480180a020);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$immediateRequested(
# 52 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b480180a020);
# 65 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(
# 56 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b4801808538);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(
# 56 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b4801808538);
# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Queue$enqueue(resource_client_id_t id);
#line 53
static bool /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Queue$isEmpty(void );
#line 70
static resource_client_id_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Queue$dequeue(void );
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$granted(
# 51 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x2b48017dee30);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask$postTask(void );
# 69 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
enum /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4308 {
#line 69
  SimpleArbiterP$0$grantedTask = 10U
};
#line 69
typedef int /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$__nesc_sillytask_grantedTask[/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask];
#line 62
enum /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4309 {
#line 62
  SimpleArbiterP$0$RES_IDLE = 0, SimpleArbiterP$0$RES_GRANTING = 1, SimpleArbiterP$0$RES_BUSY = 2
};
#line 63
enum /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4310 {
#line 63
  SimpleArbiterP$0$NO_RES = 0xFF
};
uint8_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$state = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$RES_IDLE;
uint8_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$NO_RES;
uint8_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$reqResId;



static inline error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$request(uint8_t id);
#line 84
static inline error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$immediateRequest(uint8_t id);
#line 97
static inline error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$release(uint8_t id);
#line 148
static bool /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$isOwner(uint8_t id);






static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void );









static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$default$granted(uint8_t id);

static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(uint8_t id);

static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$immediateRequested(uint8_t id);

static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(uint8_t id);

static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id);
# 82 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$sendDone(
# 58 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSpiPioM.nc"
uint8_t arg_0x2b4801861448, 
# 75 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
uint8_t * txBuf, 
uint8_t * rxBuf, 





uint16_t len, 
error_t error);
# 43 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSSR(uint32_t val);








static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSTO(uint32_t val);
#line 49
static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSDR(uint32_t val);
#line 37
static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR0(uint32_t val);






static uint32_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSSR(void );





static uint32_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSDR(void );
#line 40
static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR1(uint32_t val);
# 67 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSpiPioM.nc"
enum /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$__nesc_unnamed4311 {
  HalPXA27xSpiPioM$0$FLAGS_SSCR0 = (((1 & 0xFFF) << 8) | ((0 & 0x3) << 4)) | ((7U & 0xF) << 0), 
  HalPXA27xSpiPioM$0$FLAGS_SSCR1 = 0
};


unsigned long long /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txBitBucket;
#line 73
unsigned long long /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxBitBucket;
uint8_t */*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txCurrentBuf;
#line 74
uint8_t */*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxCurrentBuf;
#line 74
uint8_t */*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txPtr;
#line 74
uint8_t */*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxPtr;
uint8_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txInc;
#line 75
uint8_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxInc;
uint8_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$instanceCurrent;
uint32_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenCurrent;
#line 77
uint32_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenRemain;

static inline error_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$Init$init(void );
#line 95
static uint8_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiByte$write(uint8_t tx);
#line 111
static error_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$send(uint8_t instance, uint8_t *txBuf, uint8_t *rxBuf, uint16_t len);
#line 169
static inline void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$interruptSSP(void );
#line 208
static inline void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$default$sendDone(uint8_t instance, uint8_t *txBuf, uint8_t *rxBuf, 
uint16_t len, error_t error);
# 176 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSSPControlP.nc"
static inline void HalPXA27xSSPControlP$SSP$interruptSSP(void );
# 134 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void IM2CC2420InitSpiP$RXD$setGAFRpin(uint8_t func);
#line 52
static void IM2CC2420InitSpiP$RXD$setGPDRbit(bool dir);
#line 134
static void IM2CC2420InitSpiP$SCLK$setGAFRpin(uint8_t func);
#line 52
static void IM2CC2420InitSpiP$SCLK$setGPDRbit(bool dir);
#line 134
static void IM2CC2420InitSpiP$TXD$setGAFRpin(uint8_t func);
#line 52
static void IM2CC2420InitSpiP$TXD$setGPDRbit(bool dir);
# 48 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/cc2420/IM2CC2420InitSpiP.nc"
static inline error_t IM2CC2420InitSpiP$Init$init(void );









static inline void IM2CC2420InitSpiP$SCLK$interruptGPIOPin(void );
static inline void IM2CC2420InitSpiP$TXD$interruptGPIOPin(void );
static inline void IM2CC2420InitSpiP$RXD$interruptGPIOPin(void );
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xSSPP$SSP2Irq$enable(void );
#line 65
static void HplPXA27xSSPP$SSP3Irq$enable(void );
# 70 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
static void HplPXA27xSSPP$HplPXA27xSSP$interruptSSP(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
uint8_t arg_0x2b48018f7cd8);
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xSSPP$SSP1Irq$enable(void );
# 52 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
static inline error_t HplPXA27xSSPP$Init$init(uint8_t chnl);
#line 77
static void HplPXA27xSSPP$HplPXA27xSSP$setSSCR0(uint8_t chnl, uint32_t val);
#line 96
static void HplPXA27xSSPP$HplPXA27xSSP$setSSCR1(uint8_t chnl, uint32_t val);
#line 114
static void HplPXA27xSSPP$HplPXA27xSSP$setSSSR(uint8_t chnl, uint32_t val);








static uint32_t HplPXA27xSSPP$HplPXA27xSSP$getSSSR(uint8_t chnl);
#line 150
static void HplPXA27xSSPP$HplPXA27xSSP$setSSDR(uint8_t chnl, uint32_t val);








static uint32_t HplPXA27xSSPP$HplPXA27xSSP$getSSDR(uint8_t chnl);








static inline void HplPXA27xSSPP$HplPXA27xSSP$setSSTO(uint8_t chnl, uint32_t val);
#line 276
static inline void HplPXA27xSSPP$HplPXA27xSSP$default$interruptSSP(uint8_t chnl);
#line 288
static inline void HplPXA27xSSPP$SSP3Irq$fired(void );



static inline void HplPXA27xSSPP$SSP1Irq$default$enable(void );
static inline void HplPXA27xSSPP$SSP2Irq$default$enable(void );
# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xDMAM$DMAIrq$enable(void );
#line 60
static error_t HplPXA27xDMAM$DMAIrq$allocate(void );
# 91 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
static void HplPXA27xDMAM$HplPXA27xDMAChnl$interruptDMA(
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x2b480198d670);









static inline error_t HplPXA27xDMAM$Init$init(void );
#line 70
static inline uint32_t HplPXA27xDMAM$HplPXA27xDMACntl$getDINT(void );
#line 94
static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCSR(uint8_t chnl, uint32_t val);






static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCMD(uint8_t chnl, uint32_t val);








static inline void HplPXA27xDMAM$DMAIrq$fired(void );
#line 123
static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$default$interruptDMA(uint8_t chnl);
# 62 "/opt/tinyos-2.1.2/tos/system/ActiveMessageAddressC.nc"
am_addr_t ActiveMessageAddressC$addr = TOS_AM_ADDRESS;


am_group_t ActiveMessageAddressC$group = TOS_AM_GROUP;






static inline am_addr_t ActiveMessageAddressC$ActiveMessageAddress$amAddress(void );
#line 93
static inline am_group_t ActiveMessageAddressC$ActiveMessageAddress$amGroup(void );
#line 106
static am_addr_t ActiveMessageAddressC$amAddress(void );
# 43 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/LocalIeeeEui64C.nc"
static ieee_eui64_t LocalIeeeEui64C$LocalIeeeEui64$getId(void );
# 81 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420TransmitP$RadioBackoff$requestInitialBackoff(message_t * msg);






static void CC2420TransmitP$RadioBackoff$requestCongestionBackoff(message_t * msg);
# 70 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
static void CC2420TransmitP$PacketTimeStamp$clear(
#line 66
message_t * msg);
#line 78
static void CC2420TransmitP$PacketTimeStamp$set(
#line 73
message_t * msg, 




CC2420TransmitP$PacketTimeStamp$size_type value);
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP$STXONCCA$strobe(void );
# 54 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
static error_t CC2420TransmitP$CaptureSFD$captureFallingEdge(void );
#line 66
static void CC2420TransmitP$CaptureSFD$disable(void );
#line 53
static error_t CC2420TransmitP$CaptureSFD$captureRisingEdge(void );
# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
static CC2420TransmitP$BackoffTimer$size_type CC2420TransmitP$BackoffTimer$getNow(void );
#line 66
static void CC2420TransmitP$BackoffTimer$start(CC2420TransmitP$BackoffTimer$size_type dt);






static void CC2420TransmitP$BackoffTimer$stop(void );
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Ram.nc"
static cc2420_status_t CC2420TransmitP$TXFIFO_RAM$write(uint8_t offset, uint8_t * data, uint8_t length);
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
static cc2420_status_t CC2420TransmitP$TXCTRL$write(uint16_t data);
# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420TransmitP$CC2420Receive$sfd_dropped(void );
#line 49
static void CC2420TransmitP$CC2420Receive$sfd(uint32_t time);
# 73 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
static void CC2420TransmitP$Send$sendDone(message_t * p_msg, error_t error);
# 31 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
static void CC2420TransmitP$ChipSpiResource$abortRelease(void );







static error_t CC2420TransmitP$ChipSpiResource$attemptRelease(void );
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP$SFLUSHTX$strobe(void );
# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP$CSN$makeOutput(void );
#line 40
static void CC2420TransmitP$CSN$set(void );
static void CC2420TransmitP$CSN$clr(void );
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TransmitP$CC2420PacketBody$getHeader(message_t * msg);










static cc2420_metadata_t * CC2420TransmitP$CC2420PacketBody$getMetadata(message_t * msg);
# 58 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
static uint8_t CC2420TransmitP$PacketTimeSyncOffset$get(
#line 53
message_t * msg);
#line 50
static bool CC2420TransmitP$PacketTimeSyncOffset$isSet(
#line 46
message_t * msg);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420TransmitP$SpiResource$release(void );
#line 97
static error_t CC2420TransmitP$SpiResource$immediateRequest(void );
#line 88
static error_t CC2420TransmitP$SpiResource$request(void );
# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP$CCA$makeInput(void );
#line 43
static bool CC2420TransmitP$CCA$get(void );
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP$SNOP$strobe(void );
# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420TransmitP$SFD$makeInput(void );
#line 43
static bool CC2420TransmitP$SFD$get(void );
# 82 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static cc2420_status_t CC2420TransmitP$TXFIFO$write(uint8_t * data, uint8_t length);
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420TransmitP$STXON$strobe(void );
# 99 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
#line 89
typedef enum CC2420TransmitP$__nesc_unnamed4312 {
  CC2420TransmitP$S_STOPPED, 
  CC2420TransmitP$S_STARTED, 
  CC2420TransmitP$S_LOAD, 
  CC2420TransmitP$S_SAMPLE_CCA, 
  CC2420TransmitP$S_BEGIN_TRANSMIT, 
  CC2420TransmitP$S_SFD, 
  CC2420TransmitP$S_EFD, 
  CC2420TransmitP$S_ACK_WAIT, 
  CC2420TransmitP$S_CANCEL
} CC2420TransmitP$cc2420_transmit_state_t;





enum CC2420TransmitP$__nesc_unnamed4313 {
  CC2420TransmitP$CC2420_ABORT_PERIOD = 320
};
#line 120
message_t * CC2420TransmitP$m_msg;

bool CC2420TransmitP$m_cca;

uint8_t CC2420TransmitP$m_tx_power;

CC2420TransmitP$cc2420_transmit_state_t CC2420TransmitP$m_state = CC2420TransmitP$S_STOPPED;

bool CC2420TransmitP$m_receiving = FALSE;

uint16_t CC2420TransmitP$m_prev_time;


bool CC2420TransmitP$sfdHigh;


bool CC2420TransmitP$abortSpiRelease;


int8_t CC2420TransmitP$totalCcaChecks;


uint16_t CC2420TransmitP$myInitialBackoff;


uint16_t CC2420TransmitP$myCongestionBackoff;



static inline error_t CC2420TransmitP$send(message_t * p_msg, bool cca);

static void CC2420TransmitP$loadTXFIFO(void );
static void CC2420TransmitP$attemptSend(void );
static void CC2420TransmitP$congestionBackoff(void );
static error_t CC2420TransmitP$acquireSpiResource(void );
static inline error_t CC2420TransmitP$releaseSpiResource(void );
static void CC2420TransmitP$signalDone(error_t err);



static inline error_t CC2420TransmitP$Init$init(void );







static inline error_t CC2420TransmitP$StdControl$start(void );










static inline error_t CC2420TransmitP$StdControl$stop(void );
#line 192
static inline error_t CC2420TransmitP$Send$send(message_t * p_msg, bool useCca);
#line 243
static inline void CC2420TransmitP$RadioBackoff$setInitialBackoff(uint16_t backoffTime);







static inline void CC2420TransmitP$RadioBackoff$setCongestionBackoff(uint16_t backoffTime);







static __inline uint32_t CC2420TransmitP$getTime32(uint16_t captured_time);
#line 280
static inline void CC2420TransmitP$CaptureSFD$captured(uint16_t time);
#line 377
static inline void CC2420TransmitP$ChipSpiResource$releasing(void );
#line 389
static inline void CC2420TransmitP$CC2420Receive$receive(uint8_t type, message_t *ack_msg);
#line 416
static inline void CC2420TransmitP$SpiResource$granted(void );
#line 454
static inline void CC2420TransmitP$TXFIFO$writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);
#line 486
static inline void CC2420TransmitP$TXFIFO$readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error);










static void CC2420TransmitP$BackoffTimer$fired(void );
#line 547
static inline error_t CC2420TransmitP$send(message_t * p_msg, bool cca);
#line 743
static void CC2420TransmitP$attemptSend(void );
#line 788
static void CC2420TransmitP$congestionBackoff(void );






static error_t CC2420TransmitP$acquireSpiResource(void );







static inline error_t CC2420TransmitP$releaseSpiResource(void );
#line 825
static void CC2420TransmitP$loadTXFIFO(void );
#line 850
static void CC2420TransmitP$signalDone(error_t err);
# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP$FIFO$get(void );
# 93 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
static bool CC2420ReceiveP$CC2420Config$isAddressRecognitionEnabled(void );
#line 117
static bool CC2420ReceiveP$CC2420Config$isAutoAckEnabled(void );
#line 112
static bool CC2420ReceiveP$CC2420Config$isHwAutoAckDefault(void );
#line 66
static ieee_eui64_t CC2420ReceiveP$CC2420Config$getExtAddr(void );




static uint16_t CC2420ReceiveP$CC2420Config$getShortAddr(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t CC2420ReceiveP$receiveDone_task$postTask(void );
# 70 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
static void CC2420ReceiveP$PacketTimeStamp$clear(
#line 66
message_t * msg);
#line 78
static void CC2420ReceiveP$PacketTimeStamp$set(
#line 73
message_t * msg, 




CC2420ReceiveP$PacketTimeStamp$size_type value);
# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP$FIFOP$get(void );
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Receive.nc"
static void CC2420ReceiveP$CC2420Receive$receive(uint8_t type, message_t * message);
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP$SACK$strobe(void );
# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static void CC2420ReceiveP$CSN$set(void );
static void CC2420ReceiveP$CSN$clr(void );
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ReceiveP$CC2420PacketBody$getHeader(message_t * msg);










static cc2420_metadata_t * CC2420ReceiveP$CC2420PacketBody$getMetadata(message_t * msg);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ReceiveP$Receive$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420ReceiveP$SpiResource$release(void );
#line 97
static error_t CC2420ReceiveP$SpiResource$immediateRequest(void );
#line 88
static error_t CC2420ReceiveP$SpiResource$request(void );
#line 128
static bool CC2420ReceiveP$SpiResource$isOwner(void );
# 62 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
static error_t CC2420ReceiveP$RXFIFO$continueRead(uint8_t * data, uint8_t length);
#line 51
static cc2420_status_t CC2420ReceiveP$RXFIFO$beginRead(uint8_t * data, uint8_t length);
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
static error_t CC2420ReceiveP$InterruptFIFOP$disable(void );
#line 54
static error_t CC2420ReceiveP$InterruptFIFOP$enableFallingEdge(void );
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
static cc2420_status_t CC2420ReceiveP$SFLUSHRX$strobe(void );
# 148 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
enum CC2420ReceiveP$__nesc_unnamed4314 {
#line 148
  CC2420ReceiveP$receiveDone_task = 11U
};
#line 148
typedef int CC2420ReceiveP$__nesc_sillytask_receiveDone_task[CC2420ReceiveP$receiveDone_task];
#line 89
#line 81
typedef enum CC2420ReceiveP$__nesc_unnamed4315 {
  CC2420ReceiveP$S_STOPPED, 
  CC2420ReceiveP$S_STARTED, 
  CC2420ReceiveP$S_RX_LENGTH, 
  CC2420ReceiveP$S_RX_DEC, 
  CC2420ReceiveP$S_RX_DEC_WAIT, 
  CC2420ReceiveP$S_RX_FCF, 
  CC2420ReceiveP$S_RX_PAYLOAD
} CC2420ReceiveP$cc2420_receive_state_t;

enum CC2420ReceiveP$__nesc_unnamed4316 {
  CC2420ReceiveP$RXFIFO_SIZE = 128, 
  CC2420ReceiveP$TIMESTAMP_QUEUE_SIZE = 8, 
  CC2420ReceiveP$SACK_HEADER_LENGTH = 7
};

uint32_t CC2420ReceiveP$m_timestamp_queue[CC2420ReceiveP$TIMESTAMP_QUEUE_SIZE];

uint8_t CC2420ReceiveP$m_timestamp_head;

uint8_t CC2420ReceiveP$m_timestamp_size;





uint8_t CC2420ReceiveP$m_missed_packets;



bool CC2420ReceiveP$receivingPacket;


uint8_t CC2420ReceiveP$rxFrameLength;

uint8_t CC2420ReceiveP$m_bytes_left;

message_t * CC2420ReceiveP$m_p_rx_buf;

message_t CC2420ReceiveP$m_rx_buf;
#line 137
CC2420ReceiveP$cc2420_receive_state_t CC2420ReceiveP$m_state;



static void CC2420ReceiveP$reset_state(void );
static void CC2420ReceiveP$beginReceive(void );
static void CC2420ReceiveP$receive(void );
static void CC2420ReceiveP$waitForNextPacket(void );
static void CC2420ReceiveP$flush(void );
static inline bool CC2420ReceiveP$passesAddressCheck(message_t * msg);




static inline error_t CC2420ReceiveP$Init$init(void );





static inline error_t CC2420ReceiveP$StdControl$start(void );
#line 171
static inline error_t CC2420ReceiveP$StdControl$stop(void );
#line 186
static inline void CC2420ReceiveP$CC2420Receive$sfd(uint32_t time);








static inline void CC2420ReceiveP$CC2420Receive$sfd_dropped(void );
#line 212
static inline void CC2420ReceiveP$InterruptFIFOP$fired(void );
#line 513
static inline void CC2420ReceiveP$SpiResource$granted(void );
#line 530
static inline void CC2420ReceiveP$RXFIFO$readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error);
#line 668
static inline void CC2420ReceiveP$RXFIFO$writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error);







static inline void CC2420ReceiveP$receiveDone_task$runTask(void );
#line 709
static inline void CC2420ReceiveP$CC2420Config$syncDone(error_t error);






static void CC2420ReceiveP$beginReceive(void );
#line 733
static void CC2420ReceiveP$flush(void );
#line 759
static void CC2420ReceiveP$receive(void );









static void CC2420ReceiveP$waitForNextPacket(void );
#line 813
static void CC2420ReceiveP$reset_state(void );










static inline bool CC2420ReceiveP$passesAddressCheck(message_t *msg);
# 81 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline int CC2420PacketP$getAddressLength(int type);








static uint8_t * CC2420PacketP$getNetwork(message_t * msg);
#line 119
static inline uint8_t CC2420PacketP$CC2420Packet$getNetwork(message_t * p_msg);








static inline void CC2420PacketP$CC2420Packet$setNetwork(message_t * p_msg, uint8_t networkId);








static inline cc2420_header_t * CC2420PacketP$CC2420PacketBody$getHeader(message_t * msg);
#line 152
static inline cc2420_metadata_t *CC2420PacketP$CC2420PacketBody$getMetadata(message_t *msg);
#line 171
static void CC2420PacketP$PacketTimeStamp32khz$clear(message_t *msg);





static inline void CC2420PacketP$PacketTimeStamp32khz$set(message_t *msg, uint32_t value);
#line 210
static inline bool CC2420PacketP$PacketTimeSyncOffset$isSet(message_t *msg);








static inline uint8_t CC2420PacketP$PacketTimeSyncOffset$get(message_t *msg);
# 71 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOSMR(uint32_t val);
#line 85
static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOMCR(uint32_t val);
#line 112
static bool /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$clearOSSRbit(void );
#line 57
static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOSCR(uint32_t val);
#line 119
static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOIERbit(bool flag);
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
static error_t /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTInit$init(void );
# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$Counter$overflow(void );
# 56 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline error_t /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$Init$init(void );
#line 91
static inline void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$fired(void );
# 58 "/opt/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 52 "/opt/tinyos-2.1.2/tos/system/RandomMlcgC.nc"
uint32_t RandomMlcgC$seed;


static inline error_t RandomMlcgC$Init$init(void );
#line 69
static uint32_t RandomMlcgC$Random$rand32(void );
#line 89
static inline uint16_t RandomMlcgC$Random$rand16(void );
# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t UniqueSendP$SubSend$send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t UniqueSendP$SubSend$maxPayloadLength(void );
#line 100
static void UniqueSendP$Send$sendDone(
#line 96
message_t * msg, 



error_t error);
# 52 "/opt/tinyos-2.1.2/tos/interfaces/Random.nc"
static uint16_t UniqueSendP$Random$rand16(void );
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueSendP$CC2420PacketBody$getHeader(message_t * msg);
# 56 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
static void UniqueSendP$State$toIdle(void );
#line 45
static error_t UniqueSendP$State$requestState(uint8_t reqState);
# 54 "/opt/tinyos-2.1.2/tos/chips/cc2420/unique/UniqueSendP.nc"
uint8_t UniqueSendP$localSendId;

enum UniqueSendP$__nesc_unnamed4317 {
  UniqueSendP$S_IDLE, 
  UniqueSendP$S_SENDING
};


static inline error_t UniqueSendP$Init$init(void );
#line 75
static inline error_t UniqueSendP$Send$send(message_t *msg, uint8_t len);
#line 95
static inline uint8_t UniqueSendP$Send$maxPayloadLength(void );








static inline void UniqueSendP$SubSend$sendDone(message_t *msg, error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP$Receive$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * UniqueReceiveP$CC2420PacketBody$getHeader(message_t * msg);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



UniqueReceiveP$DuplicateReceive$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 59 "/opt/tinyos-2.1.2/tos/chips/cc2420/unique/UniqueReceiveP.nc"
#line 56
struct UniqueReceiveP$__nesc_unnamed4318 {
  uint16_t source;
  uint8_t dsn;
} UniqueReceiveP$receivedMessages[4];

uint8_t UniqueReceiveP$writeIndex = 0;


uint8_t UniqueReceiveP$recycleSourceElement;

enum UniqueReceiveP$__nesc_unnamed4319 {
  UniqueReceiveP$INVALID_ELEMENT = 0xFF
};


static inline error_t UniqueReceiveP$Init$init(void );









static inline bool UniqueReceiveP$hasSeen(uint16_t msgSource, uint8_t msgDsn);
static inline void UniqueReceiveP$insert(uint16_t msgSource, uint8_t msgDsn);
static inline uint16_t UniqueReceiveP$getSourceKey(message_t  *msg);


static inline message_t *UniqueReceiveP$SubReceive$receive(message_t *msg, void *payload, 
uint8_t len);
#line 112
static inline bool UniqueReceiveP$hasSeen(uint16_t msgSource, uint8_t msgDsn);
#line 138
static inline void UniqueReceiveP$insert(uint16_t msgSource, uint8_t msgDsn);
#line 165
static inline uint16_t UniqueReceiveP$getSourceKey(message_t * msg);
#line 192
static inline message_t *UniqueReceiveP$DuplicateReceive$default$receive(message_t *msg, void *payload, uint8_t len);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t CC2420TinyosNetworkP$SubSend$send(
#line 67
message_t * msg, 







uint8_t len);
#line 112
static uint8_t CC2420TinyosNetworkP$SubSend$maxPayloadLength(void );
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t CC2420TinyosNetworkP$grantTask$postTask(void );
# 77 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Packet.nc"
static void CC2420TinyosNetworkP$CC2420Packet$setNetwork(message_t * p_msg, uint8_t networkId);
#line 75
static uint8_t CC2420TinyosNetworkP$CC2420Packet$getNetwork(message_t * p_msg);
# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP$ActiveSend$sendDone(
#line 96
message_t * msg, 



error_t error);
# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
static error_t CC2420TinyosNetworkP$Queue$enqueue(resource_client_id_t id);
#line 53
static bool CC2420TinyosNetworkP$Queue$isEmpty(void );
#line 70
static resource_client_id_t CC2420TinyosNetworkP$Queue$dequeue(void );
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420TinyosNetworkP$CC2420PacketBody$getHeader(message_t * msg);










static cc2420_metadata_t * CC2420TinyosNetworkP$CC2420PacketBody$getMetadata(message_t * msg);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP$BareReceive$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static void CC2420TinyosNetworkP$Resource$granted(
# 46 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
uint8_t arg_0x2b4801d92280);
# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static void CC2420TinyosNetworkP$BareSend$sendDone(
#line 96
message_t * msg, 



error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420TinyosNetworkP$ActiveReceive$receive(
#line 71
message_t * msg, 
void * payload, 





uint8_t len);
# 180 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
enum CC2420TinyosNetworkP$__nesc_unnamed4320 {
#line 180
  CC2420TinyosNetworkP$grantTask = 12U
};
#line 180
typedef int CC2420TinyosNetworkP$__nesc_sillytask_grantTask[CC2420TinyosNetworkP$grantTask];
#line 68
enum CC2420TinyosNetworkP$__nesc_unnamed4321 {
  CC2420TinyosNetworkP$OWNER_NONE = 0xff, 
  CC2420TinyosNetworkP$TINYOS_N_NETWORKS = 1U
};




#line 73
enum CC2420TinyosNetworkP$__nesc_unnamed4322 {
  CC2420TinyosNetworkP$CLIENT_AM, 
  CC2420TinyosNetworkP$CLIENT_BARE
} CC2420TinyosNetworkP$m_busy_client;

uint8_t CC2420TinyosNetworkP$resource_owner = CC2420TinyosNetworkP$OWNER_NONE;
#line 78
uint8_t CC2420TinyosNetworkP$next_owner;

static error_t CC2420TinyosNetworkP$ActiveSend$send(message_t *msg, uint8_t len);









static inline uint8_t CC2420TinyosNetworkP$ActiveSend$maxPayloadLength(void );



static inline void *CC2420TinyosNetworkP$ActiveSend$getPayload(message_t *msg, uint8_t len);
#line 138
static inline void *CC2420TinyosNetworkP$BareSend$getPayload(message_t *msg, uint8_t len);









static inline void CC2420TinyosNetworkP$SubSend$sendDone(message_t *msg, error_t error);








static inline message_t *CC2420TinyosNetworkP$SubReceive$receive(message_t *msg, void *payload, uint8_t len);
#line 180
static inline void CC2420TinyosNetworkP$grantTask$runTask(void );
#line 199
static inline error_t CC2420TinyosNetworkP$Resource$request(uint8_t id);
#line 215
static inline error_t CC2420TinyosNetworkP$Resource$immediateRequest(uint8_t id);
#line 229
static inline error_t CC2420TinyosNetworkP$Resource$release(uint8_t id);
#line 241
static inline message_t *CC2420TinyosNetworkP$BareReceive$default$receive(message_t *msg, void *payload, uint8_t len);


static inline void CC2420TinyosNetworkP$BareSend$default$sendDone(message_t *msg, error_t error);








static inline void CC2420TinyosNetworkP$Resource$default$granted(uint8_t client);
# 49 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
enum /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$__nesc_unnamed4323 {
#line 49
  FcfsResourceQueueC$0$NO_ENTRY = 0xFF
};
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$resQ[1];
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY;
uint8_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY;

static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$Init$init(void );




static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void );



static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void );
#line 82
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$enqueue(resource_client_id_t id);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t CC2420ActiveMessageP$SubSend$send(
#line 67
message_t * msg, 







uint8_t len);
#line 125
static 
#line 123
void * 

CC2420ActiveMessageP$SubSend$getPayload(
#line 122
message_t * msg, 


uint8_t len);
#line 112
static uint8_t CC2420ActiveMessageP$SubSend$maxPayloadLength(void );
# 77 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
static uint16_t CC2420ActiveMessageP$CC2420Config$getPanAddr(void );
# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
static void CC2420ActiveMessageP$RadioBackoff$requestCca(
# 54 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e14d60, 
# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
#line 81
static void CC2420ActiveMessageP$RadioBackoff$requestInitialBackoff(
# 54 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e14d60, 
# 81 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);






static void CC2420ActiveMessageP$RadioBackoff$requestCongestionBackoff(
# 54 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e14d60, 
# 88 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
message_t * msg);
# 59 "/opt/tinyos-2.1.2/tos/interfaces/SendNotifier.nc"
static void CC2420ActiveMessageP$SendNotifier$aboutToSend(
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e14258, 
# 59 "/opt/tinyos-2.1.2/tos/interfaces/SendNotifier.nc"
am_addr_t dest, 
#line 57
message_t * msg);
# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void CC2420ActiveMessageP$AMSend$sendDone(
# 48 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e197d8, 
# 103 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP$Snoop$receive(
# 50 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e17480, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 50 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
static am_addr_t CC2420ActiveMessageP$ActiveMessageAddress$amAddress(void );
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
static cc2420_header_t * CC2420ActiveMessageP$CC2420PacketBody$getHeader(message_t * msg);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
static 
#line 74
message_t * 



CC2420ActiveMessageP$Receive$receive(
# 49 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
am_id_t arg_0x2b4801e18908, 
# 71 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
static error_t CC2420ActiveMessageP$RadioResource$release(void );
#line 97
static error_t CC2420ActiveMessageP$RadioResource$immediateRequest(void );
#line 88
static error_t CC2420ActiveMessageP$RadioResource$request(void );
# 71 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
uint16_t CC2420ActiveMessageP$pending_length;
message_t * CC2420ActiveMessageP$pending_message = (void *)0;

static void CC2420ActiveMessageP$RadioResource$granted(void );
#line 87
static error_t CC2420ActiveMessageP$AMSend$send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len);
#line 135
static inline am_addr_t CC2420ActiveMessageP$AMPacket$address(void );



static am_addr_t CC2420ActiveMessageP$AMPacket$destination(message_t *amsg);









static inline void CC2420ActiveMessageP$AMPacket$setDestination(message_t *amsg, am_addr_t addr);









static inline bool CC2420ActiveMessageP$AMPacket$isForMe(message_t *amsg);




static inline am_id_t CC2420ActiveMessageP$AMPacket$type(message_t *amsg);




static inline void CC2420ActiveMessageP$AMPacket$setType(message_t *amsg, am_id_t type);
#line 194
static inline uint8_t CC2420ActiveMessageP$Packet$payloadLength(message_t *msg);



static inline void CC2420ActiveMessageP$Packet$setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t CC2420ActiveMessageP$Packet$maxPayloadLength(void );



static inline void *CC2420ActiveMessageP$Packet$getPayload(message_t *msg, uint8_t len);





static inline void CC2420ActiveMessageP$SubSend$sendDone(message_t *msg, error_t result);






static inline message_t *CC2420ActiveMessageP$SubReceive$receive(message_t *msg, void *payload, uint8_t len);
#line 235
static inline void CC2420ActiveMessageP$CC2420Config$syncDone(error_t error);





static inline void CC2420ActiveMessageP$SubBackoff$requestInitialBackoff(message_t *msg);




static inline void CC2420ActiveMessageP$SubBackoff$requestCongestionBackoff(message_t *msg);



static inline void CC2420ActiveMessageP$SubBackoff$requestCca(message_t *msg);
#line 279
static inline message_t *CC2420ActiveMessageP$Receive$default$receive(am_id_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t *CC2420ActiveMessageP$Snoop$default$receive(am_id_t id, message_t *msg, void *payload, uint8_t len);







static inline void CC2420ActiveMessageP$SendNotifier$default$aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg);

static inline void CC2420ActiveMessageP$RadioBackoff$default$requestInitialBackoff(am_id_t id, 
message_t *msg);


static inline void CC2420ActiveMessageP$RadioBackoff$default$requestCongestionBackoff(am_id_t id, 
message_t *msg);


static inline void CC2420ActiveMessageP$RadioBackoff$default$requestCca(am_id_t id, 
message_t *msg);
# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static void /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$sendDone(
#line 103
message_t * msg, 






error_t error);
# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static error_t /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$send(
#line 67
message_t * msg, 







uint8_t len);
# 103 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
static void /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setDestination(
#line 99
message_t * amsg, 



am_addr_t addr);
#line 162
static void /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setType(
#line 158
message_t * amsg, 



am_id_t t);
# 53 "/opt/tinyos-2.1.2/tos/system/AMQueueEntryP.nc"
static inline error_t /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$sendDone(message_t *m, error_t err);
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$send(
# 48 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
am_id_t arg_0x2b4801ec0488, 
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 71
message_t * msg, 








uint8_t len);
# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$sendDone(
# 46 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
uint8_t arg_0x2b4801ec1318, 
# 96 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 78 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$payloadLength(
#line 74
message_t * msg);
#line 94
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$setPayloadLength(
#line 90
message_t * msg, 



uint8_t len);
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$postTask(void );
# 78 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$destination(
#line 74
message_t * amsg);
#line 147
static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$type(
#line 143
message_t * amsg);
# 126 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$__nesc_unnamed4324 {
#line 126
  AMQueueImplP$0$CancelTask = 13U
};
#line 126
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$__nesc_sillytask_CancelTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask];
#line 169
enum /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$__nesc_unnamed4325 {
#line 169
  AMQueueImplP$0$errorTask = 14U
};
#line 169
typedef int /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$__nesc_sillytask_errorTask[/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask];
#line 57
#line 55
typedef struct /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$__nesc_unnamed4326 {
  message_t * msg;
} /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue_entry_t;

uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current = 1;
/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue_entry_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[1];
uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$cancelMask[1 / 8 + 1];

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$tryToSend(void );

static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$nextPacket(void );
#line 90
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 126
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask$runTask(void );
#line 163
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$runTask(void );




static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$tryToSend(void );
#line 189
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$sendDone(am_id_t id, message_t *msg, error_t err);
#line 215
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$default$sendDone(uint8_t id, message_t *msg, error_t err);
# 106 "/opt/tinyos-2.1.2/tos/chips/pxa27x/pxa27xhardware.h"
__inline  __nesc_atomic_t __nesc_atomic_start(void )
{
  uint32_t result = 0;
  uint32_t temp = 0;

   __asm volatile (
  "mrs %0,CPSR\n\t"
  "orr %1,%2,%4\n\t"
  "msr CPSR_cf,%3" : 
  "=r"(result), "=r"(temp) : 
  "0"(result), "1"(temp), "i"(0x000000C0));

   __asm volatile ("" :  :  : "memory");
  return result;
}

__inline  void __nesc_atomic_end(__nesc_atomic_t oldState)
{
  uint32_t statusReg = 0;

   __asm volatile ("" :  :  : "memory");
  oldState &= 0x000000C0;
   __asm volatile (
  "mrs %0,CPSR\n\t"
  "bic %0, %1, %2\n\t"
  "orr %0, %1, %3\n\t"
  "msr CPSR_c, %1" : 
  "=r"(statusReg) : 
  "0"(statusReg), "i"(0x000000C0), "r"(oldState));


  return;
}

# 59 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
static inline uint32_t HplPXA27xInterruptM$getICHP(void )
#line 59
{
  uint32_t val;

   __asm volatile ("mrc p6,0,%0,c5,c0,0\n\t" : "=r"(val));
  return val;
}

# 55 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static inline void HplPXA27xOSTimerM$DispatchOSTInterrupt(uint8_t id)
{
  HplPXA27xOSTimerM$PXA27xOST$fired(id);
  return;
}

# 97 "/opt/tinyos-2.1.2/tos/chips/pxa27x/pxa27xhardware.h"
static __inline uint32_t _pxa27x_clzui(uint32_t i)
#line 97
{
  uint32_t count;

#line 99
   __asm volatile ("clz %0,%1" : "=r"(count) : "r"(i));
  return count;
}

# 216 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static inline void HplPXA27xOSTimerM$OST4_11Irq$fired(void )
{
  uint32_t statusReg;
  uint8_t chnl;

  statusReg = * (volatile uint32_t *)0x40A00014;
  statusReg &= ~((((1 << 3) | (1 << 2)) | (1 << 1)) | (1 << 0));

  while (statusReg) {
      chnl = 31 - _pxa27x_clzui(statusReg);
      HplPXA27xOSTimerM$DispatchOSTInterrupt(chnl);
      statusReg &= ~(1 << chnl);
    }

  return;
}

#line 211
static inline void HplPXA27xOSTimerM$OST3Irq$fired(void )
{
  HplPXA27xOSTimerM$DispatchOSTInterrupt(3);
}

#line 206
static inline void HplPXA27xOSTimerM$OST2Irq$fired(void )
{
  HplPXA27xOSTimerM$DispatchOSTInterrupt(2);
}

#line 201
static inline void HplPXA27xOSTimerM$OST1Irq$fired(void )
{
  HplPXA27xOSTimerM$DispatchOSTInterrupt(1);
}

#line 196
static inline void HplPXA27xOSTimerM$OST0Irq$fired(void )
{
  HplPXA27xOSTimerM$DispatchOSTInterrupt(0);
}

# 140 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setISAR(uint32_t val)
#line 140
{
  switch (1) {
      case 0: * (volatile uint32_t *)0x403016A0 = val;
#line 142
      break;
      case 1: * (volatile uint32_t *)0x40F001A0 = val;
#line 143
      break;
      default: break;
    }
  return;
}

# 51 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void PMICM$PI2C$setISAR(uint32_t val){
#line 51
  /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setISAR(val);
#line 51
}
#line 51
# 132 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getISR(void )
#line 132
{
  switch (1) {
      case 0: return * (volatile uint32_t *)0x40301698;
      case 1: return * (volatile uint32_t *)0x40F00198;
      default: return 0;
    }
}

# 49 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static uint32_t PMICM$PI2C$getISR(void ){
#line 49
  unsigned int __nesc_result;
#line 49

#line 49
  __nesc_result = /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getISR();
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 256 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline void PMICM$PI2C$interruptI2C(void )
#line 256
{
  uint32_t status;
#line 257
  uint32_t update = 0;

#line 258
  status = PMICM$PI2C$getISR();
  if (status & (1 << 6)) {
      update |= 1 << 6;
    }


  if (status & (1 << 10)) {
      update |= 1 << 10;
    }

  PMICM$PI2C$setISAR(update);
}

# 54 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$interruptI2C(void ){
#line 54
  PMICM$PI2C$interruptI2C();
#line 54
}
#line 54
# 157 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$fired(void )
#line 157
{

  /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$interruptI2C();
  return;
}

# 259 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline void HplPXA27xGPIOM$HplPXA27xGPIO$default$fired(void )
#line 259
{
  return;
}

# 132 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIO.nc"
inline static void HplPXA27xGPIOM$HplPXA27xGPIO$fired(void ){
#line 132
  HplPXA27xGPIOM$HplPXA27xGPIO$default$fired();
#line 132
}
#line 132
# 263 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline void HplPXA27xGPIOM$GPIOIrq$fired(void )
{

  uint32_t DetectReg;
  uint8_t pin;

  HplPXA27xGPIOM$HplPXA27xGPIO$fired();
  /* atomic removed: atomic calls only */

  DetectReg = * (volatile uint32_t *)0x40E00048 & ~((1 << 1) | (1 << 0));

  while (DetectReg) {
      pin = 31 - _pxa27x_clzui(DetectReg);
      HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(pin);
      DetectReg &= ~(1 << pin);
    }
  /* atomic removed: atomic calls only */
  DetectReg = * (volatile uint32_t *)0x40E0004C;

  while (DetectReg) {
      pin = 31 - _pxa27x_clzui(DetectReg);
      HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(pin + 32);
      DetectReg &= ~(1 << pin);
    }
  /* atomic removed: atomic calls only */
  DetectReg = * (volatile uint32_t *)0x40E00050;

  while (DetectReg) {
      pin = 31 - _pxa27x_clzui(DetectReg);
      HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(pin + 64);
      DetectReg &= ~(1 << pin);
    }
  /* atomic removed: atomic calls only */
  DetectReg = * (volatile uint32_t *)0x40E00148;

  while (DetectReg) {
      pin = 31 - _pxa27x_clzui(DetectReg);
      HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(pin + 96);
      DetectReg &= ~(1 << pin);
    }

  return;
}






static inline void HplPXA27xGPIOM$GPIOIrq1$fired(void )
{
  HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(1);
}

#line 307
static inline void HplPXA27xGPIOM$GPIOIrq0$fired(void )
{
  HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(0);
}

# 66 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP$BackoffTimer$start(CC2420TransmitP$BackoffTimer$size_type dt){
#line 66
  /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$start(dt);
#line 66
}
#line 66
# 292 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_leuint8(const void * source)
#line 292
{
  const uint8_t *base = source;

#line 294
  return base[0];
}

# 293 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$RadioBackoff$default$requestInitialBackoff(am_id_t id, 
message_t *msg)
#line 294
{
}

# 81 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP$RadioBackoff$requestInitialBackoff(am_id_t arg_0x2b4801e14d60, message_t * msg){
#line 81
    CC2420ActiveMessageP$RadioBackoff$default$requestInitialBackoff(arg_0x2b4801e14d60, msg);
#line 81
}
#line 81
# 241 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$SubBackoff$requestInitialBackoff(message_t *msg)
#line 241
{
  CC2420ActiveMessageP$RadioBackoff$requestInitialBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.nxdata), msg);
}

# 81 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP$RadioBackoff$requestInitialBackoff(message_t * msg){
#line 81
  CC2420ActiveMessageP$SubBackoff$requestInitialBackoff(msg);
#line 81
}
#line 81
# 89 "/opt/tinyos-2.1.2/tos/system/RandomMlcgC.nc"
static inline uint16_t RandomMlcgC$Random$rand16(void )
#line 89
{
  return (uint16_t )RandomMlcgC$Random$rand32();
}

# 52 "/opt/tinyos-2.1.2/tos/interfaces/Random.nc"
inline static uint16_t CC2420CsmaP$Random$rand16(void ){
#line 52
  unsigned short __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC$Random$rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 243 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP$RadioBackoff$setInitialBackoff(uint16_t backoffTime)
#line 243
{
  CC2420TransmitP$myInitialBackoff = backoffTime + 1;
}

# 60 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP$SubBackoff$setInitialBackoff(uint16_t backoffTime){
#line 60
  CC2420TransmitP$RadioBackoff$setInitialBackoff(backoffTime);
#line 60
}
#line 60
# 223 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP$SubBackoff$requestInitialBackoff(message_t *msg)
#line 223
{
  CC2420CsmaP$SubBackoff$setInitialBackoff(CC2420CsmaP$Random$rand16()
   % (0x1F * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP$RadioBackoff$requestInitialBackoff(msg);
}

# 81 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP$RadioBackoff$requestInitialBackoff(message_t * msg){
#line 81
  CC2420CsmaP$SubBackoff$requestInitialBackoff(msg);
#line 81
}
#line 81
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP$SpiResource$release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP$Resource$release(/*CC2420TransmitC.Spi*/CC2420SpiC$3$CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 803 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP$releaseSpiResource(void )
#line 803
{
  CC2420TransmitP$SpiResource$release();
  return SUCCESS;
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP$sendDone_task$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420CsmaP$sendDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 205 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP$CC2420Transmit$sendDone(message_t *p_msg, error_t err)
#line 205
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 206
    CC2420CsmaP$sendErr = err;
#line 206
    __nesc_atomic_end(__nesc_atomic); }
  CC2420CsmaP$sendDone_task$postTask();
}

# 73 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static void CC2420TransmitP$Send$sendDone(message_t * p_msg, error_t error){
#line 73
  CC2420CsmaP$CC2420Transmit$sendDone(p_msg, error);
#line 73
}
#line 73
# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP$CSN$set(void ){
#line 40
  HalPXA27xGeneralIOM$GeneralIO$set(39);
#line 40
}
#line 40
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP$SFLUSHTX$strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP$Strobe$strobe(CC2420_SFLUSHTX);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 41 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP$CSN$clr(void ){
#line 41
  HalPXA27xGeneralIOM$GeneralIO$clr(39);
#line 41
}
#line 41
# 454 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP$TXFIFO$writeDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 455
{

  CC2420TransmitP$CSN$set();
  if (CC2420TransmitP$m_state == CC2420TransmitP$S_CANCEL) {
      /* atomic removed: atomic calls only */
#line 459
      {
        CC2420TransmitP$CSN$clr();
        CC2420TransmitP$SFLUSHTX$strobe();
        CC2420TransmitP$CSN$set();
      }
      CC2420TransmitP$releaseSpiResource();
      CC2420TransmitP$m_state = CC2420TransmitP$S_STARTED;
      CC2420TransmitP$Send$sendDone(CC2420TransmitP$m_msg, ECANCEL);
    }
  else {
#line 468
    if (!CC2420TransmitP$m_cca) {
        /* atomic removed: atomic calls only */
#line 469
        {
          CC2420TransmitP$m_state = CC2420TransmitP$S_BEGIN_TRANSMIT;
        }
        CC2420TransmitP$attemptSend();
      }
    else {
        CC2420TransmitP$releaseSpiResource();
        /* atomic removed: atomic calls only */
#line 476
        {
          CC2420TransmitP$m_state = CC2420TransmitP$S_SAMPLE_CCA;
        }

        CC2420TransmitP$RadioBackoff$requestInitialBackoff(CC2420TransmitP$m_msg);
        CC2420TransmitP$BackoffTimer$start(CC2420TransmitP$myInitialBackoff);
      }
    }
}

# 668 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP$RXFIFO$writeDone(uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 668
{
}

# 373 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP$Fifo$default$writeDone(uint8_t addr, uint8_t *tx_buf, uint8_t tx_len, error_t error)
#line 373
{
}

# 91 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP$Fifo$writeDone(uint8_t arg_0x2b480173f328, uint8_t * data, uint8_t length, error_t error){
#line 91
  switch (arg_0x2b480173f328) {
#line 91
    case CC2420_TXFIFO:
#line 91
      CC2420TransmitP$TXFIFO$writeDone(data, length, error);
#line 91
      break;
#line 91
    case CC2420_RXFIFO:
#line 91
      CC2420ReceiveP$RXFIFO$writeDone(data, length, error);
#line 91
      break;
#line 91
    default:
#line 91
      CC2420SpiP$Fifo$default$writeDone(arg_0x2b480173f328, data, length, error);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 486 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP$TXFIFO$readDone(uint8_t *tx_buf, uint8_t tx_len, 
error_t error)
#line 487
{
}

# 322 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_leuint16(const void * source)
#line 322
{
  const uint8_t *base = source;

#line 324
  return ((uint16_t )base[1] << 8) | base[0];
}

# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420ReceiveP$SpiResource$release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP$Resource$release(/*CC2420ReceiveC.Spi*/CC2420SpiC$4$CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP$CSN$set(void ){
#line 40
  HalPXA27xGeneralIOM$GeneralIO$set(39);
#line 40
}
#line 40
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ReceiveP$receiveDone_task$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420ReceiveP$receiveDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 286 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 286
{
  uint8_t *base = target;

#line 288
  base[0] = value;
  return value;
}

#line 303
static __inline  int8_t __nesc_hton_int8(void * target, int8_t value)
#line 303
{
#line 303
  __nesc_hton_uint8(target, value);
#line 303
  return value;
}

# 152 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline cc2420_metadata_t *CC2420PacketP$CC2420PacketBody$getMetadata(message_t *msg)
#line 152
{
  return (cc2420_metadata_t *)msg->metadata;
}

# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TransmitP$CC2420PacketBody$getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP$CC2420PacketBody$getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 119 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOIERbit(bool flag){
#line 119
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(6, flag);
#line 119
}
#line 119
# 117 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$stop(void )
#line 117
{
  /* atomic removed: atomic calls only */
#line 118
  {
    /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOIERbit(FALSE);
    /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mfRunning = FALSE;
  }
  return;
}

# 73 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void CC2420TransmitP$BackoffTimer$stop(void ){
#line 73
  /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$stop();
#line 73
}
#line 73
# 137 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline cc2420_header_t * CC2420PacketP$CC2420PacketBody$getHeader(message_t * msg)
#line 137
{
  return (cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
}

# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TransmitP$CC2420PacketBody$getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP$CC2420PacketBody$getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 389 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP$CC2420Receive$receive(uint8_t type, message_t *ack_msg)
#line 389
{
  cc2420_header_t *ack_header;
  cc2420_header_t *msg_header;
  cc2420_metadata_t *msg_metadata;
  uint8_t *ack_buf;
  uint8_t length;

  if (type == IEEE154_TYPE_ACK && CC2420TransmitP$m_msg) {
      ack_header = CC2420TransmitP$CC2420PacketBody$getHeader(ack_msg);
      msg_header = CC2420TransmitP$CC2420PacketBody$getHeader(CC2420TransmitP$m_msg);

      if (CC2420TransmitP$m_state == CC2420TransmitP$S_ACK_WAIT && __nesc_ntoh_leuint8(msg_header->dsn.nxdata) == __nesc_ntoh_leuint8(ack_header->dsn.nxdata)) {
          CC2420TransmitP$BackoffTimer$stop();

          msg_metadata = CC2420TransmitP$CC2420PacketBody$getMetadata(CC2420TransmitP$m_msg);
          ack_buf = (uint8_t *)ack_header;
          length = __nesc_ntoh_leuint8(ack_header->length.nxdata);

          __nesc_hton_int8(msg_metadata->ack.nxdata, TRUE);
          __nesc_hton_uint8(msg_metadata->rssi.nxdata, ack_buf[length - 1]);
          __nesc_hton_uint8(msg_metadata->lqi.nxdata, ack_buf[length] & 0x7f);
          CC2420TransmitP$signalDone(SUCCESS);
        }
    }
}

# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420ReceiveP$CC2420Receive$receive(uint8_t type, message_t * message){
#line 63
  CC2420TransmitP$CC2420Receive$receive(type, message);
#line 63
}
#line 63
# 70 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420ReceiveP$PacketTimeStamp$clear(message_t * msg){
#line 70
  CC2420PacketP$PacketTimeStamp32khz$clear(msg);
#line 70
}
#line 70
# 347 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint32_t __nesc_hton_uint32(void * target, uint32_t value)
#line 347
{
  uint8_t *base = target;

#line 349
  base[3] = value;
  base[2] = value >> 8;
  base[1] = value >> 16;
  base[0] = value >> 24;
  return value;
}

# 177 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline void CC2420PacketP$PacketTimeStamp32khz$set(message_t *msg, uint32_t value)
{
  __nesc_hton_uint32(CC2420PacketP$CC2420PacketBody$getMetadata(msg)->timestamp.nxdata, value);
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420ReceiveP$PacketTimeStamp$set(message_t * msg, CC2420ReceiveP$PacketTimeStamp$size_type value){
#line 78
  CC2420PacketP$PacketTimeStamp32khz$set(msg, value);
#line 78
}
#line 78
# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static bool HalPXA27xGeneralIOM$HplPXA27xGPIOPin$getGPLRbit(uint8_t arg_0x2b48014b4d00){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = HplPXA27xGPIOM$HplPXA27xGPIOPin$getGPLRbit(arg_0x2b48014b4d00);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 77 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline bool HalPXA27xGeneralIOM$GeneralIO$get(uint8_t pin)
#line 77
{
  bool result;

#line 79
  result = HalPXA27xGeneralIOM$HplPXA27xGPIOPin$getGPLRbit(pin);
  return result;
}

# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool CC2420ReceiveP$FIFO$get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = HalPXA27xGeneralIOM$GeneralIO$get(114);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 70 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
inline static error_t CC2420SpiP$SpiPacket$send(uint8_t * txBuf, uint8_t * rxBuf, uint16_t len){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$send(/*CC2420SpiWireC.HplCC2420SpiC*/HplCC2420SpiC$0$SPI_CLIENT_ID, txBuf, rxBuf, len);
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 209 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP$Fifo$continueRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 210
{
  return CC2420SpiP$SpiPacket$send((void *)0, data, len);
}

# 62 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static error_t CC2420ReceiveP$RXFIFO$continueRead(uint8_t * data, uint8_t length){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = CC2420SpiP$Fifo$continueRead(CC2420_RXFIFO, data, length);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
#line 51
inline static cc2420_status_t CC2420ReceiveP$RXFIFO$beginRead(uint8_t * data, uint8_t length){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420SpiP$Fifo$beginRead(CC2420_RXFIFO, data, length);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 41 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420ReceiveP$CSN$clr(void ){
#line 41
  HalPXA27xGeneralIOM$GeneralIO$clr(39);
#line 41
}
#line 41
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP$SACK$strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP$Strobe$strobe(CC2420_SACK);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 71 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420ReceiveP$CC2420Config$getShortAddr(void ){
#line 71
  unsigned short __nesc_result;
#line 71

#line 71
  __nesc_result = CC2420ControlP$CC2420Config$getShortAddr();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 382 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP$CC2420Config$isHwAutoAckDefault(void )
#line 382
{
  /* atomic removed: atomic calls only */
#line 383
  {
    unsigned char __nesc_temp = 
#line 383
    CC2420ControlP$hwAutoAckDefault;

#line 383
    return __nesc_temp;
  }
}

# 112 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP$CC2420Config$isHwAutoAckDefault(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = CC2420ControlP$CC2420Config$isHwAutoAckDefault();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 389 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP$CC2420Config$isAutoAckEnabled(void )
#line 389
{
  /* atomic removed: atomic calls only */
#line 390
  {
    unsigned char __nesc_temp = 
#line 390
    CC2420ControlP$autoAckEnabled;

#line 390
    return __nesc_temp;
  }
}

# 117 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP$CC2420Config$isAutoAckEnabled(void ){
#line 117
  unsigned char __nesc_result;
#line 117

#line 117
  __nesc_result = CC2420ControlP$CC2420Config$isAutoAckEnabled();
#line 117

#line 117
  return __nesc_result;
#line 117
}
#line 117
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ReceiveP$CC2420PacketBody$getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP$CC2420PacketBody$getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 530 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP$RXFIFO$readDone(uint8_t *rx_buf, uint8_t rx_len, 
error_t error)
#line 531
{
  cc2420_header_t *header = CC2420ReceiveP$CC2420PacketBody$getHeader(CC2420ReceiveP$m_p_rx_buf);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

#line 535
  CC2420ReceiveP$rxFrameLength = buf[0];

  switch (CC2420ReceiveP$m_state) {

      case CC2420ReceiveP$S_RX_LENGTH: 
        CC2420ReceiveP$m_state = CC2420ReceiveP$S_RX_FCF;



      if (CC2420ReceiveP$rxFrameLength + 1 > CC2420ReceiveP$m_bytes_left) 



        {

          CC2420ReceiveP$flush();
        }
      else {
          if (!CC2420ReceiveP$FIFO$get() && !CC2420ReceiveP$FIFOP$get()) {
              CC2420ReceiveP$m_bytes_left -= CC2420ReceiveP$rxFrameLength + 1;
            }

          if (CC2420ReceiveP$rxFrameLength <= MAC_PACKET_SIZE) {
              if (CC2420ReceiveP$rxFrameLength > 0) {
                  if (CC2420ReceiveP$rxFrameLength > CC2420ReceiveP$SACK_HEADER_LENGTH) {

                      CC2420ReceiveP$RXFIFO$continueRead(buf + 1, CC2420ReceiveP$SACK_HEADER_LENGTH);
                    }
                  else {

                      CC2420ReceiveP$m_state = CC2420ReceiveP$S_RX_PAYLOAD;
                      CC2420ReceiveP$RXFIFO$continueRead(buf + 1, CC2420ReceiveP$rxFrameLength);
                    }
                }
              else {
                  /* atomic removed: atomic calls only */
                  CC2420ReceiveP$receivingPacket = FALSE;
                  CC2420ReceiveP$CSN$set();
                  CC2420ReceiveP$SpiResource$release();
                  CC2420ReceiveP$waitForNextPacket();
                }
            }
          else {

              CC2420ReceiveP$flush();
            }
        }
      break;

      case CC2420ReceiveP$S_RX_FCF: 
        CC2420ReceiveP$m_state = CC2420ReceiveP$S_RX_PAYLOAD;










      if (CC2420ReceiveP$CC2420Config$isAutoAckEnabled() && !CC2420ReceiveP$CC2420Config$isHwAutoAckDefault()) {



          if (((__nesc_ntoh_leuint16(
#line 597
          header->fcf.nxdata) >> IEEE154_FCF_ACK_REQ) & 0x01) == 1
           && (__nesc_ntoh_leuint16(header->dest.nxdata) == CC2420ReceiveP$CC2420Config$getShortAddr()
           || __nesc_ntoh_leuint16(header->dest.nxdata) == AM_BROADCAST_ADDR)
           && ((__nesc_ntoh_leuint16(header->fcf.nxdata) >> IEEE154_FCF_FRAME_TYPE) & 7) == IEEE154_TYPE_DATA) {

              CC2420ReceiveP$CSN$set();
              CC2420ReceiveP$CSN$clr();
              CC2420ReceiveP$SACK$strobe();
              CC2420ReceiveP$CSN$set();
              CC2420ReceiveP$CSN$clr();
              CC2420ReceiveP$RXFIFO$beginRead(buf + 1 + CC2420ReceiveP$SACK_HEADER_LENGTH, 
              CC2420ReceiveP$rxFrameLength - CC2420ReceiveP$SACK_HEADER_LENGTH);
              return;
            }
        }

      CC2420ReceiveP$RXFIFO$continueRead(buf + 1 + CC2420ReceiveP$SACK_HEADER_LENGTH, 
      CC2420ReceiveP$rxFrameLength - CC2420ReceiveP$SACK_HEADER_LENGTH);
      break;

      case CC2420ReceiveP$S_RX_PAYLOAD: 

        CC2420ReceiveP$CSN$set();
      if (!CC2420ReceiveP$m_missed_packets) {

          CC2420ReceiveP$SpiResource$release();
        }




      if ((((
#line 626
      CC2420ReceiveP$m_missed_packets && CC2420ReceiveP$FIFO$get()) || !CC2420ReceiveP$FIFOP$get())
       || !CC2420ReceiveP$m_timestamp_size)
       || CC2420ReceiveP$rxFrameLength <= 10) {
          CC2420ReceiveP$PacketTimeStamp$clear(CC2420ReceiveP$m_p_rx_buf);
        }
      else {
          if (CC2420ReceiveP$m_timestamp_size == 1) {
            CC2420ReceiveP$PacketTimeStamp$set(CC2420ReceiveP$m_p_rx_buf, CC2420ReceiveP$m_timestamp_queue[CC2420ReceiveP$m_timestamp_head]);
            }
#line 634
          CC2420ReceiveP$m_timestamp_head = (CC2420ReceiveP$m_timestamp_head + 1) % CC2420ReceiveP$TIMESTAMP_QUEUE_SIZE;
          CC2420ReceiveP$m_timestamp_size--;

          if (CC2420ReceiveP$m_timestamp_size > 0) {
              CC2420ReceiveP$PacketTimeStamp$clear(CC2420ReceiveP$m_p_rx_buf);
              CC2420ReceiveP$m_timestamp_head = 0;
              CC2420ReceiveP$m_timestamp_size = 0;
            }
        }



      if (buf[CC2420ReceiveP$rxFrameLength] >> 7 && rx_buf) {
          uint8_t type = (__nesc_ntoh_leuint16(header->fcf.nxdata) >> IEEE154_FCF_FRAME_TYPE) & 7;

#line 648
          CC2420ReceiveP$CC2420Receive$receive(type, CC2420ReceiveP$m_p_rx_buf);
          if (type == IEEE154_TYPE_DATA) {
              CC2420ReceiveP$receiveDone_task$postTask();
              return;
            }
        }

      CC2420ReceiveP$waitForNextPacket();
      break;

      default: /* atomic removed: atomic calls only */
        CC2420ReceiveP$receivingPacket = FALSE;
      CC2420ReceiveP$CSN$set();
      CC2420ReceiveP$SpiResource$release();
      break;
    }
}

# 370 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP$Fifo$default$readDone(uint8_t addr, uint8_t *rx_buf, uint8_t rx_len, error_t error)
#line 370
{
}

# 71 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static void CC2420SpiP$Fifo$readDone(uint8_t arg_0x2b480173f328, uint8_t * data, uint8_t length, error_t error){
#line 71
  switch (arg_0x2b480173f328) {
#line 71
    case CC2420_TXFIFO:
#line 71
      CC2420TransmitP$TXFIFO$readDone(data, length, error);
#line 71
      break;
#line 71
    case CC2420_RXFIFO:
#line 71
      CC2420ReceiveP$RXFIFO$readDone(data, length, error);
#line 71
      break;
#line 71
    default:
#line 71
      CC2420SpiP$Fifo$default$readDone(arg_0x2b480173f328, data, length, error);
#line 71
      break;
#line 71
    }
#line 71
}
#line 71
# 329 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP$SpiPacket$sendDone(uint8_t *tx_buf, uint8_t *rx_buf, 
uint16_t len, error_t error)
#line 330
{
  if (CC2420SpiP$m_addr & 0x40) {
      CC2420SpiP$Fifo$readDone(CC2420SpiP$m_addr & ~0x40, rx_buf, len, error);
    }
  else 
#line 333
    {
      CC2420SpiP$Fifo$writeDone(CC2420SpiP$m_addr, tx_buf, len, error);
    }
}

# 208 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSpiPioM.nc"
static inline void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$default$sendDone(uint8_t instance, uint8_t *txBuf, uint8_t *rxBuf, 
uint16_t len, error_t error)
#line 209
{
  return;
}

# 82 "/opt/tinyos-2.1.2/tos/interfaces/SpiPacket.nc"
inline static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$sendDone(uint8_t arg_0x2b4801861448, uint8_t * txBuf, uint8_t * rxBuf, uint16_t len, error_t error){
#line 82
  switch (arg_0x2b4801861448) {
#line 82
    case /*CC2420SpiWireC.HplCC2420SpiC*/HplCC2420SpiC$0$SPI_CLIENT_ID:
#line 82
      CC2420SpiP$SpiPacket$sendDone(txBuf, rxBuf, len, error);
#line 82
      break;
#line 82
    default:
#line 82
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$default$sendDone(arg_0x2b4801861448, txBuf, rxBuf, len, error);
#line 82
      break;
#line 82
    }
#line 82
}
#line 82
# 40 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
inline static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR1(uint32_t val){
#line 40
  HplPXA27xSSPP$HplPXA27xSSP$setSSCR1(3, val);
#line 40
}
#line 40









inline static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSDR(uint32_t val){
#line 49
  HplPXA27xSSPP$HplPXA27xSSP$setSSDR(3, val);
#line 49
}
#line 49

inline static uint32_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSDR(void ){
#line 50
  unsigned int __nesc_result;
#line 50

#line 50
  __nesc_result = HplPXA27xSSPP$HplPXA27xSSP$getSSDR(3);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
#line 43
inline static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSSR(uint32_t val){
#line 43
  HplPXA27xSSPP$HplPXA27xSSP$setSSSR(3, val);
#line 43
}
#line 43

inline static uint32_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSSR(void ){
#line 44
  unsigned int __nesc_result;
#line 44

#line 44
  __nesc_result = HplPXA27xSSPP$HplPXA27xSSP$getSSSR(3);
#line 44

#line 44
  return __nesc_result;
#line 44
}
#line 44
# 169 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSpiPioM.nc"
static inline void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$interruptSSP(void )
#line 169
{
  uint32_t i;
#line 170
  uint32_t uiStatus;
#line 170
  uint32_t uiFifoLevel;
  uint32_t burst;

  uiStatus = /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSSR();
  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSSR(1 << 19);

  uiFifoLevel = (((uiStatus & (0xf << 12)) >> 12) | 0xF) + 1;
  uiFifoLevel = uiFifoLevel > /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenRemain ? /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenRemain : uiFifoLevel;

  if (!(uiStatus & (1 << 3))) {
    return;
    }
  for (i = 0; i < uiFifoLevel; i++) {
      */*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxPtr = /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSDR();
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxPtr += /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxInc;
    }
  /* atomic removed: atomic calls only */
  {
    /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenRemain -= uiFifoLevel;
    burst = /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenRemain < 16 ? /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenRemain : 16;
  }

  if (burst > 0) {
      for (i = 0; i < burst; i++) {
          /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSDR(*/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txPtr);
          /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txPtr += /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txInc;
        }
    }
  else {
      uint32_t len = /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenCurrent;

#line 200
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR1(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR1);
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenCurrent = 0;
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$sendDone(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$instanceCurrent, /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txCurrentBuf, /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxCurrentBuf, len, SUCCESS);
    }

  return;
}

# 176 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSSPControlP.nc"
static inline void HalPXA27xSSPControlP$SSP$interruptSSP(void )
#line 176
{
}

# 276 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
static inline void HplPXA27xSSPP$HplPXA27xSSP$default$interruptSSP(uint8_t chnl)
#line 276
{
  HplPXA27xSSPP$HplPXA27xSSP$setSSSR(chnl, (((((1 << 23) | (1 << 21)) | (1 << 20)) | (1 << 19)) | (
  1 << 18)) | (1 << 7));
  return;
}

# 70 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
inline static void HplPXA27xSSPP$HplPXA27xSSP$interruptSSP(uint8_t arg_0x2b48018f7cd8){
#line 70
  switch (arg_0x2b48018f7cd8) {
#line 70
    case 3:
#line 70
      HalPXA27xSSPControlP$SSP$interruptSSP();
#line 70
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$interruptSSP();
#line 70
      break;
#line 70
    default:
#line 70
      HplPXA27xSSPP$HplPXA27xSSP$default$interruptSSP(arg_0x2b48018f7cd8);
#line 70
      break;
#line 70
    }
#line 70
}
#line 70
# 288 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
static inline void HplPXA27xSSPP$SSP3Irq$fired(void )
#line 288
{
  HplPXA27xSSPP$HplPXA27xSSP$interruptSSP(3);
}

# 94 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCSR(uint8_t chnl, uint32_t val)
#line 94
{


  * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40000000 + (uint32_t )(chnl << 2)) = val;
}


static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCMD(uint8_t chnl, uint32_t val)
#line 101
{
#line 101
  * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x4000020C + (uint32_t )(chnl << 4)) = val;
}

#line 123
static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$default$interruptDMA(uint8_t chnl)
#line 123
{
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDCMD(chnl, 0);
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDCSR(chnl, (((1 << 9) | (1 << 2))
   | (1 << 1)) | (1 << 0));
}

# 91 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
inline static void HplPXA27xDMAM$HplPXA27xDMAChnl$interruptDMA(uint8_t arg_0x2b480198d670){
#line 91
    HplPXA27xDMAM$HplPXA27xDMAChnl$default$interruptDMA(arg_0x2b480198d670);
#line 91
}
#line 91
# 70 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static inline uint32_t HplPXA27xDMAM$HplPXA27xDMACntl$getDINT(void )
#line 70
{
#line 70
  return * (volatile uint32_t *)0x400000F0;
}

#line 110
static inline void HplPXA27xDMAM$DMAIrq$fired(void )
#line 110
{
  uint32_t IntReg;
  uint8_t chnl;

#line 113
  IntReg = HplPXA27xDMAM$HplPXA27xDMACntl$getDINT();

  while (IntReg) {
      chnl = 31 - _pxa27x_clzui(IntReg);
      HplPXA27xDMAM$HplPXA27xDMAChnl$interruptDMA(chnl);
      IntReg &= ~(1 << chnl);
    }
  return;
}

# 227 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
static inline void HplPXA27xInterruptM$PXA27xIrq$default$fired(uint8_t id)
{
  return;
}

# 75 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void HplPXA27xInterruptM$PXA27xIrq$fired(uint8_t arg_0x2b480120bd08){
#line 75
  switch (arg_0x2b480120bd08) {
#line 75
    case 0:
#line 75
      HplPXA27xSSPP$SSP3Irq$fired();
#line 75
      break;
#line 75
    case 6:
#line 75
      /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$fired();
#line 75
      break;
#line 75
    case 7:
#line 75
      HplPXA27xOSTimerM$OST4_11Irq$fired();
#line 75
      break;
#line 75
    case 8:
#line 75
      HplPXA27xGPIOM$GPIOIrq0$fired();
#line 75
      break;
#line 75
    case 9:
#line 75
      HplPXA27xGPIOM$GPIOIrq1$fired();
#line 75
      break;
#line 75
    case 10:
#line 75
      HplPXA27xGPIOM$GPIOIrq$fired();
#line 75
      break;
#line 75
    case 25:
#line 75
      HplPXA27xDMAM$DMAIrq$fired();
#line 75
      break;
#line 75
    case 26:
#line 75
      HplPXA27xOSTimerM$OST0Irq$fired();
#line 75
      break;
#line 75
    case 27:
#line 75
      HplPXA27xOSTimerM$OST1Irq$fired();
#line 75
      break;
#line 75
    case 28:
#line 75
      HplPXA27xOSTimerM$OST2Irq$fired();
#line 75
      break;
#line 75
    case 29:
#line 75
      HplPXA27xOSTimerM$OST3Irq$fired();
#line 75
      break;
#line 75
    default:
#line 75
      HplPXA27xInterruptM$PXA27xIrq$default$fired(arg_0x2b480120bd08);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 101 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPSRbit(uint8_t pin)
{
  *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00018 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00118) = 1 << (pin & 0x1f);
  return;
}

# 66 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPSRbit(uint8_t arg_0x2b48014b4d00){
#line 66
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPSRbit(arg_0x2b48014b4d00);
#line 66
}
#line 66
# 107 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPCRbit(uint8_t pin)
{
  *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00024 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00124) = 1 << (pin & 0x1f);
  return;
}

# 72 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPCRbit(uint8_t arg_0x2b48014b4d00){
#line 72
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPCRbit(arg_0x2b48014b4d00);
#line 72
}
#line 72
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ReceiveP$SFLUSHRX$strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP$Strobe$strobe(CC2420_SFLUSHRX);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 118 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
static inline void StateImplP$State$toIdle(uint8_t id)
#line 118
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 119
    StateImplP$state[id] = StateImplP$S_IDLE;
#line 119
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
inline static void CC2420SpiP$WorkingState$toIdle(void ){
#line 56
  StateImplP$State$toIdle(0U);
#line 56
}
#line 56
# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP$ChipSpiResource$abortRelease(void )
#line 95
{
  /* atomic removed: atomic calls only */
#line 96
  CC2420SpiP$release = FALSE;
}

# 31 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420TransmitP$ChipSpiResource$abortRelease(void ){
#line 31
  CC2420SpiP$ChipSpiResource$abortRelease();
#line 31
}
#line 31
# 377 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP$ChipSpiResource$releasing(void )
#line 377
{
  if (CC2420TransmitP$abortSpiRelease) {
      CC2420TransmitP$ChipSpiResource$abortRelease();
    }
}

# 24 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static void CC2420SpiP$ChipSpiResource$releasing(void ){
#line 24
  CC2420TransmitP$ChipSpiResource$releasing();
#line 24
}
#line 24
# 173 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id)
#line 173
{
}

# 65 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(uint8_t arg_0x2b4801808538){
#line 65
    /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(arg_0x2b4801808538);
#line 65
}
#line 65
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 68 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$dequeue(void )
#line 68
{
  /* atomic removed: atomic calls only */
#line 69
  {
    if (/*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qHead != /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY) {
        uint8_t id = /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qHead;

#line 72
        /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qHead = /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$resQ[/*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qHead];
        if (/*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qHead == /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY) {
          /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qTail = /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;
          }
#line 75
        /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$resQ[id] = /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 76
          id;

#line 76
          return __nesc_temp;
        }
      }
#line 78
    {
      unsigned char __nesc_temp = 
#line 78
      /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;

#line 78
      return __nesc_temp;
    }
  }
}

# 70 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Queue$dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 60 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEmpty(void )
#line 60
{
  /* atomic removed: atomic calls only */
#line 61
  {
    unsigned char __nesc_temp = 
#line 61
    /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qHead == /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY;

#line 61
    return __nesc_temp;
  }
}

# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static bool /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Queue$isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 97 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$release(uint8_t id)
#line 97
{
  bool released = FALSE;

  /* atomic removed: atomic calls only */
#line 99
  {
    if (/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$state == /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$RES_BUSY && /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId == id) {
        if (/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Queue$isEmpty() == FALSE) {
            /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$NO_RES;
            /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$reqResId = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Queue$dequeue();
            /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$state = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$RES_GRANTING;
            /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask$postTask();
          }
        else {
            /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$NO_RES;
            /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$state = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$RES_IDLE;
          }
        released = TRUE;
      }
  }
  if (released == TRUE) {
      /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP$SpiResource$release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$release(/*CC2420SpiWireC.HplCC2420SpiC*/HplCC2420SpiC$0$SPI_CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 97 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP$isWaiting(uint8_t id)
{
  return SchedulerBasicP$m_next[id] != SchedulerBasicP$NO_TASK || SchedulerBasicP$m_tail == id;
}

static inline bool SchedulerBasicP$pushTask(uint8_t id)
{
  if (!SchedulerBasicP$isWaiting(id)) 
    {
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_head = id;
          SchedulerBasicP$m_tail = id;
        }
      else 
        {
          SchedulerBasicP$m_next[SchedulerBasicP$m_tail] = id;
          SchedulerBasicP$m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 178 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline bool CC2420SpiP$Resource$isOwner(uint8_t id)
#line 178
{
  /* atomic removed: atomic calls only */
#line 179
  {
    unsigned char __nesc_temp = 
#line 179
    CC2420SpiP$m_holder == id;

#line 179
    return __nesc_temp;
  }
}

# 128 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static bool CC2420ReceiveP$SpiResource$isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = CC2420SpiP$Resource$isOwner(/*CC2420ReceiveC.Spi*/CC2420SpiC$4$CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
#line 97
inline static error_t CC2420ReceiveP$SpiResource$immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420SpiP$Resource$immediateRequest(/*CC2420ReceiveC.Spi*/CC2420SpiC$4$CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 45 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
inline static error_t CC2420SpiP$WorkingState$requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP$State$requestState(0U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 128 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static bool CC2420SpiP$SpiResource$isOwner(void ){
#line 128
  unsigned char __nesc_result;
#line 128

#line 128
  __nesc_result = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$isOwner(/*CC2420SpiWireC.HplCC2420SpiC*/HplCC2420SpiC$0$SPI_CLIENT_ID);
#line 128

#line 128
  return __nesc_result;
#line 128
}
#line 128
# 171 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(uint8_t id)
#line 171
{
}

# 59 "/opt/tinyos-2.1.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(uint8_t arg_0x2b4801808538){
#line 59
    /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(arg_0x2b4801808538);
#line 59
}
#line 59
# 169 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$immediateRequested(uint8_t id)
#line 169
{
}

# 61 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$immediateRequested(uint8_t arg_0x2b480180a020){
#line 61
    /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$immediateRequested(arg_0x2b480180a020);
#line 61
}
#line 61
# 84 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$immediateRequest(uint8_t id)
#line 84
{
  /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$immediateRequested(/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId);
  /* atomic removed: atomic calls only */
#line 86
  {
    if (/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$state == /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$RES_IDLE) {
        /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$state = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$RES_BUSY;
        /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId = id;
        /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId);
        {
          unsigned char __nesc_temp = 
#line 91
          SUCCESS;

#line 91
          return __nesc_temp;
        }
      }
#line 93
    {
      unsigned char __nesc_temp = 
#line 93
      FAIL;

#line 93
      return __nesc_temp;
    }
  }
}

# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP$SpiResource$immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$immediateRequest(/*CC2420SpiWireC.HplCC2420SpiC*/HplCC2420SpiC$0$SPI_CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
#line 88
inline static error_t CC2420ReceiveP$SpiResource$request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP$Resource$request(/*CC2420ReceiveC.Spi*/CC2420SpiC$4$CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 64 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$resQ[id] != /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY || /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$enqueue(resource_client_id_t id)
#line 82
{
  /* atomic removed: atomic calls only */
#line 83
  {
    if (!/*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$isEnqueued(id)) {
        if (/*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qHead == /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY) {
          /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qHead = id;
          }
        else {
#line 88
          /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$resQ[/*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qTail] = id;
          }
#line 89
        /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$qTail = id;
        {
          unsigned char __nesc_temp = 
#line 90
          SUCCESS;

#line 90
          return __nesc_temp;
        }
      }
#line 92
    {
      unsigned char __nesc_temp = 
#line 92
      EBUSY;

#line 92
      return __nesc_temp;
    }
  }
}

# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Queue$enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$FcfsQueue$enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 167 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(uint8_t id)
#line 167
{
}

# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceRequested.nc"
inline static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(uint8_t arg_0x2b480180a020){
#line 53
    /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(arg_0x2b480180a020);
#line 53
}
#line 53
# 71 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline error_t /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$request(uint8_t id)
#line 71
{
  /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId);
  /* atomic removed: atomic calls only */
#line 73
  {
    if (/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$state == /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$RES_IDLE) {
        /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$state = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$RES_GRANTING;
        /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$reqResId = id;
        /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask$postTask();
        {
          unsigned char __nesc_temp = 
#line 78
          SUCCESS;

#line 78
          return __nesc_temp;
        }
      }
#line 80
    {
      unsigned char __nesc_temp = 
#line 80
      /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Queue$enqueue(id);

#line 80
      return __nesc_temp;
    }
  }
}

# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420SpiP$SpiResource$request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$request(/*CC2420SpiWireC.HplCC2420SpiC*/HplCC2420SpiC$0$SPI_CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 102 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline error_t CC2420SpiP$ChipSpiResource$attemptRelease(void )
#line 102
{
  return CC2420SpiP$attemptRelease();
}

# 39 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/ChipSpiResource.nc"
inline static error_t CC2420TransmitP$ChipSpiResource$attemptRelease(void ){
#line 39
  unsigned char __nesc_result;
#line 39

#line 39
  __nesc_result = CC2420SpiP$ChipSpiResource$attemptRelease();
#line 39

#line 39
  return __nesc_result;
#line 39
}
#line 39
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420TransmitP$STXONCCA$strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP$Strobe$strobe(CC2420_STXONCCA);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420TransmitP$STXON$strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP$Strobe$strobe(CC2420_STXON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420TransmitP$SNOP$strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP$Strobe$strobe(CC2420_SNOP);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 297 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$RadioBackoff$default$requestCongestionBackoff(am_id_t id, 
message_t *msg)
#line 298
{
}

# 88 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP$RadioBackoff$requestCongestionBackoff(am_id_t arg_0x2b4801e14d60, message_t * msg){
#line 88
    CC2420ActiveMessageP$RadioBackoff$default$requestCongestionBackoff(arg_0x2b4801e14d60, msg);
#line 88
}
#line 88
# 246 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$SubBackoff$requestCongestionBackoff(message_t *msg)
#line 246
{
  CC2420ActiveMessageP$RadioBackoff$requestCongestionBackoff(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.nxdata), msg);
}

# 88 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP$RadioBackoff$requestCongestionBackoff(message_t * msg){
#line 88
  CC2420ActiveMessageP$SubBackoff$requestCongestionBackoff(msg);
#line 88
}
#line 88
# 251 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP$RadioBackoff$setCongestionBackoff(uint16_t backoffTime)
#line 251
{
  CC2420TransmitP$myCongestionBackoff = backoffTime + 1;
}

# 66 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP$SubBackoff$setCongestionBackoff(uint16_t backoffTime){
#line 66
  CC2420TransmitP$RadioBackoff$setCongestionBackoff(backoffTime);
#line 66
}
#line 66
# 230 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP$SubBackoff$requestCongestionBackoff(message_t *msg)
#line 230
{
  CC2420CsmaP$SubBackoff$setCongestionBackoff(CC2420CsmaP$Random$rand16()
   % (0x7 * CC2420_BACKOFF_PERIOD) + CC2420_MIN_BACKOFF);

  CC2420CsmaP$RadioBackoff$requestCongestionBackoff(msg);
}

# 88 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420TransmitP$RadioBackoff$requestCongestionBackoff(message_t * msg){
#line 88
  CC2420CsmaP$SubBackoff$requestCongestionBackoff(msg);
#line 88
}
#line 88
# 71 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOSMR(uint32_t val){
#line 71
  HplPXA27xOSTimerM$PXA27xOST$setOSMR(6, val);
#line 71
}
#line 71
#line 103
inline static bool /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$getOSSRbit(void ){
#line 103
  unsigned char __nesc_result;
#line 103

#line 103
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSSRbit(6);
#line 103

#line 103
  return __nesc_result;
#line 103
}
#line 103
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$lateAlarm$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$lateAlarm);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 340 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint32_t __nesc_ntoh_uint32(const void * source)
#line 340
{
  const uint8_t *base = source;

#line 342
  return ((((uint32_t )base[0] << 24) | (
  (uint32_t )base[1] << 16)) | (
  (uint32_t )base[2] << 8)) | base[3];
}

# 70 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP$PacketTimeStamp$clear(message_t * msg){
#line 70
  CC2420PacketP$PacketTimeStamp32khz$clear(msg);
#line 70
}
#line 70
# 195 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP$CC2420Receive$sfd_dropped(void )
#line 195
{
  if (CC2420ReceiveP$m_timestamp_size) {
      CC2420ReceiveP$m_timestamp_size--;
    }
}

# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP$CC2420Receive$sfd_dropped(void ){
#line 55
  CC2420ReceiveP$CC2420Receive$sfd_dropped();
#line 55
}
#line 55
# 138 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline error_t HalPXA27xGeneralIOM$GpioInterrupt$enableRisingEdge(uint8_t pin)
#line 138
{
  return HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableRisingEdge(pin);
}

# 53 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$enableRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = HalPXA27xGeneralIOM$GpioInterrupt$enableRisingEdge(16);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 59 "/opt/tinyos-2.1.2/tos/lib/gpio/SoftCaptureP.nc"
static inline error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captureRisingEdge(void )
#line 59
{
  return /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$enableRisingEdge();
}

# 53 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP$CaptureSFD$captureRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captureRisingEdge();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 186 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP$CC2420Receive$sfd(uint32_t time)
#line 186
{
  if (CC2420ReceiveP$m_timestamp_size < CC2420ReceiveP$TIMESTAMP_QUEUE_SIZE) {
      uint8_t tail = (CC2420ReceiveP$m_timestamp_head + CC2420ReceiveP$m_timestamp_size) % 
      CC2420ReceiveP$TIMESTAMP_QUEUE_SIZE;

#line 190
      CC2420ReceiveP$m_timestamp_queue[tail] = time;
      CC2420ReceiveP$m_timestamp_size++;
    }
}

# 49 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Receive.nc"
inline static void CC2420TransmitP$CC2420Receive$sfd(uint32_t time){
#line 49
  CC2420ReceiveP$CC2420Receive$sfd(time);
#line 49
}
#line 49
# 142 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline error_t HalPXA27xGeneralIOM$GpioInterrupt$enableFallingEdge(uint8_t pin)
#line 142
{
  return HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableFallingEdge(pin);
}

# 54 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$enableFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = HalPXA27xGeneralIOM$GpioInterrupt$enableFallingEdge(16);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 63 "/opt/tinyos-2.1.2/tos/lib/gpio/SoftCaptureP.nc"
static inline error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captureFallingEdge(void )
#line 63
{
  return /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$enableFallingEdge();
}

# 54 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static error_t CC2420TransmitP$CaptureSFD$captureFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captureFallingEdge();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420TransmitP$TXFIFO_RAM$write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP$Ram$write(CC2420_RAM_TXFIFO, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 219 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP$PacketTimeSyncOffset$get(message_t *msg)
{
  return __nesc_ntoh_leuint8(CC2420PacketP$CC2420PacketBody$getHeader(msg)->length.nxdata)
   + (sizeof(cc2420_header_t ) - MAC_HEADER_SIZE)
   - MAC_FOOTER_SIZE
   - sizeof(timesync_radio_t );
}

# 58 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static uint8_t CC2420TransmitP$PacketTimeSyncOffset$get(message_t * msg){
#line 58
  unsigned char __nesc_result;
#line 58

#line 58
  __nesc_result = CC2420PacketP$PacketTimeSyncOffset$get(msg);
#line 58

#line 58
  return __nesc_result;
#line 58
}
#line 58
# 281 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 281
{
  const uint8_t *base = source;

#line 283
  return base[0];
}

#line 303
static __inline  int8_t __nesc_ntoh_int8(const void * source)
#line 303
{
#line 303
  return __nesc_ntoh_uint8(source);
}

# 210 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline bool CC2420PacketP$PacketTimeSyncOffset$isSet(message_t *msg)
{
  return __nesc_ntoh_int8(CC2420PacketP$CC2420PacketBody$getMetadata(msg)->timesync.nxdata);
}

# 50 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/PacketTimeSyncOffset.nc"
inline static bool CC2420TransmitP$PacketTimeSyncOffset$isSet(message_t * msg){
#line 50
  unsigned char __nesc_result;
#line 50

#line 50
  __nesc_result = CC2420PacketP$PacketTimeSyncOffset$isSet(msg);
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 78 "/opt/tinyos-2.1.2/tos/interfaces/PacketTimeStamp.nc"
inline static void CC2420TransmitP$PacketTimeStamp$set(message_t * msg, CC2420TransmitP$PacketTimeStamp$size_type value){
#line 78
  CC2420PacketP$PacketTimeStamp32khz$set(msg, value);
#line 78
}
#line 78
# 64 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static uint32_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$getOSCR(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSCR(6);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 152 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline uint32_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$getNow(void )
#line 152
{
  return /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$getOSCR();
}

# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static CC2420TransmitP$BackoffTimer$size_type CC2420TransmitP$BackoffTimer$getNow(void ){
#line 109
  unsigned int __nesc_result;
#line 109

#line 109
  __nesc_result = /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 259 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static __inline uint32_t CC2420TransmitP$getTime32(uint16_t captured_time)
{
  uint32_t now = CC2420TransmitP$BackoffTimer$getNow();


  return now - (uint16_t )(now - captured_time);
}

#line 280
static inline void CC2420TransmitP$CaptureSFD$captured(uint16_t time)
#line 280
{
  unsigned char *__nesc_temp45;
  unsigned char *__nesc_temp44;
#line 281
  uint32_t time32;
  uint8_t sfd_state = 0;

  /* atomic removed: atomic calls only */
#line 283
  {
    time32 = CC2420TransmitP$getTime32(time);
    switch (CC2420TransmitP$m_state) {

        case CC2420TransmitP$S_SFD: 
          CC2420TransmitP$m_state = CC2420TransmitP$S_EFD;
        CC2420TransmitP$sfdHigh = TRUE;


        CC2420TransmitP$m_receiving = FALSE;
        CC2420TransmitP$CaptureSFD$captureFallingEdge();
        CC2420TransmitP$PacketTimeStamp$set(CC2420TransmitP$m_msg, time32);
        if (CC2420TransmitP$PacketTimeSyncOffset$isSet(CC2420TransmitP$m_msg)) {
            uint8_t absOffset = sizeof(message_header_t ) - sizeof(cc2420_header_t ) + CC2420TransmitP$PacketTimeSyncOffset$get(CC2420TransmitP$m_msg);
            timesync_radio_t *timesync = (timesync_radio_t *)((nx_uint8_t *)CC2420TransmitP$m_msg + absOffset);

            (__nesc_temp44 = (*timesync).nxdata, __nesc_hton_uint32(__nesc_temp44, __nesc_ntoh_uint32(__nesc_temp44) - time32));
            CC2420TransmitP$CSN$clr();
            CC2420TransmitP$TXFIFO_RAM$write(absOffset, (uint8_t *)timesync, sizeof(timesync_radio_t ));
            CC2420TransmitP$CSN$set();

            (__nesc_temp45 = (*timesync).nxdata, __nesc_hton_uint32(__nesc_temp45, __nesc_ntoh_uint32(__nesc_temp45) + time32));
          }

        if (__nesc_ntoh_leuint16(CC2420TransmitP$CC2420PacketBody$getHeader(CC2420TransmitP$m_msg)->fcf.nxdata) & (1 << IEEE154_FCF_ACK_REQ)) {

            CC2420TransmitP$abortSpiRelease = TRUE;
          }
        CC2420TransmitP$releaseSpiResource();
        CC2420TransmitP$BackoffTimer$stop();

        if (CC2420TransmitP$SFD$get()) {
            break;
          }


        case CC2420TransmitP$S_EFD: 
          CC2420TransmitP$sfdHigh = FALSE;
        CC2420TransmitP$CaptureSFD$captureRisingEdge();

        if (__nesc_ntoh_leuint16(CC2420TransmitP$CC2420PacketBody$getHeader(CC2420TransmitP$m_msg)->fcf.nxdata) & (1 << IEEE154_FCF_ACK_REQ)) {
            CC2420TransmitP$m_state = CC2420TransmitP$S_ACK_WAIT;
            CC2420TransmitP$BackoffTimer$start(CC2420_ACK_WAIT_DELAY);
          }
        else 
#line 326
          {
            CC2420TransmitP$signalDone(SUCCESS);
          }

        if (!CC2420TransmitP$SFD$get()) {
            break;
          }


        default: 

          if (!CC2420TransmitP$m_receiving && CC2420TransmitP$sfdHigh == FALSE) {
              CC2420TransmitP$sfdHigh = TRUE;
              CC2420TransmitP$CaptureSFD$captureFallingEdge();

              sfd_state = CC2420TransmitP$SFD$get();
              CC2420TransmitP$CC2420Receive$sfd(time32);
              CC2420TransmitP$m_receiving = TRUE;
              CC2420TransmitP$m_prev_time = time;
              if (CC2420TransmitP$SFD$get()) {

                  return;
                }
            }



        if (CC2420TransmitP$sfdHigh == TRUE) {
            CC2420TransmitP$sfdHigh = FALSE;
            CC2420TransmitP$CaptureSFD$captureRisingEdge();
            CC2420TransmitP$m_receiving = FALSE;








            if (sfd_state == 0 && time - CC2420TransmitP$m_prev_time < 10) {
                CC2420TransmitP$CC2420Receive$sfd_dropped();
                if (CC2420TransmitP$m_msg) {
                  CC2420TransmitP$PacketTimeStamp$clear(CC2420TransmitP$m_msg);
                  }
              }
#line 370
            break;
          }
      }
  }
}

# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captured(uint16_t time){
#line 61
  CC2420TransmitP$CaptureSFD$captured(time);
#line 61
}
#line 61
# 64 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static uint32_t /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$getOSCR(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSCR(7);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 72 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline uint32_t /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Counter$get(void )
#line 72
{
  uint32_t cntr;

  cntr = /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$getOSCR();
  return cntr;
}

# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$size_type /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$get(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Counter$get();
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 72 "/opt/tinyos-2.1.2/tos/lib/gpio/SoftCaptureP.nc"
static inline void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$fired(void )
#line 72
{
  uint16_t captureTime;

  captureTime = (uint16_t )/*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$get();
  /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$captured(captureTime);
  return;
}

# 212 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP$InterruptFIFOP$fired(void )
#line 212
{
  if (CC2420ReceiveP$m_state == CC2420ReceiveP$S_STARTED) {

      CC2420ReceiveP$m_state = CC2420ReceiveP$S_RX_LENGTH;
      CC2420ReceiveP$beginReceive();
    }
  else 



    {
      CC2420ReceiveP$m_missed_packets++;
    }
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP$startDone_task$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420CsmaP$startDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 218 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP$CC2420Power$startOscillatorDone(void )
#line 218
{
  CC2420CsmaP$startDone_task$postTask();
}

# 76 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP$CC2420Power$startOscillatorDone(void ){
#line 76
  CC2420CsmaP$CC2420Power$startOscillatorDone();
#line 76
}
#line 76
# 41 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP$CSN$clr(void ){
#line 41
  HalPXA27xGeneralIOM$GeneralIO$clr(39);
#line 41
}
#line 41
#line 40
inline static void CC2420ControlP$CSN$set(void ){
#line 40
  HalPXA27xGeneralIOM$GeneralIO$set(39);
#line 40
}
#line 40
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP$IOCFG1$write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP$Reg$write(CC2420_IOCFG1, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 146 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline error_t HalPXA27xGeneralIOM$GpioInterrupt$disable(uint8_t pin)
#line 146
{
  return HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$disable(pin);
}

# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP$InterruptCCA$disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = HalPXA27xGeneralIOM$GpioInterrupt$disable(116);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 441 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP$InterruptCCA$fired(void )
#line 441
{
  CC2420ControlP$m_state = CC2420ControlP$S_XOSC_STARTED;
  CC2420ControlP$InterruptCCA$disable();
  CC2420ControlP$IOCFG1$write(0);
  CC2420ControlP$writeId();
  CC2420ControlP$CSN$set();
  CC2420ControlP$CSN$clr();
  CC2420ControlP$CC2420Power$startOscillatorDone();
}

# 162 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline void HalPXA27xGeneralIOM$GpioInterrupt$default$fired(uint8_t pin)
#line 162
{
  return;
}

# 68 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static void HalPXA27xGeneralIOM$GpioInterrupt$fired(uint8_t arg_0x2b48014b5c60){
#line 68
  switch (arg_0x2b48014b5c60) {
#line 68
    case 0:
#line 68
      CC2420ReceiveP$InterruptFIFOP$fired();
#line 68
      break;
#line 68
    case 16:
#line 68
      /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$fired();
#line 68
      break;
#line 68
    case 116:
#line 68
      CC2420ControlP$InterruptCCA$fired();
#line 68
      break;
#line 68
    default:
#line 68
      HalPXA27xGeneralIOM$GpioInterrupt$default$fired(arg_0x2b48014b5c60);
#line 68
      break;
#line 68
    }
#line 68
}
#line 68
# 158 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline void HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$default$fired(uint8_t pin)
#line 158
{
  return;
}

# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
inline static void HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$fired(uint8_t arg_0x2b48014979a8){
#line 65
    HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$default$fired(arg_0x2b48014979a8);
#line 65
}
#line 65
# 124 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static bool HalPXA27xGeneralIOM$HplPXA27xGPIOPin$clearGEDRbit(uint8_t arg_0x2b48014b4d00){
#line 124
  unsigned char __nesc_result;
#line 124

#line 124
  __nesc_result = HplPXA27xGPIOM$HplPXA27xGPIOPin$clearGEDRbit(arg_0x2b48014b4d00);
#line 124

#line 124
  return __nesc_result;
#line 124
}
#line 124
# 150 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$interruptGPIOPin(uint8_t pin)
#line 150
{
  HalPXA27xGeneralIOM$HplPXA27xGPIOPin$clearGEDRbit(pin);
  HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$fired(pin);
  HalPXA27xGeneralIOM$GpioInterrupt$fired(pin);
  return;
}

# 83 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGRERbit(uint8_t arg_0x2b48014b4d00, bool flag){
#line 83
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGRERbit(arg_0x2b48014b4d00, flag);
#line 83
}
#line 83
#line 101
inline static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGFERbit(uint8_t arg_0x2b48014b4d00, bool flag){
#line 101
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGFERbit(arg_0x2b48014b4d00, flag);
#line 101
}
#line 101
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Ram.nc"
inline static cc2420_status_t CC2420ControlP$IEEEADR$write(uint8_t offset, uint8_t * data, uint8_t length){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP$Ram$write(CC2420_RAM_IEEEADR, offset, data, length);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 58 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/cc2420/IM2CC2420InitSpiP.nc"
static inline void IM2CC2420InitSpiP$SCLK$interruptGPIOPin(void )
#line 58
{
#line 58
  return;
}

#line 59
static inline void IM2CC2420InitSpiP$TXD$interruptGPIOPin(void )
#line 59
{
#line 59
  return;
}

#line 60
static inline void IM2CC2420InitSpiP$RXD$interruptGPIOPin(void )
#line 60
{
#line 60
  return;
}

# 186 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static inline void HplPXA27xOSTimerM$PXA27xWD$enableWatchdog(void )
{
  * (volatile uint32_t *)0x40A00018 = 1 << 0;
}

# 45 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerWatchdog.nc"
inline static void PlatformP$PXA27xWD$enableWatchdog(void ){
#line 45
  HplPXA27xOSTimerM$PXA27xWD$enableWatchdog();
#line 45
}
#line 45
# 64 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static uint32_t PlatformP$OST0M3$getOSCR(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSCR(3);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64







inline static void PlatformP$OST0M3$setOSMR(uint32_t val){
#line 71
  HplPXA27xOSTimerM$PXA27xOST$setOSMR(3, val);
#line 71
}
#line 71
# 117 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/PlatformP.nc"
static inline void PlatformP$PlatformReset$reset(void )
#line 117
{
  PlatformP$OST0M3$setOSMR(PlatformP$OST0M3$getOSCR() + 1000);
  PlatformP$PXA27xWD$enableWatchdog();
  while (1) ;
  return;
}

# 32 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/PlatformReset.nc"
inline static void PMICM$PlatformReset$reset(void ){
#line 32
  PlatformP$PlatformReset$reset();
#line 32
}
#line 32
# 271 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline void PMICM$PMICGPIO$interruptGPIOPin(void )
#line 271
{
  uint8_t events[3];
  bool localGotReset;



  PMICM$readPMIC(0x01, events, 3);

  if (events[0] & 0x1) {
      /* atomic removed: atomic calls only */
#line 280
      {
        localGotReset = PMICM$gotReset;
      }
      if (localGotReset == TRUE) {
          PMICM$PlatformReset$reset();
        }
      else {
          /* atomic removed: atomic calls only */
#line 287
          {
            PMICM$gotReset = TRUE;
          }
        }
    }
  else {
    }
}

# 42 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void PMICM$PI2C$setIDBR(uint32_t val){
#line 42
  /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setIDBR(val);
#line 42
}
#line 42

inline static uint32_t PMICM$PI2C$getIDBR(void ){
#line 43
  unsigned int __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getIDBR();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 112 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static bool PlatformP$OST0M3$clearOSSRbit(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(3);
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112







inline static void PlatformP$OST0M3$setOIERbit(bool flag){
#line 119
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(3, flag);
#line 119
}
#line 119
# 124 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/PlatformP.nc"
static inline void PlatformP$OST0M3$fired(void )
{
  PlatformP$OST0M3$setOIERbit(FALSE);
  PlatformP$OST0M3$clearOSSRbit();
  return;
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 81 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$fired(void )
{
#line 82
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$postTask();
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$fired(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$fired();
#line 78
}
#line 78
# 119 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(bool flag){
#line 119
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(4, flag);
#line 119
}
#line 119
#line 112
inline static bool /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$clearOSSRbit(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(4);
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 160 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$fired(void )
#line 160
{
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$clearOSSRbit();
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(FALSE);
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning = FALSE;
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$fired();
  return;
}

# 104 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$default$overflow(void )
#line 104
{
  return;
}

# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$overflow(void ){
#line 82
  /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$default$overflow();
#line 82
}
#line 82
# 112 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static bool /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$clearOSSRbit(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(5);
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 91 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$fired(void )
#line 91
{
  /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$clearOSSRbit();
  /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$overflow();
  return;
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$fired(void ){
#line 78
  CC2420TransmitP$BackoffTimer$fired();
#line 78
  CC2420ControlP$StartupTimer$fired();
#line 78
}
#line 78
# 112 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static bool /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$clearOSSRbit(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(6);
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 160 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$fired(void )
#line 160
{
  /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$clearOSSRbit();
  /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOIERbit(FALSE);
  /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mfRunning = FALSE;
  /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$fired();
  return;
}

# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static bool CC2420TransmitP$CCA$get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = HalPXA27xGeneralIOM$GeneralIO$get(116);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420TransmitP$SpiResource$immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420SpiP$Resource$immediateRequest(/*CC2420TransmitC.Spi*/CC2420SpiC$3$CLIENT_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
#line 88
inline static error_t CC2420TransmitP$SpiResource$request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP$Resource$request(/*CC2420TransmitC.Spi*/CC2420SpiC$3$CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
inline static error_t CC2420ControlP$SpiResource$request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP$Resource$request(/*CC2420ControlC.Spi*/CC2420SpiC$0$CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 188 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP$Resource$request(void )
#line 188
{
  return CC2420ControlP$SpiResource$request();
}

# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP$Resource$request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420ControlP$Resource$request();
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 210 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP$CC2420Power$startVRegDone(void )
#line 210
{
  CC2420CsmaP$Resource$request();
}

# 56 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static void CC2420ControlP$CC2420Power$startVRegDone(void ){
#line 56
  CC2420CsmaP$CC2420Power$startVRegDone();
#line 56
}
#line 56
# 80 "/opt/tinyos-2.1.2/tos/lib/gpio/SoftCaptureP.nc"
static inline void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$overflow(void )
#line 80
{
  return;
}

# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Counter$overflow(void ){
#line 82
  /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$Counter32khz32$overflow();
#line 82
}
#line 82
# 112 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static bool /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$clearOSSRbit(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(7);
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 91 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$fired(void )
#line 91
{
  /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$clearOSSRbit();
  /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Counter$overflow();
  return;
}

# 58 "/opt/tinyos-2.1.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void )
{
}

# 82 "/opt/tinyos-2.1.2/tos/lib/timer/Counter.nc"
inline static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$Counter$overflow(void ){
#line 82
  /*CC2420PacketC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow();
#line 82
}
#line 82
# 112 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static bool /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$clearOSSRbit(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(8);
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 91 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$fired(void )
#line 91
{
  /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$clearOSSRbit();
  /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$Counter$overflow();
  return;
}

# 124 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP$Scheduler$init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP$m_next, SchedulerBasicP$NO_TASK, sizeof SchedulerBasicP$m_next);
    SchedulerBasicP$m_head = SchedulerBasicP$NO_TASK;
    SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
  }
}

# 57 "/opt/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$init(void ){
#line 57
  SchedulerBasicP$Scheduler$init();
#line 57
}
#line 57
# 69 "/opt/tinyos-2.1.2/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$set(void ){
#line 40
  HalPXA27xGeneralIOM$GeneralIO$set(105);
#line 40
}
#line 40
inline static void LedsP$Led1$set(void ){
#line 40
  HalPXA27xGeneralIOM$GeneralIO$set(104);
#line 40
}
#line 40
inline static void LedsP$Led0$set(void ){
#line 40
  HalPXA27xGeneralIOM$GeneralIO$set(103);
#line 40
}
#line 40
# 52 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPDRbit(uint8_t arg_0x2b48014b4d00, bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(arg_0x2b48014b4d00, dir);
#line 52
}
#line 52
# 94 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline void HalPXA27xGeneralIOM$GeneralIO$makeOutput(uint8_t pin)
#line 94
{
  /* atomic removed: atomic calls only */
#line 95
  HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPDRbit(pin, TRUE);
  return;
}

# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$makeOutput(void ){
#line 46
  HalPXA27xGeneralIOM$GeneralIO$makeOutput(105);
#line 46
}
#line 46
inline static void LedsP$Led1$makeOutput(void ){
#line 46
  HalPXA27xGeneralIOM$GeneralIO$makeOutput(104);
#line 46
}
#line 46
inline static void LedsP$Led0$makeOutput(void ){
#line 46
  HalPXA27xGeneralIOM$GeneralIO$makeOutput(103);
#line 46
}
#line 46
# 56 "/opt/tinyos-2.1.2/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 57
  {
    ;
    LedsP$Led0$makeOutput();
    LedsP$Led1$makeOutput();
    LedsP$Led2$makeOutput();
    LedsP$Led0$set();
    LedsP$Led1$set();
    LedsP$Led2$set();
  }
  return SUCCESS;
}

# 37 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
inline static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR0(uint32_t val){
#line 37
  HplPXA27xSSPP$HplPXA27xSSP$setSSCR0(3, val);
#line 37
}
#line 37
# 168 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
static inline void HplPXA27xSSPP$HplPXA27xSSP$setSSTO(uint8_t chnl, uint32_t val)
#line 168
{
  switch (chnl) {
      case 1: * (volatile uint32_t *)0x41000028 = val;
#line 170
      break;
      case 2: * (volatile uint32_t *)0x41700028 = val;
#line 171
      break;
      case 3: * (volatile uint32_t *)0x41900028 = val;
#line 172
      break;
      default: break;
    }
  return;
}

# 52 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSP.nc"
inline static void /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSTO(uint32_t val){
#line 52
  HplPXA27xSSPP$HplPXA27xSSP$setSSTO(3, val);
#line 52
}
#line 52
# 79 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSpiPioM.nc"
static inline error_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$Init$init(void )
#line 79
{

  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txBitBucket = 0, /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxBitBucket = 0;
  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txCurrentBuf = /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxCurrentBuf = (void *)0;
  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenCurrent = 0;
  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$instanceCurrent = 0;
  /* atomic removed: atomic calls only */
#line 85
  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenRemain = 0;

  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR1(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR1);
  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSTO(3500);
  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR0(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR0);
  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR0(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR0 | (1 << 7));

  return SUCCESS;
}

# 52 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void IM2CC2420InitSpiP$RXD$setGPDRbit(bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(41, dir);
#line 52
}
#line 52
#line 134
inline static void IM2CC2420InitSpiP$RXD$setGAFRpin(uint8_t func){
#line 134
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(41, func);
#line 134
}
#line 134
#line 52
inline static void IM2CC2420InitSpiP$TXD$setGPDRbit(bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(35, dir);
#line 52
}
#line 52
#line 134
inline static void IM2CC2420InitSpiP$TXD$setGAFRpin(uint8_t func){
#line 134
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(35, func);
#line 134
}
#line 134
#line 52
inline static void IM2CC2420InitSpiP$SCLK$setGPDRbit(bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(34, dir);
#line 52
}
#line 52
#line 134
inline static void IM2CC2420InitSpiP$SCLK$setGAFRpin(uint8_t func){
#line 134
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(34, func);
#line 134
}
#line 134
# 48 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/cc2420/IM2CC2420InitSpiP.nc"
static inline error_t IM2CC2420InitSpiP$Init$init(void )
#line 48
{
  IM2CC2420InitSpiP$SCLK$setGAFRpin(3);
  IM2CC2420InitSpiP$SCLK$setGPDRbit(TRUE);
  IM2CC2420InitSpiP$TXD$setGAFRpin(3);
  IM2CC2420InitSpiP$TXD$setGPDRbit(TRUE);
  IM2CC2420InitSpiP$RXD$setGAFRpin(3);
  IM2CC2420InitSpiP$RXD$setGPDRbit(FALSE);

  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t PlatformP$InitL3$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = IM2CC2420InitSpiP$Init$init();
#line 62
  __nesc_result = ecombine(__nesc_result, /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, LedsP$Init$init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 171 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline void PMICM$startLDOs(void )
#line 171
{

  uint8_t oldVal;
#line 173
  uint8_t newVal;



  PMICM$readPMIC(0x17, &oldVal, 1);
  newVal = (oldVal | 0x2) | 0x4;
  PMICM$writePMIC(0x17, newVal);

  PMICM$readPMIC(0x98, &oldVal, 1);
  newVal = (oldVal | 0x4) | 0x8;
  PMICM$writePMIC(0x98, newVal);




  PMICM$readPMIC(0x97, &oldVal, 1);
  newVal = oldVal | 0x20;
  PMICM$writePMIC(0x97, newVal);
}

#line 300
static inline error_t PMICM$PMIC$setCoreVoltage(uint8_t trimValue)
#line 300
{
  PMICM$writePMIC(0x15, (trimValue & 0x1f) | 0x80);
  return SUCCESS;
}

# 101 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void PMICM$PMICGPIO$setGFERbit(bool flag){
#line 101
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGFERbit(1, flag);
#line 101
}
#line 101
#line 52
inline static void PMICM$PMICGPIO$setGPDRbit(bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(1, dir);
#line 52
}
#line 52
#line 134
inline static void PMICM$PMICGPIO$setGAFRpin(uint8_t func){
#line 134
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(1, func);
#line 134
}
#line 134
# 46 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static uint32_t PMICM$PI2C$getICR(void ){
#line 46
  unsigned int __nesc_result;
#line 46

#line 46
  __nesc_result = /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getICR();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
#line 45
inline static void PMICM$PI2C$setICR(uint32_t val){
#line 45
  /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setICR(val);
#line 45
}
#line 45
# 219 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline error_t PMICM$Init$init(void )
#line 219
{
  uint8_t val[3];

#line 221
  * (volatile uint32_t *)0x40F0001C |= 1 << 6;
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | ((1 << 6) | (1 << 5)));
  /* atomic removed: atomic calls only */
#line 223
  {
    PMICM$gotReset = FALSE;
  }

  PMICM$PMICGPIO$setGAFRpin(0);
  PMICM$PMICGPIO$setGPDRbit(FALSE);
  PMICM$PMICGPIO$setGFERbit(TRUE);




  PMICM$writePMIC(0x08, (
  0x80 | 0x8) | 0x4);


  PMICM$writePMIC(0x05, ~0x1);
  PMICM$writePMIC(0x06, 0xFF);
  PMICM$writePMIC(0x07, 0xFF);


  PMICM$readPMIC(0x01, val, 3);




  PMICM$PMIC$setCoreVoltage(0x4);



  PMICM$startLDOs();
  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t PlatformP$InitL2$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PMICM$Init$init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 193 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
static inline void HplPXA27xInterruptM$PXA27xIrq$enable(uint8_t id)
{
  HplPXA27xInterruptM$enable(id);
  return;
}

# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(6);
#line 65
}
#line 65
# 188 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
static inline error_t HplPXA27xInterruptM$PXA27xIrq$allocate(uint8_t id)
{
  return HplPXA27xInterruptM$allocate(id, FALSE, TOSH_IRP_TABLE[id]);
}

# 60 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static error_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(6);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 54 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline error_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$Init$init(void )
#line 54
{
  bool isInited;

  /* atomic removed: atomic calls only */
#line 57
  {
    isInited = /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$m_fInit;
    /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$m_fInit = TRUE;
  }

  if (!isInited) {
      switch (1) {
          case 0: 
            * (volatile uint32_t *)0x41300004 |= 1 << 14;
          * (volatile uint32_t *)0x40301690 = 0;
          break;
          case 1: 
            * (volatile uint32_t *)0x41300004 |= 1 << 15;
          * (volatile uint32_t *)0x40F00190 = 0;
          break;
          default: 
            break;
        }
      /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$allocate();
      /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$enable();
    }

  return SUCCESS;
}

# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void HplPXA27xGPIOM$GPIOIrq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(10);
#line 65
}
#line 65
inline static void HplPXA27xGPIOM$GPIOIrq1$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(9);
#line 65
}
#line 65
inline static void HplPXA27xGPIOM$GPIOIrq0$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(8);
#line 65
}
#line 65
#line 60
inline static error_t HplPXA27xGPIOM$GPIOIrq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(10);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xGPIOM$GPIOIrq1$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(9);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xGPIOM$GPIOIrq0$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(8);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 60 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline error_t HplPXA27xGPIOM$Init$init(void )
{
  bool isInited;

  /* atomic removed: atomic calls only */
#line 64
  {
    isInited = HplPXA27xGPIOM$gfInitialized;
    HplPXA27xGPIOM$gfInitialized = TRUE;
  }

  if (!isInited) {
      HplPXA27xGPIOM$GPIOIrq0$allocate();
      HplPXA27xGPIOM$GPIOIrq1$allocate();
      HplPXA27xGPIOM$GPIOIrq$allocate();
      HplPXA27xGPIOM$GPIOIrq0$enable();
      HplPXA27xGPIOM$GPIOIrq1$enable();
      HplPXA27xGPIOM$GPIOIrq$enable();
    }
  return SUCCESS;
}

# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void HplPXA27xSSPP$SSP3Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(0);
#line 65
}
#line 65
# 293 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
static inline void HplPXA27xSSPP$SSP2Irq$default$enable(void )
#line 293
{
#line 293
  return;
}

# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void HplPXA27xSSPP$SSP2Irq$enable(void ){
#line 65
  HplPXA27xSSPP$SSP2Irq$default$enable();
#line 65
}
#line 65
# 292 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
static inline void HplPXA27xSSPP$SSP1Irq$default$enable(void )
#line 292
{
#line 292
  return;
}

# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void HplPXA27xSSPP$SSP1Irq$enable(void ){
#line 65
  HplPXA27xSSPP$SSP1Irq$default$enable();
#line 65
}
#line 65
# 52 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
static inline error_t HplPXA27xSSPP$Init$init(uint8_t chnl)
#line 52
{
  error_t error = SUCCESS;

  switch (chnl) {
      case 1: 
        * (volatile uint32_t *)0x41300004 |= 1 << 23;
      HplPXA27xSSPP$SSP1Irq$enable();
      break;
      case 2: 
        * (volatile uint32_t *)0x41300004 |= 1 << 3;
      HplPXA27xSSPP$SSP2Irq$enable();
      break;
      case 3: 
        * (volatile uint32_t *)0x41300004 |= 1 << 4;

      HplPXA27xSSPP$SSP3Irq$enable();
      break;
      default: 
        error = FAIL;
      break;
    }

  return error;
}

# 65 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void HplPXA27xDMAM$DMAIrq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(25);
#line 65
}
#line 65
#line 60
inline static error_t HplPXA27xDMAM$DMAIrq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(25);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 50 "/opt/tinyos-2.1.2/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static inline error_t HplPXA27xDMAM$Init$init(void )
#line 50
{
  HplPXA27xDMAM$DMAIrq$allocate();
  HplPXA27xDMAM$DMAIrq$enable();
  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t PlatformP$InitL1$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = HplPXA27xDMAM$Init$init();
#line 62
  __nesc_result = ecombine(__nesc_result, HplPXA27xSSPP$Init$init(3));
#line 62
  __nesc_result = ecombine(__nesc_result, HplPXA27xGPIOM$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$Init$init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 119 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOIERbit(bool flag){
#line 119
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(5, flag);
#line 119
}
#line 119
#line 57
inline static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSCR(uint32_t val){
#line 57
  HplPXA27xOSTimerM$PXA27xOST$setOSCR(5, val);
#line 57
}
#line 57
#line 71
inline static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSMR(uint32_t val){
#line 71
  HplPXA27xOSTimerM$PXA27xOST$setOSMR(5, val);
#line 71
}
#line 71
#line 85
inline static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOMCR(uint32_t val){
#line 85
  HplPXA27xOSTimerM$PXA27xOST$setOMCR(5, val);
#line 85
}
#line 85
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTInit$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = HplPXA27xOSTimerM$Init$init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 56 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline error_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Init$init(void )
#line 56
{

  /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTInit$init();
  /* atomic removed: atomic calls only */

  {
    /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOMCR(((1 << 7) | (1 << 6)) | (((2 & 0x8) << 5) | ((2 & 0x7) << 0)));
    /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSMR(0);
    /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSCR(1);
    /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$clearOSSRbit();
    /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOIERbit(TRUE);
  }
  return SUCCESS;
}

# 119 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOIERbit(bool flag){
#line 119
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(7, flag);
#line 119
}
#line 119
#line 57
inline static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOSCR(uint32_t val){
#line 57
  HplPXA27xOSTimerM$PXA27xOST$setOSCR(7, val);
#line 57
}
#line 57
#line 71
inline static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOSMR(uint32_t val){
#line 71
  HplPXA27xOSTimerM$PXA27xOST$setOSMR(7, val);
#line 71
}
#line 71
#line 85
inline static void /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOMCR(uint32_t val){
#line 85
  HplPXA27xOSTimerM$PXA27xOST$setOMCR(7, val);
#line 85
}
#line 85
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTInit$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = HplPXA27xOSTimerM$Init$init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 56 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline error_t /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Init$init(void )
#line 56
{

  /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTInit$init();
  /* atomic removed: atomic calls only */

  {
    /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOMCR(((1 << 7) | (1 << 6)) | (((1 & 0x8) << 5) | ((1 & 0x7) << 0)));
    /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOSMR(0);
    /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOSCR(1);
    /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$clearOSSRbit();
    /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$setOIERbit(TRUE);
  }
  return SUCCESS;
}

# 119 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOIERbit(bool flag){
#line 119
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(8, flag);
#line 119
}
#line 119
#line 57
inline static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOSCR(uint32_t val){
#line 57
  HplPXA27xOSTimerM$PXA27xOST$setOSCR(8, val);
#line 57
}
#line 57
#line 71
inline static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOSMR(uint32_t val){
#line 71
  HplPXA27xOSTimerM$PXA27xOST$setOSMR(8, val);
#line 71
}
#line 71
#line 85
inline static void /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOMCR(uint32_t val){
#line 85
  HplPXA27xOSTimerM$PXA27xOST$setOMCR(8, val);
#line 85
}
#line 85
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTInit$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = HplPXA27xOSTimerM$Init$init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 56 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline error_t /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$Init$init(void )
#line 56
{

  /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTInit$init();
  /* atomic removed: atomic calls only */

  {
    /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOMCR(((1 << 7) | (1 << 6)) | (((1 & 0x8) << 5) | ((1 & 0x7) << 0)));
    /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOSMR(0);
    /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOSCR(1);
    /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$clearOSSRbit();
    /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$setOIERbit(TRUE);
  }
  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t PlatformP$InitL0$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$Init$init();
#line 62
  __nesc_result = ecombine(__nesc_result, /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Init$init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 263 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/hardware.h"
static inline void TOSH_SET_PIN_DIRECTIONS(void )
{

  * (volatile uint32_t *)0x40F00004 = (1 << 5) | (1 << 4);
}

# 52 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/PlatformP.nc"
static inline error_t PlatformP$Init$init(void )
#line 52
{


  * (volatile uint32_t *)0x41300004 = (((1 << 22) | (1 << 20)) | (1 << 15)) | (1 << 9);

  * (volatile uint32_t *)0x48000048 = (((1 << 24) | ((
  0 & 0xF) << 8)) | ((1 & 0xF) << 4)) | ((4 & 0xF) << 0);


  * (volatile uint32_t *)0x41300008 = 1 << 1;
  while ((* (volatile uint32_t *)0x41300008 & (1 << 0)) == 0) ;

  TOSH_SET_PIN_DIRECTIONS();



   __asm volatile ("mcr p15,0,%0,c15,c1,0\n\t" :  : "r"(0x43));




  * (volatile uint32_t *)0x41300000 = (((1 << 31) | (8 & 0x1f)) | ((2 & 0xf) << 7)) | (1 << 25);
   __asm volatile (
  "mcr p14,0,%0,c6,c0,0\n\t" :  : 

  "r"(1 << 1));
#line 91
  * (volatile uint32_t *)0x48000064 = (1 & 0x3) << 12;
  * (volatile uint32_t *)0x48000008 = ((* (volatile uint32_t *)0x48000008 | (1 << 3)) | (1 << 15)) | ((2 & 0x7) << 0);
  * (volatile uint32_t *)0x4800000C = * (volatile uint32_t *)0x4800000C | (1 << 3);
  * (volatile uint32_t *)0x48000010 = * (volatile uint32_t *)0x48000010 | (1 << 3);
  * (volatile uint32_t *)0x48000014 = 0;

  * (volatile uint32_t *)0x48000000 = ((((((((1 << 27) | (1 << 11)) | ((0x3 & 0x3) << 24)) | (
  1 << 13)) | ((0x3 & 0x3) << 8)) | (1 << 7)) | ((
  0x2 & 0x3) << 5)) | ((0x1 & 0x3) << 3)) | (1 << 2);

  * (volatile uint32_t *)0x48000004 = (* (volatile uint32_t *)0x48000004 & ~((1 << 29) | (1 << 14))) | (1 << 14);

  enableICache();
  initSyncFlash();



  PlatformP$InitL0$init();
  PlatformP$InitL1$init();
  PlatformP$InitL2$init();
  PlatformP$InitL3$init();


  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t RealMainP$PlatformInit$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = PlatformP$Init$init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 60 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static error_t HplPXA27xOSTimerM$OST0Irq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(26);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xOSTimerM$OST1Irq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(27);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xOSTimerM$OST2Irq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(28);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xOSTimerM$OST3Irq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(29);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xOSTimerM$OST4_11Irq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(7);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60





inline static void HplPXA27xOSTimerM$OST0Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(26);
#line 65
}
#line 65
inline static void HplPXA27xOSTimerM$OST1Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(27);
#line 65
}
#line 65
inline static void HplPXA27xOSTimerM$OST2Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(28);
#line 65
}
#line 65
inline static void HplPXA27xOSTimerM$OST3Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(29);
#line 65
}
#line 65
inline static void HplPXA27xOSTimerM$OST4_11Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(7);
#line 65
}
#line 65
# 65 "/opt/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
inline static bool RealMainP$Scheduler$runNextTask(void ){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = SchedulerBasicP$Scheduler$runNextTask();
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 52 "MoteToMoteC.nc"
static inline void MoteToMoteC$AMSend$sendDone(message_t *msg, error_t error)
{
  if (msg == &MoteToMoteC$packet) 
    {
      MoteToMoteC$RadioBusy = FALSE;
    }
}

# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static void /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$sendDone(message_t * msg, error_t error){
#line 110
  MoteToMoteC$AMSend$sendDone(msg, error);
#line 110
}
#line 110
# 65 "/opt/tinyos-2.1.2/tos/system/AMQueueEntryP.nc"
static inline void /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$sendDone(message_t *m, error_t err)
#line 65
{
  /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$sendDone(m, err);
}

# 215 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$default$sendDone(uint8_t id, message_t *msg, error_t err)
#line 215
{
}

# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$sendDone(uint8_t arg_0x2b4801ec1318, message_t * msg, error_t error){
#line 100
  switch (arg_0x2b4801ec1318) {
#line 100
    case 0U:
#line 100
      /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$sendDone(msg, error);
#line 100
      break;
#line 100
    default:
#line 100
      /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$default$sendDone(arg_0x2b4801ec1318, msg, error);
#line 100
      break;
#line 100
    }
#line 100
}
#line 100
# 126 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask$runTask(void )
#line 126
{
  uint8_t i;
#line 127
  uint8_t j;
#line 127
  uint8_t mask;
#line 127
  uint8_t last;
  message_t *msg;

#line 129
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[last].msg;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[last].msg = (void *)0;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$cancelMask[i] &= ~mask;
                  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

#line 169
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$runTask(void )
#line 169
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current, /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current].msg, FAIL);
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$send(am_id_t arg_0x2b4801ec0488, am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = CC2420ActiveMessageP$AMSend$send(arg_0x2b4801ec0488, addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420ActiveMessageP$CC2420PacketBody$getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP$CC2420PacketBody$getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 194 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP$Packet$payloadLength(message_t *msg)
#line 194
{
  return __nesc_ntoh_leuint8(CC2420ActiveMessageP$CC2420PacketBody$getHeader(msg)->length.nxdata) - CC2420_SIZE;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
inline static uint8_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$payloadLength(message_t * msg){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ActiveMessageP$Packet$payloadLength(msg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 78 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$destination(message_t * amsg){
#line 78
  unsigned short __nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ActiveMessageP$AMPacket$destination(amsg);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 164 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline am_id_t CC2420ActiveMessageP$AMPacket$type(message_t *amsg)
#line 164
{
  cc2420_header_t *header = CC2420ActiveMessageP$CC2420PacketBody$getHeader(amsg);

#line 166
  return __nesc_ntoh_leuint8(header->type.nxdata);
}

# 147 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static am_id_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$type(message_t * amsg){
#line 147
  unsigned char __nesc_result;
#line 147

#line 147
  __nesc_result = CC2420ActiveMessageP$AMPacket$type(amsg);
#line 147

#line 147
  return __nesc_result;
#line 147
}
#line 147
# 65 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$nextPacket(void )
#line 65
{
  uint8_t i;

#line 67
  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current].msg == (void *)0 || 
      /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$cancelMask[/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current / 8] & (1 << /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current % 8)) 
        {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current = (/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 78
    /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current = 1;
    }
}

#line 174
static inline void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$tryToSend(void )
#line 174
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$nextPacket();
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current].msg;
      am_id_t nextId = /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$type(nextMsg);
      am_addr_t nextDest = /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$destination(nextMsg);
      uint8_t len = /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$payloadLength(nextMsg);

#line 182
      nextErr = /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$postTask();
        }
    }
}

# 173 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline uint8_t CC2420CsmaP$Send$maxPayloadLength(void )
#line 173
{
  return 28;
}

# 112 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static uint8_t UniqueSendP$SubSend$maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = CC2420CsmaP$Send$maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline uint8_t UniqueSendP$Send$maxPayloadLength(void )
#line 95
{
  return UniqueSendP$SubSend$maxPayloadLength();
}

# 112 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static uint8_t CC2420TinyosNetworkP$SubSend$maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = UniqueSendP$Send$maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 90 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline uint8_t CC2420TinyosNetworkP$ActiveSend$maxPayloadLength(void )
#line 90
{
  return CC2420TinyosNetworkP$SubSend$maxPayloadLength();
}

# 112 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static uint8_t CC2420ActiveMessageP$SubSend$maxPayloadLength(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = CC2420TinyosNetworkP$ActiveSend$maxPayloadLength();
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 202 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline uint8_t CC2420ActiveMessageP$Packet$maxPayloadLength(void )
#line 202
{
  return CC2420ActiveMessageP$SubSend$maxPayloadLength();
}

# 310 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline uint16_t CC2420ControlP$CC2420Config$getPanAddr(void )
#line 310
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 311
    {
      unsigned short __nesc_temp = 
#line 311
      CC2420ControlP$m_pan;

      {
#line 311
        __nesc_atomic_end(__nesc_atomic); 
#line 311
        return __nesc_temp;
      }
    }
#line 313
    __nesc_atomic_end(__nesc_atomic); }
}

# 77 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static uint16_t CC2420ActiveMessageP$CC2420Config$getPanAddr(void ){
#line 77
  unsigned short __nesc_result;
#line 77

#line 77
  __nesc_result = CC2420ControlP$CC2420Config$getPanAddr();
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 53 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static bool CC2420TinyosNetworkP$Queue$isEmpty(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$isEmpty();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 215 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP$Resource$immediateRequest(uint8_t id)
#line 215
{
  if (CC2420TinyosNetworkP$resource_owner == id) {
#line 216
    return EALREADY;
    }
  if (CC2420TinyosNetworkP$TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP$resource_owner == CC2420TinyosNetworkP$OWNER_NONE && CC2420TinyosNetworkP$Queue$isEmpty()) {
          CC2420TinyosNetworkP$resource_owner = id;
          return SUCCESS;
        }
      return FAIL;
    }
  else 
#line 224
    {
      CC2420TinyosNetworkP$resource_owner = id;
      return SUCCESS;
    }
}

# 97 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP$RadioResource$immediateRequest(void ){
#line 97
  unsigned char __nesc_result;
#line 97

#line 97
  __nesc_result = CC2420TinyosNetworkP$Resource$immediateRequest(CC2420ActiveMessageC$CC2420_AM_SEND_ID);
#line 97

#line 97
  return __nesc_result;
#line 97
}
#line 97
# 291 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$SendNotifier$default$aboutToSend(am_id_t amId, am_addr_t addr, message_t *msg)
#line 291
{
}

# 59 "/opt/tinyos-2.1.2/tos/interfaces/SendNotifier.nc"
inline static void CC2420ActiveMessageP$SendNotifier$aboutToSend(am_id_t arg_0x2b4801e14258, am_addr_t dest, message_t * msg){
#line 59
    CC2420ActiveMessageP$SendNotifier$default$aboutToSend(arg_0x2b4801e14258, dest, msg);
#line 59
}
#line 59
# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static error_t CC2420ActiveMessageP$SubSend$send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420TinyosNetworkP$ActiveSend$send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 128 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline void CC2420PacketP$CC2420Packet$setNetwork(message_t * p_msg, uint8_t networkId)
#line 128
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    *CC2420PacketP$getNetwork(p_msg) = networkId;
#line 131
    __nesc_atomic_end(__nesc_atomic); }
}

# 77 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static void CC2420TinyosNetworkP$CC2420Packet$setNetwork(message_t * p_msg, uint8_t networkId){
#line 77
  CC2420PacketP$CC2420Packet$setNetwork(p_msg, networkId);
#line 77
}
#line 77
# 81 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline int CC2420PacketP$getAddressLength(int type)
#line 81
{
  switch (type) {
      case IEEE154_ADDR_SHORT: return 2;
      case IEEE154_ADDR_EXT: return 8;
      case IEEE154_ADDR_NONE: return 0;
      default: return -100;
    }
}

# 297 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_leuint8(void * target, uint8_t value)
#line 297
{
  uint8_t *base = target;

#line 299
  base[0] = value;
  return value;
}

#line 327
static __inline  uint16_t __nesc_hton_leuint16(void * target, uint16_t value)
#line 327
{
  uint8_t *base = target;

#line 329
  base[0] = value;
  base[1] = value >> 8;
  return value;
}

# 547 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP$send(message_t * p_msg, bool cca)
#line 547
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 548
    {
      if (CC2420TransmitP$m_state == CC2420TransmitP$S_CANCEL) {
          {
            unsigned char __nesc_temp = 
#line 550
            ECANCEL;

            {
#line 550
              __nesc_atomic_end(__nesc_atomic); 
#line 550
              return __nesc_temp;
            }
          }
        }
#line 553
      if (CC2420TransmitP$m_state != CC2420TransmitP$S_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 554
            FAIL;

            {
#line 554
              __nesc_atomic_end(__nesc_atomic); 
#line 554
              return __nesc_temp;
            }
          }
        }


      CC2420TransmitP$m_state = CC2420TransmitP$S_LOAD;
      CC2420TransmitP$m_cca = cca;
      CC2420TransmitP$m_msg = p_msg;
      CC2420TransmitP$totalCcaChecks = 0;
    }
#line 564
    __nesc_atomic_end(__nesc_atomic); }

  if (CC2420TransmitP$acquireSpiResource() == SUCCESS) {
      CC2420TransmitP$loadTXFIFO();
    }

  return SUCCESS;
}

#line 192
static inline error_t CC2420TransmitP$Send$send(message_t * p_msg, bool useCca)
#line 192
{
  return CC2420TransmitP$send(p_msg, useCca);
}

# 51 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Transmit.nc"
inline static error_t CC2420CsmaP$CC2420Transmit$send(message_t * p_msg, bool useCca){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420TransmitP$Send$send(p_msg, useCca);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 301 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$RadioBackoff$default$requestCca(am_id_t id, 
message_t *msg)
#line 302
{
}

# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420ActiveMessageP$RadioBackoff$requestCca(am_id_t arg_0x2b4801e14d60, message_t * msg){
#line 95
    CC2420ActiveMessageP$RadioBackoff$default$requestCca(arg_0x2b4801e14d60, msg);
#line 95
}
#line 95
# 250 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$SubBackoff$requestCca(message_t *msg)
#line 250
{

  CC2420ActiveMessageP$RadioBackoff$requestCca(__nesc_ntoh_leuint8(((cc2420_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t )))->type.nxdata), msg);
}

# 95 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/RadioBackoff.nc"
inline static void CC2420CsmaP$RadioBackoff$requestCca(message_t * msg){
#line 95
  CC2420ActiveMessageP$SubBackoff$requestCca(msg);
#line 95
}
#line 95
# 111 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
static inline void StateImplP$State$forceState(uint8_t id, uint8_t reqState)
#line 111
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    StateImplP$state[id] = reqState;
#line 112
    __nesc_atomic_end(__nesc_atomic); }
}

# 51 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
inline static void CC2420CsmaP$SplitControlState$forceState(uint8_t reqState){
#line 51
  StateImplP$State$forceState(1U, reqState);
#line 51
}
#line 51
#line 66
inline static bool CC2420CsmaP$SplitControlState$isState(uint8_t myState){
#line 66
  unsigned char __nesc_result;
#line 66

#line 66
  __nesc_result = StateImplP$State$isState(1U, myState);
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420CsmaP$CC2420PacketBody$getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP$CC2420PacketBody$getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
#line 42
inline static cc2420_header_t * CC2420CsmaP$CC2420PacketBody$getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP$CC2420PacketBody$getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 122 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline error_t CC2420CsmaP$Send$send(message_t *p_msg, uint8_t len)
#line 122
{
  unsigned char *__nesc_temp43;
  unsigned char *__nesc_temp42;
#line 124
  cc2420_header_t *header = CC2420CsmaP$CC2420PacketBody$getHeader(p_msg);
  cc2420_metadata_t *metadata = CC2420CsmaP$CC2420PacketBody$getMetadata(p_msg);

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 127
    {
      if (!CC2420CsmaP$SplitControlState$isState(CC2420CsmaP$S_STARTED)) {
          {
            unsigned char __nesc_temp = 
#line 129
            FAIL;

            {
#line 129
              __nesc_atomic_end(__nesc_atomic); 
#line 129
              return __nesc_temp;
            }
          }
        }
#line 132
      CC2420CsmaP$SplitControlState$forceState(CC2420CsmaP$S_TRANSMITTING);
      CC2420CsmaP$m_msg = p_msg;
    }
#line 134
    __nesc_atomic_end(__nesc_atomic); }








  (__nesc_temp42 = header->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp42, __nesc_ntoh_leuint16(__nesc_temp42) & (((1 << IEEE154_FCF_ACK_REQ) | (
  0x3 << IEEE154_FCF_SRC_ADDR_MODE)) | (
  0x3 << IEEE154_FCF_DEST_ADDR_MODE))));

  (__nesc_temp43 = header->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp43, __nesc_ntoh_leuint16(__nesc_temp43) | ((IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE) | (
  1 << IEEE154_FCF_INTRAPAN))));

  __nesc_hton_int8(metadata->ack.nxdata, FALSE);
  __nesc_hton_uint8(metadata->rssi.nxdata, 0);
  __nesc_hton_uint8(metadata->lqi.nxdata, 0);

  __nesc_hton_uint32(metadata->timestamp.nxdata, CC2420_INVALID_TIMESTAMP);

  CC2420CsmaP$ccaOn = TRUE;
  CC2420CsmaP$RadioBackoff$requestCca(CC2420CsmaP$m_msg);

  CC2420CsmaP$CC2420Transmit$send(CC2420CsmaP$m_msg, CC2420CsmaP$ccaOn);
  return SUCCESS;
}

# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static error_t UniqueSendP$SubSend$send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420CsmaP$Send$send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueSendP$CC2420PacketBody$getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP$CC2420PacketBody$getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 45 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
inline static error_t UniqueSendP$State$requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP$State$requestState(2U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 75 "/opt/tinyos-2.1.2/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline error_t UniqueSendP$Send$send(message_t *msg, uint8_t len)
#line 75
{
  error_t error;

#line 77
  if (UniqueSendP$State$requestState(UniqueSendP$S_SENDING) == SUCCESS) {
      __nesc_hton_leuint8(UniqueSendP$CC2420PacketBody$getHeader(msg)->dsn.nxdata, UniqueSendP$localSendId++);

      if ((error = UniqueSendP$SubSend$send(msg, len)) != SUCCESS) {
          UniqueSendP$State$toIdle();
        }

      return error;
    }

  return EBUSY;
}

# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static error_t CC2420TinyosNetworkP$SubSend$send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = UniqueSendP$Send$send(msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420TransmitP$TXCTRL$write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP$Reg$write(CC2420_TXCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 45 "/opt/tinyos-2.1.2/tos/interfaces/SpiByte.nc"
inline static uint8_t CC2420SpiP$SpiByte$write(uint8_t tx){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiByte$write(tx);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 126 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
static inline bool StateImplP$State$isIdle(uint8_t id)
#line 126
{
  return StateImplP$State$isState(id, StateImplP$S_IDLE);
}

# 61 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
inline static bool CC2420SpiP$WorkingState$isIdle(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = StateImplP$State$isIdle(0U);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 214 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP$Fifo$write(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 215
{

  uint8_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 219
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 221
            status;

            {
#line 221
              __nesc_atomic_end(__nesc_atomic); 
#line 221
              return __nesc_temp;
            }
          }
        }
    }
#line 225
    __nesc_atomic_end(__nesc_atomic); }
#line 225
  CC2420SpiP$m_addr = addr;

  status = CC2420SpiP$SpiByte$write(CC2420SpiP$m_addr);
  CC2420SpiP$SpiPacket$send(data, (void *)0, len);

  return status;
}

# 82 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Fifo.nc"
inline static cc2420_status_t CC2420TransmitP$TXFIFO$write(uint8_t * data, uint8_t length){
#line 82
  unsigned char __nesc_result;
#line 82

#line 82
  __nesc_result = CC2420SpiP$Fifo$write(CC2420_TXFIFO, data, length);
#line 82

#line 82
  return __nesc_result;
#line 82
}
#line 82
# 64 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(resource_client_id_t id)
#line 64
{
  /* atomic removed: atomic calls only */
#line 65
  {
    unsigned char __nesc_temp = 
#line 65
    /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$resQ[id] != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY || /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qTail == id;

#line 65
    return __nesc_temp;
  }
}

#line 82
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$enqueue(resource_client_id_t id)
#line 82
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 83
    {
      if (!/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(id)) {
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qHead = id;
            }
          else {
#line 88
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qTail] = id;
            }
#line 89
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qTail = id;
          {
            unsigned char __nesc_temp = 
#line 90
            SUCCESS;

            {
#line 90
              __nesc_atomic_end(__nesc_atomic); 
#line 90
              return __nesc_temp;
            }
          }
        }
#line 92
      {
        unsigned char __nesc_temp = 
#line 92
        EBUSY;

        {
#line 92
          __nesc_atomic_end(__nesc_atomic); 
#line 92
          return __nesc_temp;
        }
      }
    }
#line 95
    __nesc_atomic_end(__nesc_atomic); }
}

# 79 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static error_t CC2420TinyosNetworkP$Queue$enqueue(resource_client_id_t id){
#line 79
  unsigned char __nesc_result;
#line 79

#line 79
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$enqueue(id);
#line 79

#line 79
  return __nesc_result;
#line 79
}
#line 79
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420TinyosNetworkP$grantTask$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420TinyosNetworkP$grantTask);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 199 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP$Resource$request(uint8_t id)
#line 199
{

  CC2420TinyosNetworkP$grantTask$postTask();

  if (CC2420TinyosNetworkP$TINYOS_N_NETWORKS > 1) {
      return CC2420TinyosNetworkP$Queue$enqueue(id);
    }
  else 
#line 205
    {
      if (id == CC2420TinyosNetworkP$resource_owner) {
          return EALREADY;
        }
      else 
#line 208
        {
          CC2420TinyosNetworkP$next_owner = id;
          return SUCCESS;
        }
    }
}

# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP$RadioResource$request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420TinyosNetworkP$Resource$request(CC2420ActiveMessageC$CC2420_AM_SEND_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 229 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline error_t CC2420TinyosNetworkP$Resource$release(uint8_t id)
#line 229
{
  if (CC2420TinyosNetworkP$TINYOS_N_NETWORKS > 1) {
      CC2420TinyosNetworkP$grantTask$postTask();
    }
  CC2420TinyosNetworkP$resource_owner = CC2420TinyosNetworkP$OWNER_NONE;
  return SUCCESS;
}

#line 253
static inline void CC2420TinyosNetworkP$Resource$default$granted(uint8_t client)
#line 253
{
  CC2420TinyosNetworkP$Resource$release(client);
}

# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void CC2420TinyosNetworkP$Resource$granted(uint8_t arg_0x2b4801d92280){
#line 102
  switch (arg_0x2b4801d92280) {
#line 102
    case CC2420ActiveMessageC$CC2420_AM_SEND_ID:
#line 102
      CC2420ActiveMessageP$RadioResource$granted();
#line 102
      break;
#line 102
    default:
#line 102
      CC2420TinyosNetworkP$Resource$default$granted(arg_0x2b4801d92280);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 68 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void )
#line 68
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 69
    {
      if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qHead != /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY) {
          uint8_t id = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qHead;

#line 72
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qHead = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$resQ[/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qHead];
          if (/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY) {
            /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qTail = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY;
            }
#line 75
          /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$resQ[id] = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY;
          {
            unsigned char __nesc_temp = 
#line 76
            id;

            {
#line 76
              __nesc_atomic_end(__nesc_atomic); 
#line 76
              return __nesc_temp;
            }
          }
        }
#line 78
      {
        unsigned char __nesc_temp = 
#line 78
        /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY;

        {
#line 78
          __nesc_atomic_end(__nesc_atomic); 
#line 78
          return __nesc_temp;
        }
      }
    }
#line 81
    __nesc_atomic_end(__nesc_atomic); }
}

# 70 "/opt/tinyos-2.1.2/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t CC2420TinyosNetworkP$Queue$dequeue(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$dequeue();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 180 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP$grantTask$runTask(void )
#line 180
{


  if (CC2420TinyosNetworkP$TINYOS_N_NETWORKS > 1) {
      if (CC2420TinyosNetworkP$resource_owner == CC2420TinyosNetworkP$OWNER_NONE && !CC2420TinyosNetworkP$Queue$isEmpty()) {
          CC2420TinyosNetworkP$resource_owner = CC2420TinyosNetworkP$Queue$dequeue();

          if (CC2420TinyosNetworkP$resource_owner != CC2420TinyosNetworkP$OWNER_NONE) {
              CC2420TinyosNetworkP$Resource$granted(CC2420TinyosNetworkP$resource_owner);
            }
        }
    }
  else 
#line 191
    {
      if (CC2420TinyosNetworkP$next_owner != CC2420TinyosNetworkP$resource_owner) {
          CC2420TinyosNetworkP$resource_owner = CC2420TinyosNetworkP$next_owner;
          CC2420TinyosNetworkP$Resource$granted(CC2420TinyosNetworkP$resource_owner);
        }
    }
}

# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * CC2420TinyosNetworkP$CC2420PacketBody$getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP$CC2420PacketBody$getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 138 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void *CC2420TinyosNetworkP$BareSend$getPayload(message_t *msg, uint8_t len)
#line 138
{

  cc2420_header_t *hdr = CC2420TinyosNetworkP$CC2420PacketBody$getHeader(msg);

#line 141
  return hdr;
}

#line 241
static inline message_t *CC2420TinyosNetworkP$BareReceive$default$receive(message_t *msg, void *payload, uint8_t len)
#line 241
{
  return msg;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP$BareReceive$receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP$BareReceive$default$receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 283 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP$Snoop$default$receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 283
{
  return msg;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP$Snoop$receive(am_id_t arg_0x2b4801e17480, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
    __nesc_result = CC2420ActiveMessageP$Snoop$default$receive(arg_0x2b4801e17480, msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 109 "/opt/tinyos-2.1.2/tos/system/LedsP.nc"
static inline void LedsP$Leds$led2Off(void )
#line 109
{
  LedsP$Led2$set();
  ;
#line 111
  ;
}

# 94 "/opt/tinyos-2.1.2/tos/interfaces/Leds.nc"
inline static void MoteToMoteC$Leds$led2Off(void ){
#line 94
  LedsP$Leds$led2Off();
#line 94
}
#line 94
# 41 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$clr(void ){
#line 41
  HalPXA27xGeneralIOM$GeneralIO$clr(105);
#line 41
}
#line 41
# 104 "/opt/tinyos-2.1.2/tos/system/LedsP.nc"
static inline void LedsP$Leds$led2On(void )
#line 104
{
  LedsP$Led2$clr();
  ;
#line 106
  ;
}

# 89 "/opt/tinyos-2.1.2/tos/interfaces/Leds.nc"
inline static void MoteToMoteC$Leds$led2On(void ){
#line 89
  LedsP$Leds$led2On();
#line 89
}
#line 89
# 77 "MoteToMoteC.nc"
static inline message_t *MoteToMoteC$Receive$receive(message_t *msg, void *payload, uint8_t len)
{
  if (len == sizeof(MoteToMoteMsg_t )) 
    {
      MoteToMoteMsg_t *incomingPacket = (MoteToMoteMsg_t *)payload;



      uint8_t data = __nesc_ntoh_uint8(incomingPacket->Data.nxdata);

      if ((data = 1)) 
        {
          MoteToMoteC$Leds$led2On();
        }
      else 
        {
          MoteToMoteC$Leds$led2Off();
        }
    }
  return msg;
}

# 279 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline message_t *CC2420ActiveMessageP$Receive$default$receive(am_id_t id, message_t *msg, void *payload, uint8_t len)
#line 279
{
  return msg;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * CC2420ActiveMessageP$Receive$receive(am_id_t arg_0x2b4801e18908, message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  switch (arg_0x2b4801e18908) {
#line 78
    case 6:
#line 78
      __nesc_result = MoteToMoteC$Receive$receive(msg, payload, len);
#line 78
      break;
#line 78
    default:
#line 78
      __nesc_result = CC2420ActiveMessageP$Receive$default$receive(arg_0x2b4801e18908, msg, payload, len);
#line 78
      break;
#line 78
    }
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 72 "/opt/tinyos-2.1.2/tos/system/ActiveMessageAddressC.nc"
static inline am_addr_t ActiveMessageAddressC$ActiveMessageAddress$amAddress(void )
#line 72
{
  return ActiveMessageAddressC$amAddress();
}

# 50 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
inline static am_addr_t CC2420ActiveMessageP$ActiveMessageAddress$amAddress(void ){
#line 50
  unsigned short __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC$ActiveMessageAddress$amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 135 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline am_addr_t CC2420ActiveMessageP$AMPacket$address(void )
#line 135
{
  return CC2420ActiveMessageP$ActiveMessageAddress$amAddress();
}

#line 159
static inline bool CC2420ActiveMessageP$AMPacket$isForMe(message_t *amsg)
#line 159
{
  return CC2420ActiveMessageP$AMPacket$destination(amsg) == CC2420ActiveMessageP$AMPacket$address() || 
  CC2420ActiveMessageP$AMPacket$destination(amsg) == AM_BROADCAST_ADDR;
}

#line 219
static inline message_t *CC2420ActiveMessageP$SubReceive$receive(message_t *msg, void *payload, uint8_t len)
#line 219
{

  if (CC2420ActiveMessageP$AMPacket$isForMe(msg)) {
      return CC2420ActiveMessageP$Receive$receive(CC2420ActiveMessageP$AMPacket$type(msg), msg, payload, len);
    }
  else {
      return CC2420ActiveMessageP$Snoop$receive(CC2420ActiveMessageP$AMPacket$type(msg), msg, payload, len);
    }
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * CC2420TinyosNetworkP$ActiveReceive$receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420ActiveMessageP$SubReceive$receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420TinyosNetworkP$CC2420PacketBody$getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP$CC2420PacketBody$getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 119 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static inline uint8_t CC2420PacketP$CC2420Packet$getNetwork(message_t * p_msg)
#line 119
{



  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      unsigned char __nesc_temp = 
#line 124
      *CC2420PacketP$getNetwork(p_msg);

      {
#line 124
        __nesc_atomic_end(__nesc_atomic); 
#line 124
        return __nesc_temp;
      }
    }
#line 126
    __nesc_atomic_end(__nesc_atomic); }
}

# 75 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Packet.nc"
inline static uint8_t CC2420TinyosNetworkP$CC2420Packet$getNetwork(message_t * p_msg){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = CC2420PacketP$CC2420Packet$getNetwork(p_msg);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 157 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline message_t *CC2420TinyosNetworkP$SubReceive$receive(message_t *msg, void *payload, uint8_t len)
#line 157
{
  uint8_t network = CC2420TinyosNetworkP$CC2420Packet$getNetwork(msg);

  if (! __nesc_ntoh_int8(CC2420TinyosNetworkP$CC2420PacketBody$getMetadata(msg)->crc.nxdata)) {
      return msg;
    }

  if (network == 0x3f) {
      return CC2420TinyosNetworkP$ActiveReceive$receive(msg, payload, len);
    }
  else 
#line 166
    {
      return CC2420TinyosNetworkP$BareReceive$receive(msg, 
      CC2420TinyosNetworkP$BareSend$getPayload(msg, len), 
      len + sizeof(cc2420_header_t ));
    }
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP$Receive$receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = CC2420TinyosNetworkP$SubReceive$receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 138 "/opt/tinyos-2.1.2/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline void UniqueReceiveP$insert(uint16_t msgSource, uint8_t msgDsn)
#line 138
{
  uint8_t element = UniqueReceiveP$recycleSourceElement;
  bool increment = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 142
    {
      if (element == UniqueReceiveP$INVALID_ELEMENT || UniqueReceiveP$writeIndex == element) {

          element = UniqueReceiveP$writeIndex;
          increment = TRUE;
        }

      UniqueReceiveP$receivedMessages[element].source = msgSource;
      UniqueReceiveP$receivedMessages[element].dsn = msgDsn;
      if (increment) {
          UniqueReceiveP$writeIndex++;
          UniqueReceiveP$writeIndex %= 4;
        }
    }
#line 155
    __nesc_atomic_end(__nesc_atomic); }
}

#line 192
static inline message_t *UniqueReceiveP$DuplicateReceive$default$receive(message_t *msg, void *payload, uint8_t len)
#line 192
{
  return msg;
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * UniqueReceiveP$DuplicateReceive$receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = UniqueReceiveP$DuplicateReceive$default$receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 112 "/opt/tinyos-2.1.2/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline bool UniqueReceiveP$hasSeen(uint16_t msgSource, uint8_t msgDsn)
#line 112
{
  int i;

#line 114
  UniqueReceiveP$recycleSourceElement = UniqueReceiveP$INVALID_ELEMENT;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 116
    {
      for (i = 0; i < 4; i++) {
          if (UniqueReceiveP$receivedMessages[i].source == msgSource) {
              if (UniqueReceiveP$receivedMessages[i].dsn == msgDsn) {

                  {
                    unsigned char __nesc_temp = 
#line 121
                    TRUE;

                    {
#line 121
                      __nesc_atomic_end(__nesc_atomic); 
#line 121
                      return __nesc_temp;
                    }
                  }
                }
#line 124
              UniqueReceiveP$recycleSourceElement = i;
            }
        }
    }
#line 127
    __nesc_atomic_end(__nesc_atomic); }

  return FALSE;
}

# 42 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_header_t * UniqueReceiveP$CC2420PacketBody$getHeader(message_t * msg){
#line 42
  nx_struct cc2420_header_t *__nesc_result;
#line 42

#line 42
  __nesc_result = CC2420PacketP$CC2420PacketBody$getHeader(msg);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 165 "/opt/tinyos-2.1.2/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline uint16_t UniqueReceiveP$getSourceKey(message_t * msg)
#line 165
{
  cc2420_header_t *hdr = UniqueReceiveP$CC2420PacketBody$getHeader(msg);
  int s_mode = (__nesc_ntoh_leuint16(hdr->fcf.nxdata) >> IEEE154_FCF_SRC_ADDR_MODE) & 0x3;
  int d_mode = (__nesc_ntoh_leuint16(hdr->fcf.nxdata) >> IEEE154_FCF_DEST_ADDR_MODE) & 0x3;
  int s_offset = 2;
#line 169
  int s_len = 2;
  uint16_t key = 0;
  uint8_t *current = (uint8_t *)& hdr->dest;
  int i;

  if (s_mode == IEEE154_ADDR_EXT) {
      s_len = 8;
    }
  if (d_mode == IEEE154_ADDR_EXT) {
      s_offset = 8;
    }

  current += s_offset;

  for (i = 0; i < s_len; i++) {
      key += current[i];
    }
  return key;
}

#line 86
static inline message_t *UniqueReceiveP$SubReceive$receive(message_t *msg, void *payload, 
uint8_t len)
#line 87
{

  uint16_t msgSource = UniqueReceiveP$getSourceKey(msg);
  uint8_t msgDsn = __nesc_ntoh_leuint8(UniqueReceiveP$CC2420PacketBody$getHeader(msg)->dsn.nxdata);

  if (UniqueReceiveP$hasSeen(msgSource, msgDsn)) {
      return UniqueReceiveP$DuplicateReceive$receive(msg, payload, len);
    }
  else 
#line 94
    {
      UniqueReceiveP$insert(msgSource, msgDsn);
      return UniqueReceiveP$Receive$receive(msg, payload, len);
    }
}

# 78 "/opt/tinyos-2.1.2/tos/interfaces/Receive.nc"
inline static message_t * CC2420ReceiveP$Receive$receive(message_t * msg, void * payload, uint8_t len){
#line 78
  nx_struct message_t *__nesc_result;
#line 78

#line 78
  __nesc_result = UniqueReceiveP$SubReceive$receive(msg, payload, len);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 298 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline ieee_eui64_t CC2420ControlP$CC2420Config$getExtAddr(void )
#line 298
{
  return CC2420ControlP$m_ext_addr;
}

# 66 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static ieee_eui64_t CC2420ReceiveP$CC2420Config$getExtAddr(void ){
#line 66
  struct ieee_eui64 __nesc_result;
#line 66

#line 66
  __nesc_result = CC2420ControlP$CC2420Config$getExtAddr();
#line 66

#line 66
  return __nesc_result;
#line 66
}
#line 66
# 355 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline bool CC2420ControlP$CC2420Config$isAddressRecognitionEnabled(void )
#line 355
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 356
    {
      unsigned char __nesc_temp = 
#line 356
      CC2420ControlP$addressRecognition;

      {
#line 356
        __nesc_atomic_end(__nesc_atomic); 
#line 356
        return __nesc_temp;
      }
    }
#line 358
    __nesc_atomic_end(__nesc_atomic); }
}

# 93 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static bool CC2420ReceiveP$CC2420Config$isAddressRecognitionEnabled(void ){
#line 93
  unsigned char __nesc_result;
#line 93

#line 93
  __nesc_result = CC2420ControlP$CC2420Config$isAddressRecognitionEnabled();
#line 93

#line 93
  return __nesc_result;
#line 93
}
#line 93
# 824 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline bool CC2420ReceiveP$passesAddressCheck(message_t *msg)
#line 824
{
  cc2420_header_t *header = CC2420ReceiveP$CC2420PacketBody$getHeader(msg);
  int mode = (__nesc_ntoh_leuint16(header->fcf.nxdata) >> IEEE154_FCF_DEST_ADDR_MODE) & 3;
  ieee_eui64_t *ext_addr;

  if (!CC2420ReceiveP$CC2420Config$isAddressRecognitionEnabled()) {
      return TRUE;
    }

  if (mode == IEEE154_ADDR_SHORT) {
      return __nesc_ntoh_leuint16(header->dest.nxdata) == CC2420ReceiveP$CC2420Config$getShortAddr()
       || __nesc_ntoh_leuint16(header->dest.nxdata) == IEEE154_BROADCAST_ADDR;
    }
  else {
#line 836
    if (mode == IEEE154_ADDR_EXT) {
        ieee_eui64_t local_addr = CC2420ReceiveP$CC2420Config$getExtAddr();

#line 838
        ext_addr = (ieee_eui64_t * )& header->dest;
        return memcmp(ext_addr->data, local_addr.data, IEEE_EUI64_LENGTH) == 0;
      }
    else 
#line 840
      {

        return FALSE;
      }
    }
}

# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420PacketBody.nc"
inline static cc2420_metadata_t * CC2420ReceiveP$CC2420PacketBody$getMetadata(message_t * msg){
#line 53
  nx_struct cc2420_metadata_t *__nesc_result;
#line 53

#line 53
  __nesc_result = CC2420PacketP$CC2420PacketBody$getMetadata(msg);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 676 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP$receiveDone_task$runTask(void )
#line 676
{
  cc2420_metadata_t *metadata = CC2420ReceiveP$CC2420PacketBody$getMetadata(CC2420ReceiveP$m_p_rx_buf);
  cc2420_header_t *header = CC2420ReceiveP$CC2420PacketBody$getHeader(CC2420ReceiveP$m_p_rx_buf);
  uint8_t length = __nesc_ntoh_leuint8(header->length.nxdata);
  uint8_t tmpLen __attribute((unused))  = sizeof(message_t ) - ((size_t )& ((message_t *)0)->data - sizeof(cc2420_header_t ));
  uint8_t * buf = (uint8_t * )header;

  __nesc_hton_int8(metadata->crc.nxdata, buf[length] >> 7);
  __nesc_hton_uint8(metadata->lqi.nxdata, buf[length] & 0x7f);
  __nesc_hton_uint8(metadata->rssi.nxdata, buf[length - 1]);

  if (CC2420ReceiveP$passesAddressCheck(CC2420ReceiveP$m_p_rx_buf) && length >= CC2420_SIZE) {
#line 701
      CC2420ReceiveP$m_p_rx_buf = CC2420ReceiveP$Receive$receive(CC2420ReceiveP$m_p_rx_buf, CC2420ReceiveP$m_p_rx_buf->data, 
      length - CC2420_SIZE);
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 704
    CC2420ReceiveP$receivingPacket = FALSE;
#line 704
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ReceiveP$waitForNextPacket();
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420SpiP$grant$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420SpiP$grant);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 184 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP$SpiResource$granted(void )
#line 184
{
  CC2420SpiP$grant$postTask();
}

# 165 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$default$granted(uint8_t id)
#line 165
{
}

# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$granted(uint8_t arg_0x2b48017dee30){
#line 102
  switch (arg_0x2b48017dee30) {
#line 102
    case /*CC2420SpiWireC.HplCC2420SpiC*/HplCC2420SpiC$0$SPI_CLIENT_ID:
#line 102
      CC2420SpiP$SpiResource$granted();
#line 102
      break;
#line 102
    default:
#line 102
      /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$default$granted(arg_0x2b48017dee30);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 155 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static inline void /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void )
#line 155
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    {
      /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$reqResId;
      /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$state = /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$RES_BUSY;
    }
#line 159
    __nesc_atomic_end(__nesc_atomic); }
  /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId);
  /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$granted(/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId);
}

# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP$TXCTRL$write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP$Reg$write(CC2420_TXCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 533 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP$writeTxctrl(void )
#line 533
{
  /* atomic removed: atomic calls only */
#line 534
  {
    CC2420ControlP$TXCTRL$write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
    3 << CC2420_TXCTRL_PA_CURRENT)) | (
    1 << CC2420_TXCTRL_RESERVED)) | ((
    31 & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
  }
}

# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP$RXCTRL1$write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP$Reg$write(CC2420_RXCTRL1, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t CC2420ControlP$IOCFG0$write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP$Reg$write(CC2420_IOCFG0, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP$SXOSCON$strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP$Strobe$strobe(CC2420_SXOSCON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 53 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ControlP$InterruptCCA$enableRisingEdge(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = HalPXA27xGeneralIOM$GpioInterrupt$enableRisingEdge(116);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 224 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP$CC2420Power$startOscillator(void )
#line 224
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 225
    {
      if (CC2420ControlP$m_state != CC2420ControlP$S_VREG_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 227
            FAIL;

            {
#line 227
              __nesc_atomic_end(__nesc_atomic); 
#line 227
              return __nesc_temp;
            }
          }
        }
#line 230
      CC2420ControlP$m_state = CC2420ControlP$S_XOSC_STARTING;
      CC2420ControlP$IOCFG1$write(CC2420_SFDMUX_XOSC16M_STABLE << 
      CC2420_IOCFG1_CCAMUX);

      CC2420ControlP$InterruptCCA$enableRisingEdge();
      CC2420ControlP$SXOSCON$strobe();

      CC2420ControlP$IOCFG0$write((1 << CC2420_IOCFG0_FIFOP_POLARITY) | (
      127 << CC2420_IOCFG0_FIFOP_THR));

      CC2420ControlP$writeFsctrl();
      CC2420ControlP$writeMdmctrl0();

      CC2420ControlP$RXCTRL1$write(((((((1 << CC2420_RXCTRL1_RXBPF_LOCUR) | (
      1 << CC2420_RXCTRL1_LOW_LOWGAIN)) | (
      1 << CC2420_RXCTRL1_HIGH_HGM)) | (
      1 << CC2420_RXCTRL1_LNA_CAP_ARRAY)) | (
      1 << CC2420_RXCTRL1_RXMIX_TAIL)) | (
      1 << CC2420_RXCTRL1_RXMIX_VCM)) | (
      2 << CC2420_RXCTRL1_RXMIX_CURRENT));

      CC2420ControlP$writeTxctrl();
    }
#line 252
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 71 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP$CC2420Power$startOscillator(void ){
#line 71
  unsigned char __nesc_result;
#line 71

#line 71
  __nesc_result = CC2420ControlP$CC2420Power$startOscillator();
#line 71

#line 71
  return __nesc_result;
#line 71
}
#line 71
# 214 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP$Resource$granted(void )
#line 214
{
  CC2420CsmaP$CC2420Power$startOscillator();
}

# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void CC2420ControlP$Resource$granted(void ){
#line 102
  CC2420CsmaP$Resource$granted();
#line 102
}
#line 102
# 413 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP$SpiResource$granted(void )
#line 413
{
  CC2420ControlP$CSN$clr();
  CC2420ControlP$Resource$granted();
}

# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420ControlP$syncDone$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420ControlP$syncDone);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP$SyncResource$release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP$Resource$release(/*CC2420ControlC.SyncSpiC*/CC2420SpiC$1$CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 53 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Strobe.nc"
inline static cc2420_status_t CC2420ControlP$SRXON$strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP$Strobe$strobe(CC2420_SRXON);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
inline static cc2420_status_t CC2420ControlP$SRFOFF$strobe(void ){
#line 53
  unsigned char __nesc_result;
#line 53

#line 53
  __nesc_result = CC2420SpiP$Strobe$strobe(CC2420_SRFOFF);
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 399 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP$SyncResource$granted(void )
#line 399
{
  CC2420ControlP$CSN$clr();
  CC2420ControlP$SRFOFF$strobe();
  CC2420ControlP$writeFsctrl();
  CC2420ControlP$writeMdmctrl0();
  CC2420ControlP$writeId();
  CC2420ControlP$CSN$set();
  CC2420ControlP$CSN$clr();
  CC2420ControlP$SRXON$strobe();
  CC2420ControlP$CSN$set();
  CC2420ControlP$SyncResource$release();
  CC2420ControlP$syncDone$postTask();
}

#line 545
static inline void CC2420ControlP$ReadRssi$default$readDone(error_t error, uint16_t data)
#line 545
{
}

# 63 "/opt/tinyos-2.1.2/tos/interfaces/Read.nc"
inline static void CC2420ControlP$ReadRssi$readDone(error_t result, CC2420ControlP$ReadRssi$val_t val){
#line 63
  CC2420ControlP$ReadRssi$default$readDone(result, val);
#line 63
}
#line 63
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP$RssiResource$release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP$Resource$release(/*CC2420ControlC.RssiResource*/CC2420SpiC$2$CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 287 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline cc2420_status_t CC2420SpiP$Reg$read(uint8_t addr, uint16_t *data)
#line 287
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 291
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 293
            status;

            {
#line 293
              __nesc_atomic_end(__nesc_atomic); 
#line 293
              return __nesc_temp;
            }
          }
        }
    }
#line 297
    __nesc_atomic_end(__nesc_atomic); }
#line 297
  status = CC2420SpiP$SpiByte$write(addr | 0x40);
  *data = (uint16_t )CC2420SpiP$SpiByte$write(0) << 8;
  *data |= CC2420SpiP$SpiByte$write(0);

  return status;
}

# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP$RSSI$read(uint16_t *data){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = CC2420SpiP$Reg$read(CC2420_RSSI, data);
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 418 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP$RssiResource$granted(void )
#line 418
{
  uint16_t data = 0;

#line 420
  CC2420ControlP$CSN$clr();
  CC2420ControlP$RSSI$read(&data);
  CC2420ControlP$CSN$set();

  CC2420ControlP$RssiResource$release();
  data += 0x7f;
  data &= 0x00ff;
  CC2420ControlP$ReadRssi$readDone(SUCCESS, data);
}

# 416 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline void CC2420TransmitP$SpiResource$granted(void )
#line 416
{
  uint8_t cur_state;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 419
    {
      cur_state = CC2420TransmitP$m_state;
    }
#line 421
    __nesc_atomic_end(__nesc_atomic); }

  switch (cur_state) {
      case CC2420TransmitP$S_LOAD: 
        CC2420TransmitP$loadTXFIFO();
      break;

      case CC2420TransmitP$S_BEGIN_TRANSMIT: 
        CC2420TransmitP$attemptSend();
      break;

      case CC2420TransmitP$S_CANCEL: 
        CC2420TransmitP$CSN$clr();
      CC2420TransmitP$SFLUSHTX$strobe();
      CC2420TransmitP$CSN$set();
      CC2420TransmitP$releaseSpiResource();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 437
        {
          CC2420TransmitP$m_state = CC2420TransmitP$S_STARTED;
        }
#line 439
        __nesc_atomic_end(__nesc_atomic); }
      CC2420TransmitP$Send$sendDone(CC2420TransmitP$m_msg, ECANCEL);
      break;

      default: 
        CC2420TransmitP$releaseSpiResource();
      break;
    }
}

# 513 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP$SpiResource$granted(void )
#line 513
{







  CC2420ReceiveP$receive();
}

# 367 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP$Resource$default$granted(uint8_t id)
#line 367
{
}

# 102 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static void CC2420SpiP$Resource$granted(uint8_t arg_0x2b4801741158){
#line 102
  switch (arg_0x2b4801741158) {
#line 102
    case /*CC2420ControlC.Spi*/CC2420SpiC$0$CLIENT_ID:
#line 102
      CC2420ControlP$SpiResource$granted();
#line 102
      break;
#line 102
    case /*CC2420ControlC.SyncSpiC*/CC2420SpiC$1$CLIENT_ID:
#line 102
      CC2420ControlP$SyncResource$granted();
#line 102
      break;
#line 102
    case /*CC2420ControlC.RssiResource*/CC2420SpiC$2$CLIENT_ID:
#line 102
      CC2420ControlP$RssiResource$granted();
#line 102
      break;
#line 102
    case /*CC2420TransmitC.Spi*/CC2420SpiC$3$CLIENT_ID:
#line 102
      CC2420TransmitP$SpiResource$granted();
#line 102
      break;
#line 102
    case /*CC2420ReceiveC.Spi*/CC2420SpiC$4$CLIENT_ID:
#line 102
      CC2420ReceiveP$SpiResource$granted();
#line 102
      break;
#line 102
    default:
#line 102
      CC2420SpiP$Resource$default$granted(arg_0x2b4801741158);
#line 102
      break;
#line 102
    }
#line 102
}
#line 102
# 358 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static inline void CC2420SpiP$grant$runTask(void )
#line 358
{
  uint8_t holder;

#line 360
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
    {
      holder = CC2420SpiP$m_holder;
    }
#line 362
    __nesc_atomic_end(__nesc_atomic); }
  CC2420SpiP$Resource$granted(holder);
}

# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Register.nc"
inline static cc2420_status_t CC2420ControlP$FSCTRL$write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP$Reg$write(CC2420_FSCTRL, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
inline static cc2420_status_t CC2420ControlP$MDMCTRL0$write(uint16_t data){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420SpiP$Reg$write(CC2420_MDMCTRL0, data);
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 56 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$lateAlarm$runTask(void )
#line 56
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    {
      /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mfRunning = FALSE;
      /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$fired();
    }
#line 60
    __nesc_atomic_end(__nesc_atomic); }
}

# 235 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$CC2420Config$syncDone(error_t error)
#line 235
{
}

# 709 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline void CC2420ReceiveP$CC2420Config$syncDone(error_t error)
#line 709
{
}

# 55 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Config.nc"
inline static void CC2420ControlP$CC2420Config$syncDone(error_t error){
#line 55
  CC2420ReceiveP$CC2420Config$syncDone(error);
#line 55
  CC2420ActiveMessageP$CC2420Config$syncDone(error);
#line 55
}
#line 55
# 469 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline void CC2420ControlP$syncDone$runTask(void )
#line 469
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 470
    CC2420ControlP$m_sync_busy = FALSE;
#line 470
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP$CC2420Config$syncDone(SUCCESS);
}

# 88 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP$SyncResource$request(void ){
#line 88
  unsigned char __nesc_result;
#line 88

#line 88
  __nesc_result = CC2420SpiP$Resource$request(/*CC2420ControlC.SyncSpiC*/CC2420SpiC$1$CLIENT_ID);
#line 88

#line 88
  return __nesc_result;
#line 88
}
#line 88
# 323 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP$CC2420Config$sync(void )
#line 323
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 324
    {
      if (CC2420ControlP$m_sync_busy) {
          {
            unsigned char __nesc_temp = 
#line 326
            FAIL;

            {
#line 326
              __nesc_atomic_end(__nesc_atomic); 
#line 326
              return __nesc_temp;
            }
          }
        }
#line 329
      CC2420ControlP$m_sync_busy = TRUE;
      if (CC2420ControlP$m_state == CC2420ControlP$S_XOSC_STARTED) {
          CC2420ControlP$SyncResource$request();
        }
      else 
#line 332
        {
          CC2420ControlP$syncDone$postTask();
        }
    }
#line 335
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

#line 465
static inline void CC2420ControlP$sync$runTask(void )
#line 465
{
  CC2420ControlP$CC2420Config$sync();
}

# 244 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP$BareSend$default$sendDone(message_t *msg, error_t error)
#line 244
{
}

# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP$BareSend$sendDone(message_t * msg, error_t error){
#line 100
  CC2420TinyosNetworkP$BareSend$default$sendDone(msg, error);
#line 100
}
#line 100
# 110 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static void CC2420ActiveMessageP$AMSend$sendDone(am_id_t arg_0x2b4801e197d8, message_t * msg, error_t error){
#line 110
  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$sendDone(arg_0x2b4801e197d8, msg, error);
#line 110
}
#line 110
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420ActiveMessageP$RadioResource$release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420TinyosNetworkP$Resource$release(CC2420ActiveMessageC$CC2420_AM_SEND_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 212 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$SubSend$sendDone(message_t *msg, error_t result)
#line 212
{
  CC2420ActiveMessageP$RadioResource$release();
  CC2420ActiveMessageP$AMSend$sendDone(CC2420ActiveMessageP$AMPacket$type(msg), msg, result);
}

# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static void CC2420TinyosNetworkP$ActiveSend$sendDone(message_t * msg, error_t error){
#line 100
  CC2420ActiveMessageP$SubSend$sendDone(msg, error);
#line 100
}
#line 100
# 148 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void CC2420TinyosNetworkP$SubSend$sendDone(message_t *msg, error_t error)
#line 148
{
  if (CC2420TinyosNetworkP$m_busy_client == CC2420TinyosNetworkP$CLIENT_AM) {
      CC2420TinyosNetworkP$ActiveSend$sendDone(msg, error);
    }
  else 
#line 151
    {
      CC2420TinyosNetworkP$BareSend$sendDone(msg, error);
    }
}

# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static void UniqueSendP$Send$sendDone(message_t * msg, error_t error){
#line 100
  CC2420TinyosNetworkP$SubSend$sendDone(msg, error);
#line 100
}
#line 100
# 104 "/opt/tinyos-2.1.2/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline void UniqueSendP$SubSend$sendDone(message_t *msg, error_t error)
#line 104
{
  UniqueSendP$State$toIdle();
  UniqueSendP$Send$sendDone(msg, error);
}

# 100 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static void CC2420CsmaP$Send$sendDone(message_t * msg, error_t error){
#line 100
  UniqueSendP$SubSend$sendDone(msg, error);
#line 100
}
#line 100
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t CC2420CsmaP$stopDone_task$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(CC2420CsmaP$stopDone_task);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP$RSTN$set(void ){
#line 40
  HalPXA27xGeneralIOM$GeneralIO$set(22);
#line 40
}
#line 40

inline static void CC2420ControlP$VREN$clr(void ){
#line 41
  HalPXA27xGeneralIOM$GeneralIO$clr(115);
#line 41
}
#line 41
inline static void CC2420ControlP$RSTN$clr(void ){
#line 41
  HalPXA27xGeneralIOM$GeneralIO$clr(22);
#line 41
}
#line 41
# 216 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP$CC2420Power$stopVReg(void )
#line 216
{
  CC2420ControlP$m_state = CC2420ControlP$S_VREG_STOPPED;
  CC2420ControlP$RSTN$clr();
  CC2420ControlP$VREN$clr();
  CC2420ControlP$RSTN$set();
  return SUCCESS;
}

# 63 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP$CC2420Power$stopVReg(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = CC2420ControlP$CC2420Power$stopVReg();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP$InterruptFIFOP$disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = HalPXA27xGeneralIOM$GpioInterrupt$disable(0);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 171 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP$StdControl$stop(void )
#line 171
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
      CC2420ReceiveP$m_state = CC2420ReceiveP$S_STOPPED;
      CC2420ReceiveP$reset_state();
      CC2420ReceiveP$CSN$set();
      CC2420ReceiveP$InterruptFIFOP$disable();
    }
#line 177
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 61 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$disable(void ){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = HalPXA27xGeneralIOM$GpioInterrupt$disable(16);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 67 "/opt/tinyos-2.1.2/tos/lib/gpio/SoftCaptureP.nc"
static inline void /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$disable(void )
#line 67
{
  /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioInterrupt$disable();
  return;
}

# 66 "/opt/tinyos-2.1.2/tos/interfaces/GpioCapture.nc"
inline static void CC2420TransmitP$CaptureSFD$disable(void ){
#line 66
  /*HplCC2420InterruptsC.SoftCaptureC.SoftCaptureP*/SoftCaptureP$0$GpioCapture$disable();
#line 66
}
#line 66
# 179 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP$StdControl$stop(void )
#line 179
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 180
    {
      CC2420TransmitP$m_state = CC2420TransmitP$S_STOPPED;
      CC2420TransmitP$BackoffTimer$stop();
      CC2420TransmitP$CaptureSFD$disable();
      CC2420TransmitP$SpiResource$release();
      CC2420TransmitP$CSN$set();
    }
#line 186
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 105 "/opt/tinyos-2.1.2/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP$SubControl$stop(void ){
#line 105
  unsigned char __nesc_result;
#line 105

#line 105
  __nesc_result = CC2420TransmitP$StdControl$stop();
#line 105
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP$StdControl$stop());
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
# 275 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP$shutdown(void )
#line 275
{
  CC2420CsmaP$SubControl$stop();
  CC2420CsmaP$CC2420Power$stopVReg();
  CC2420CsmaP$stopDone_task$postTask();
}

#line 244
static inline void CC2420CsmaP$sendDone_task$runTask(void )
#line 244
{
  error_t packetErr;

#line 246
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 246
    packetErr = CC2420CsmaP$sendErr;
#line 246
    __nesc_atomic_end(__nesc_atomic); }
  if (CC2420CsmaP$SplitControlState$isState(CC2420CsmaP$S_STOPPING)) {
      CC2420CsmaP$shutdown();
    }
  else {
      CC2420CsmaP$SplitControlState$forceState(CC2420CsmaP$S_STARTED);
    }

  CC2420CsmaP$Send$sendDone(CC2420CsmaP$m_msg, packetErr);
}

# 72 "MoteToMoteC.nc"
static inline void MoteToMoteC$AMControl$stopDone(error_t error)
{
}

# 138 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP$SplitControl$stopDone(error_t error){
#line 138
  MoteToMoteC$AMControl$stopDone(error);
#line 138
}
#line 138
# 265 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP$stopDone_task$runTask(void )
#line 265
{
  CC2420CsmaP$SplitControlState$forceState(CC2420CsmaP$S_STOPPED);
  CC2420CsmaP$SplitControl$stopDone(SUCCESS);
}

# 104 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static error_t MoteToMoteC$AMControl$start(void ){
#line 104
  unsigned char __nesc_result;
#line 104

#line 104
  __nesc_result = CC2420CsmaP$SplitControl$start();
#line 104

#line 104
  return __nesc_result;
#line 104
}
#line 104
# 41 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$clr(void ){
#line 41
  HalPXA27xGeneralIOM$GeneralIO$clr(104);
#line 41
}
#line 41
# 89 "/opt/tinyos-2.1.2/tos/system/LedsP.nc"
static inline void LedsP$Leds$led1On(void )
#line 89
{
  LedsP$Led1$clr();
  ;
#line 91
  ;
}

# 72 "/opt/tinyos-2.1.2/tos/interfaces/Leds.nc"
inline static void MoteToMoteC$Leds$led1On(void ){
#line 72
  LedsP$Leds$led1On();
#line 72
}
#line 72
# 60 "MoteToMoteC.nc"
static inline void MoteToMoteC$AMControl$startDone(error_t error)
{
  if (error == 0) 
    {
      MoteToMoteC$Leds$led1On();
    }
  else 
    {
      MoteToMoteC$AMControl$start();
    }
}

# 113 "/opt/tinyos-2.1.2/tos/interfaces/SplitControl.nc"
inline static void CC2420CsmaP$SplitControl$startDone(error_t error){
#line 113
  MoteToMoteC$AMControl$startDone(error);
#line 113
}
#line 113
# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420ControlP$SpiResource$release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420SpiP$Resource$release(/*CC2420ControlC.Spi*/CC2420SpiC$0$CLIENT_ID);
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 196 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP$Resource$release(void )
#line 196
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 197
    {
      CC2420ControlP$CSN$set();
      {
        unsigned char __nesc_temp = 
#line 199
        CC2420ControlP$SpiResource$release();

        {
#line 199
          __nesc_atomic_end(__nesc_atomic); 
#line 199
          return __nesc_temp;
        }
      }
    }
#line 202
    __nesc_atomic_end(__nesc_atomic); }
}

# 120 "/opt/tinyos-2.1.2/tos/interfaces/Resource.nc"
inline static error_t CC2420CsmaP$Resource$release(void ){
#line 120
  unsigned char __nesc_result;
#line 120

#line 120
  __nesc_result = CC2420ControlP$Resource$release();
#line 120

#line 120
  return __nesc_result;
#line 120
}
#line 120
# 268 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP$CC2420Power$rxOn(void )
#line 268
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 269
    {
      if (CC2420ControlP$m_state != CC2420ControlP$S_XOSC_STARTED) {
          {
            unsigned char __nesc_temp = 
#line 271
            FAIL;

            {
#line 271
              __nesc_atomic_end(__nesc_atomic); 
#line 271
              return __nesc_temp;
            }
          }
        }
#line 273
      CC2420ControlP$SRXON$strobe();
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 90 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP$CC2420Power$rxOn(void ){
#line 90
  unsigned char __nesc_result;
#line 90

#line 90
  __nesc_result = CC2420ControlP$CC2420Power$rxOn();
#line 90

#line 90
  return __nesc_result;
#line 90
}
#line 90
# 54 "/opt/tinyos-2.1.2/tos/interfaces/GpioInterrupt.nc"
inline static error_t CC2420ReceiveP$InterruptFIFOP$enableFallingEdge(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = HalPXA27xGeneralIOM$GpioInterrupt$enableFallingEdge(0);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 157 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP$StdControl$start(void )
#line 157
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 158
    {
      CC2420ReceiveP$reset_state();
      CC2420ReceiveP$m_state = CC2420ReceiveP$S_STARTED;
      CC2420ReceiveP$receivingPacket = FALSE;




      CC2420ReceiveP$InterruptFIFOP$enableFallingEdge();
    }
#line 167
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 168 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP$StdControl$start(void )
#line 168
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 169
    {
      CC2420TransmitP$CaptureSFD$captureRisingEdge();
      CC2420TransmitP$m_state = CC2420TransmitP$S_STARTED;
      CC2420TransmitP$m_receiving = FALSE;
      CC2420TransmitP$abortSpiRelease = FALSE;
      CC2420TransmitP$m_tx_power = 0;
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 95 "/opt/tinyos-2.1.2/tos/interfaces/StdControl.nc"
inline static error_t CC2420CsmaP$SubControl$start(void ){
#line 95
  unsigned char __nesc_result;
#line 95

#line 95
  __nesc_result = CC2420TransmitP$StdControl$start();
#line 95
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP$StdControl$start());
#line 95

#line 95
  return __nesc_result;
#line 95
}
#line 95
# 257 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static inline void CC2420CsmaP$startDone_task$runTask(void )
#line 257
{
  CC2420CsmaP$SubControl$start();
  CC2420CsmaP$CC2420Power$rxOn();
  CC2420CsmaP$Resource$release();
  CC2420CsmaP$SplitControlState$forceState(CC2420CsmaP$S_STARTED);
  CC2420CsmaP$SplitControl$startDone(SUCCESS);
}

# 45 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
inline static error_t CC2420CsmaP$SplitControlState$requestState(uint8_t reqState){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = StateImplP$State$requestState(1U, reqState);
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45
# 66 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void CC2420ControlP$StartupTimer$start(CC2420ControlP$StartupTimer$size_type dt){
#line 66
  /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$start(dt);
#line 66
}
#line 66
# 40 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP$VREN$set(void ){
#line 40
  HalPXA27xGeneralIOM$GeneralIO$set(115);
#line 40
}
#line 40
# 204 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP$CC2420Power$startVReg(void )
#line 204
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 205
    {
      if (CC2420ControlP$m_state != CC2420ControlP$S_VREG_STOPPED) {
          {
            unsigned char __nesc_temp = 
#line 207
            FAIL;

            {
#line 207
              __nesc_atomic_end(__nesc_atomic); 
#line 207
              return __nesc_temp;
            }
          }
        }
#line 209
      CC2420ControlP$m_state = CC2420ControlP$S_VREG_STARTING;
    }
#line 210
    __nesc_atomic_end(__nesc_atomic); }
  CC2420ControlP$VREN$set();
  CC2420ControlP$StartupTimer$start(CC2420_TIME_VREN);
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.2/tos/chips/cc2420/interfaces/CC2420Power.nc"
inline static error_t CC2420CsmaP$CC2420Power$startVReg(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = CC2420ControlP$CC2420Power$startVReg();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 56 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$runTask(void )
#line 56
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    {
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning = FALSE;
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$fired();
    }
#line 60
    __nesc_atomic_end(__nesc_atomic); }
}

# 64 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSCR(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSCR(4);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 152 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getNow(void )
#line 152
{
  return /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSCR();
}

# 109 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getNow(void ){
#line 109
  unsigned int __nesc_result;
#line 109

#line 109
  __nesc_result = /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getNow();
#line 109

#line 109
  return __nesc_result;
#line 109
}
#line 109
# 96 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$getNow(void )
{
#line 97
  return /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getNow();
}

# 136 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow(void ){
#line 136
  unsigned int __nesc_result;
#line 136

#line 136
  __nesc_result = /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$getNow();
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 139 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$fired(void )
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$fireTimers(/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow());
}

# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$fired(void ){
#line 83
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$fired();
#line 83
}
#line 83
# 114 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static inline uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSMR(uint8_t chnl_id)
{
  uint32_t val;

#line 117
  val = *(chnl_id < 4 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00000 + (uint32_t )(chnl_id << 2)) : & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00080 + (uint32_t )((chnl_id - 4) << 2)));
  return val;
}

# 78 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSMR(void ){
#line 78
  unsigned int __nesc_result;
#line 78

#line 78
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSMR(4);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 156 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getAlarm(void )
#line 156
{
  return /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSMR();
}

# 116 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getAlarm(void ){
#line 116
  unsigned int __nesc_result;
#line 116

#line 116
  __nesc_result = /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getAlarm();
#line 116

#line 116
  return __nesc_result;
#line 116
}
#line 116
#line 103
inline static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type dt){
#line 103
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$startAt(t0, dt);
#line 103
}
#line 103
# 58 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$startAt(t0, dt);
}










static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$start(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getAlarm(), /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_dt, FALSE);
    }
#line 78
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$fired();
}

# 71 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSMR(uint32_t val){
#line 71
  HplPXA27xOSTimerM$PXA27xOST$setOSMR(4, val);
#line 71
}
#line 71
#line 103
inline static bool /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSSRbit(void ){
#line 103
  unsigned char __nesc_result;
#line 103

#line 103
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSSRbit(4);
#line 103

#line 103
  return __nesc_result;
#line 103
}
#line 103
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 164 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(uint8_t num)
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$m_timers[num].isrunning = FALSE;
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void PMICM$chargeMonitorTimer$stop(void ){
#line 78
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(0U);
#line 78
}
#line 78
# 355 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline error_t PMICM$PMIC$getBatteryVoltage(uint8_t *val)
#line 355
{

  return PMICM$getPMICADCVal(0, val);
}

# 154 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, FALSE);
}

# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void PMICM$chargeMonitorTimer$startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(0U, dt);
#line 64
}
#line 64
# 372 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline error_t PMICM$PMIC$enableManualCharging(bool enable)
#line 372
{

  uint8_t val;

  if (enable) {

      PMICM$getPMICADCVal(2, &val);

      if (val > 75) {


          PMICM$writePMIC(0x2A, 8);

          PMICM$writePMIC(0x28, ((1 << 7) | ((1 & 0xF) << 3)) | (7 & 0x7));

          PMICM$writePMIC(0x20, 0x80);

          PMICM$chargeMonitorTimer$startPeriodic(300000);
        }
      else {
        }
    }
  else 
    {

      PMICM$PMIC$getBatteryVoltage(&val);


      PMICM$writePMIC(0x2A, 0);
      PMICM$writePMIC(0x28, 0);
      PMICM$writePMIC(0x20, 0x00);
    }
  return SUCCESS;
}

static inline void PMICM$chargeMonitorTimer$fired(void )
#line 407
{
  uint8_t val;

#line 409
  PMICM$PMIC$getBatteryVoltage(&val);

  if (val > 130) {
      PMICM$PMIC$enableManualCharging(FALSE);
      PMICM$chargeMonitorTimer$stop();
    }
  return;
}

# 315 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 315
{
  uint8_t *base = target;

#line 317
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

# 198 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$Packet$setPayloadLength(message_t *msg, uint8_t len)
#line 198
{
  __nesc_hton_leuint8(CC2420ActiveMessageP$CC2420PacketBody$getHeader(msg)->length.nxdata, len + CC2420_SIZE);
}

# 94 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
inline static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$setPayloadLength(message_t * msg, uint8_t len){
#line 94
  CC2420ActiveMessageP$Packet$setPayloadLength(msg, len);
#line 94
}
#line 94
# 90 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static inline error_t /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 91
{
  if (clientId >= 1) {
      return FAIL;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[clientId].msg = msg;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$setPayloadLength(msg, len);

  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current >= 1) {
      error_t err;
      am_id_t amId = /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$type(msg);
      am_addr_t dest = /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$destination(msg);

      ;
      /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current = clientId;

      err = /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current = 1;
          /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 75 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static error_t /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$send(message_t * msg, uint8_t len){
#line 75
  unsigned char __nesc_result;
#line 75

#line 75
  __nesc_result = /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$send(0U, msg, len);
#line 75

#line 75
  return __nesc_result;
#line 75
}
#line 75
# 169 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$AMPacket$setType(message_t *amsg, am_id_t type)
#line 169
{
  cc2420_header_t *header = CC2420ActiveMessageP$CC2420PacketBody$getHeader(amsg);

#line 171
  __nesc_hton_leuint8(header->type.nxdata, type);
}

# 162 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static void /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setType(message_t * amsg, am_id_t t){
#line 162
  CC2420ActiveMessageP$AMPacket$setType(amsg, t);
#line 162
}
#line 162
# 149 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void CC2420ActiveMessageP$AMPacket$setDestination(message_t *amsg, am_addr_t addr)
#line 149
{
  cc2420_header_t *header = CC2420ActiveMessageP$CC2420PacketBody$getHeader(amsg);

#line 151
  __nesc_hton_leuint16(header->dest.nxdata, addr);
}

# 103 "/opt/tinyos-2.1.2/tos/interfaces/AMPacket.nc"
inline static void /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setDestination(message_t * amsg, am_addr_t addr){
#line 103
  CC2420ActiveMessageP$AMPacket$setDestination(amsg, addr);
#line 103
}
#line 103
# 53 "/opt/tinyos-2.1.2/tos/system/AMQueueEntryP.nc"
static inline error_t /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 55
{
  /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setDestination(msg, dest);
  /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setType(msg, 6);
  return /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$send(msg, len);
}

# 80 "/opt/tinyos-2.1.2/tos/interfaces/AMSend.nc"
inline static error_t MoteToMoteC$AMSend$send(am_addr_t addr, message_t * msg, uint8_t len){
#line 80
  unsigned char __nesc_result;
#line 80

#line 80
  __nesc_result = /*MoteToMoteAppC.AMSenderC.SenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$send(addr, msg, len);
#line 80

#line 80
  return __nesc_result;
#line 80
}
#line 80
# 94 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static inline void *CC2420TinyosNetworkP$ActiveSend$getPayload(message_t *msg, uint8_t len)
#line 94
{
  if (len <= CC2420TinyosNetworkP$ActiveSend$maxPayloadLength()) {
      return msg->data;
    }
  else 
#line 97
    {
      return (void *)0;
    }
}

# 125 "/opt/tinyos-2.1.2/tos/interfaces/Send.nc"
inline static void * CC2420ActiveMessageP$SubSend$getPayload(message_t * msg, uint8_t len){
#line 125
  void *__nesc_result;
#line 125

#line 125
  __nesc_result = CC2420TinyosNetworkP$ActiveSend$getPayload(msg, len);
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 206 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static inline void *CC2420ActiveMessageP$Packet$getPayload(message_t *msg, uint8_t len)
#line 206
{
  return CC2420ActiveMessageP$SubSend$getPayload(msg, len);
}

# 126 "/opt/tinyos-2.1.2/tos/interfaces/Packet.nc"
inline static void * MoteToMoteC$Packet$getPayload(message_t * msg, uint8_t len){
#line 126
  void *__nesc_result;
#line 126

#line 126
  __nesc_result = CC2420ActiveMessageP$Packet$getPayload(msg, len);
#line 126

#line 126
  return __nesc_result;
#line 126
}
#line 126
# 36 "MoteToMoteC.nc"
static inline void MoteToMoteC$Timer$fired(void )
{
  if (!MoteToMoteC$RadioBusy) 
    {

      MoteToMoteMsg_t *msg = MoteToMoteC$Packet$getPayload(&MoteToMoteC$packet, sizeof(MoteToMoteMsg_t ));

#line 42
      __nesc_hton_uint16(msg->NodeId.nxdata, TOS_NODE_ID);
      __nesc_hton_uint8(msg->Data.nxdata, MoteToMoteC$dummyVar1);

      if (MoteToMoteC$AMSend$send(AM_BROADCAST_ADDR, &MoteToMoteC$packet, sizeof(MoteToMoteMsg_t )) == SUCCESS) 
        {
          MoteToMoteC$RadioBusy = TRUE;
        }
    }
}

# 204 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num)
{
}

# 83 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$fired(uint8_t arg_0x2b4801136da0){
#line 83
  switch (arg_0x2b4801136da0) {
#line 83
    case 0U:
#line 83
      PMICM$chargeMonitorTimer$fired();
#line 83
      break;
#line 83
    case 1U:
#line 83
      MoteToMoteC$Timer$fired();
#line 83
      break;
#line 83
    default:
#line 83
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$default$fired(arg_0x2b4801136da0);
#line 83
      break;
#line 83
    }
#line 83
}
#line 83
# 67 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$postTask(void ){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 93 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 94
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$start(t0, dt, TRUE);
}

# 129 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt){
#line 129
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$startOneShotAt(t0, dt);
#line 129
}
#line 129
# 117 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$stop(void )
#line 117
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 118
    {
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(FALSE);
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning = FALSE;
    }
#line 121
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 73 "/opt/tinyos-2.1.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$stop(void ){
#line 73
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$stop();
#line 73
}
#line 73
# 71 "/opt/tinyos-2.1.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$stop(void )
{
#line 72
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$stop();
}

# 78 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$stop(void ){
#line 78
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$stop();
#line 78
}
#line 78
# 100 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint16_t num;

  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$stop();

  for (num = 0; num < /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$fireTimers(now);
        }
      else {
#line 135
        /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(now, min_remaining);
        }
    }
}

# 57 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSCR(uint32_t val){
#line 57
  HplPXA27xOSTimerM$PXA27xOST$setOSCR(4, val);
#line 57
}
#line 57
#line 85
inline static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOMCR(uint32_t val){
#line 85
  HplPXA27xOSTimerM$PXA27xOST$setOMCR(4, val);
#line 85
}
#line 85
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTInit$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = HplPXA27xOSTimerM$Init$init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 63 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Init$init(void )
#line 63
{

  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTInit$init();
  /* atomic removed: atomic calls only */
  {
    /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning = FALSE;
    switch (2) {
        case 1: 
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT = 10;
        break;
        case 2: 
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT = 1;
        break;
        case 3: 
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT = 1;
        break;
        case 4: 
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT = 300;
        break;
        default: 
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT = 0;
        break;
      }
    /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOMCR(((1 << 7) | (1 << 6)) | (((2 & 0x8) << 5) | ((2 & 0x7) << 0)));
    /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSCR(0);
  }
  return SUCCESS;
}

# 48 "/opt/tinyos-2.1.2/tos/interfaces/LocalIeeeEui64.nc"
inline static ieee_eui64_t CC2420ControlP$LocalIeeeEui64$getId(void ){
#line 48
  struct ieee_eui64 __nesc_result;
#line 48

#line 48
  __nesc_result = LocalIeeeEui64C$LocalIeeeEui64$getId();
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 93 "/opt/tinyos-2.1.2/tos/system/ActiveMessageAddressC.nc"
static inline am_group_t ActiveMessageAddressC$ActiveMessageAddress$amGroup(void )
#line 93
{
  am_group_t myGroup;

  /* atomic removed: atomic calls only */
#line 95
  myGroup = ActiveMessageAddressC$group;
  return myGroup;
}

# 55 "/opt/tinyos-2.1.2/tos/interfaces/ActiveMessageAddress.nc"
inline static am_group_t CC2420ControlP$ActiveMessageAddress$amGroup(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = ActiveMessageAddressC$ActiveMessageAddress$amGroup();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
#line 50
inline static am_addr_t CC2420ControlP$ActiveMessageAddress$amAddress(void ){
#line 50
  unsigned short __nesc_result;
#line 50

#line 50
  __nesc_result = ActiveMessageAddressC$ActiveMessageAddress$amAddress();
#line 50

#line 50
  return __nesc_result;
#line 50
}
#line 50
# 46 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420ControlP$VREN$makeOutput(void ){
#line 46
  HalPXA27xGeneralIOM$GeneralIO$makeOutput(115);
#line 46
}
#line 46
inline static void CC2420ControlP$RSTN$makeOutput(void ){
#line 46
  HalPXA27xGeneralIOM$GeneralIO$makeOutput(22);
#line 46
}
#line 46
inline static void CC2420ControlP$CSN$makeOutput(void ){
#line 46
  HalPXA27xGeneralIOM$GeneralIO$makeOutput(39);
#line 46
}
#line 46
# 129 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static inline error_t CC2420ControlP$Init$init(void )
#line 129
{
  int i;
#line 130
  int t;

#line 131
  CC2420ControlP$CSN$makeOutput();
  CC2420ControlP$RSTN$makeOutput();
  CC2420ControlP$VREN$makeOutput();

  CC2420ControlP$m_short_addr = CC2420ControlP$ActiveMessageAddress$amAddress();
  CC2420ControlP$m_ext_addr = CC2420ControlP$LocalIeeeEui64$getId();
  CC2420ControlP$m_pan = CC2420ControlP$ActiveMessageAddress$amGroup();
  CC2420ControlP$m_tx_power = 31;
  CC2420ControlP$m_channel = 26;

  CC2420ControlP$m_ext_addr = CC2420ControlP$LocalIeeeEui64$getId();
  for (i = 0; i < 4; i++) {
      t = CC2420ControlP$m_ext_addr.data[i];
      CC2420ControlP$m_ext_addr.data[i] = CC2420ControlP$m_ext_addr.data[7 - i];
      CC2420ControlP$m_ext_addr.data[7 - i] = t;
    }





  CC2420ControlP$addressRecognition = TRUE;





  CC2420ControlP$hwAddressRecognition = FALSE;






  CC2420ControlP$autoAckEnabled = TRUE;






  CC2420ControlP$hwAutoAckDefault = FALSE;



  return SUCCESS;
}

# 81 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
static inline error_t StateImplP$Init$init(void )
#line 81
{
  int i;

#line 83
  for (i = 0; i < 4U; i++) {
      StateImplP$state[i] = StateImplP$S_IDLE;
    }
  return SUCCESS;
}

# 55 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$Init$init(void )
#line 55
{
  memset(/*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$resQ, /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$NO_ENTRY, sizeof /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$resQ);
  return SUCCESS;
}

# 57 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOSCR(uint32_t val){
#line 57
  HplPXA27xOSTimerM$PXA27xOST$setOSCR(6, val);
#line 57
}
#line 57
#line 85
inline static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOMCR(uint32_t val){
#line 85
  HplPXA27xOSTimerM$PXA27xOST$setOMCR(6, val);
#line 85
}
#line 85
# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTInit$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = HplPXA27xOSTimerM$Init$init();
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 63 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline error_t /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Init$init(void )
#line 63
{

  /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTInit$init();
  /* atomic removed: atomic calls only */
  {
    /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mfRunning = FALSE;
    switch (1) {
        case 1: 
          /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mMinDeltaT = 10;
        break;
        case 2: 
          /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mMinDeltaT = 1;
        break;
        case 3: 
          /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mMinDeltaT = 1;
        break;
        case 4: 
          /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mMinDeltaT = 300;
        break;
        default: 
          /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mMinDeltaT = 0;
        break;
      }
    /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOMCR(((1 << 7) | (1 << 6)) | (((1 & 0x8) << 5) | ((1 & 0x7) << 0)));
    /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOSCR(0);
  }
  return SUCCESS;
}

# 83 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline void HalPXA27xGeneralIOM$GeneralIO$makeInput(uint8_t pin)
#line 83
{
  /* atomic removed: atomic calls only */
#line 84
  HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPDRbit(pin, FALSE);
  return;
}

# 44 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
inline static void CC2420TransmitP$SFD$makeInput(void ){
#line 44
  HalPXA27xGeneralIOM$GeneralIO$makeInput(16);
#line 44
}
#line 44


inline static void CC2420TransmitP$CSN$makeOutput(void ){
#line 46
  HalPXA27xGeneralIOM$GeneralIO$makeOutput(39);
#line 46
}
#line 46
#line 44
inline static void CC2420TransmitP$CCA$makeInput(void ){
#line 44
  HalPXA27xGeneralIOM$GeneralIO$makeInput(116);
#line 44
}
#line 44
# 160 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static inline error_t CC2420TransmitP$Init$init(void )
#line 160
{
  CC2420TransmitP$CCA$makeInput();
  CC2420TransmitP$CSN$makeOutput();
  CC2420TransmitP$SFD$makeInput();
  return SUCCESS;
}

# 151 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static inline error_t CC2420ReceiveP$Init$init(void )
#line 151
{
  CC2420ReceiveP$m_p_rx_buf = &CC2420ReceiveP$m_rx_buf;
  return SUCCESS;
}

# 55 "/opt/tinyos-2.1.2/tos/system/RandomMlcgC.nc"
static inline error_t RandomMlcgC$Init$init(void )
#line 55
{
  /* atomic removed: atomic calls only */
#line 56
  RandomMlcgC$seed = (uint32_t )(TOS_NODE_ID + 1);

  return SUCCESS;
}

# 52 "/opt/tinyos-2.1.2/tos/interfaces/Random.nc"
inline static uint16_t UniqueSendP$Random$rand16(void ){
#line 52
  unsigned short __nesc_result;
#line 52

#line 52
  __nesc_result = RandomMlcgC$Random$rand16();
#line 52

#line 52
  return __nesc_result;
#line 52
}
#line 52
# 62 "/opt/tinyos-2.1.2/tos/chips/cc2420/unique/UniqueSendP.nc"
static inline error_t UniqueSendP$Init$init(void )
#line 62
{
  UniqueSendP$localSendId = UniqueSendP$Random$rand16();
  return SUCCESS;
}

# 71 "/opt/tinyos-2.1.2/tos/chips/cc2420/unique/UniqueReceiveP.nc"
static inline error_t UniqueReceiveP$Init$init(void )
#line 71
{
  int i;

#line 73
  for (i = 0; i < 4; i++) {
      UniqueReceiveP$receivedMessages[i].source = (am_addr_t )0xFFFF;
      UniqueReceiveP$receivedMessages[i].dsn = 0;
    }
  return SUCCESS;
}

# 55 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$Init$init(void )
#line 55
{
  memset(/*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$resQ, /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY, sizeof /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$resQ);
  return SUCCESS;
}

# 62 "/opt/tinyos-2.1.2/tos/interfaces/Init.nc"
inline static error_t RealMainP$SoftwareInit$init(void ){
#line 62
  unsigned char __nesc_result;
#line 62

#line 62
  __nesc_result = /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$Init$init();
#line 62
  __nesc_result = ecombine(__nesc_result, UniqueReceiveP$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, UniqueSendP$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, RandomMlcgC$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420ReceiveP$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420TransmitP$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*IM2CC2420SpiP.FcfsArbiterC.Queue*/FcfsResourceQueueC$1$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, StateImplP$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, CC2420ControlP$Init$init());
#line 62
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Init$init());
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 64 "/opt/tinyos-2.1.2/tos/lib/timer/Timer.nc"
inline static void MoteToMoteC$Timer$startPeriodic(uint32_t dt){
#line 64
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(1U, dt);
#line 64
}
#line 64
# 28 "MoteToMoteC.nc"
static inline void MoteToMoteC$Boot$booted(void )
#line 28
{
  MoteToMoteC$Timer$startPeriodic(500);
  MoteToMoteC$AMControl$start();
}

# 60 "/opt/tinyos-2.1.2/tos/interfaces/Boot.nc"
inline static void RealMainP$Boot$booted(void ){
#line 60
  MoteToMoteC$Boot$booted();
#line 60
}
#line 60
# 154 "/opt/tinyos-2.1.2/tos/chips/pxa27x/pxa27xhardware.h"
static __inline void __nesc_disable_interrupt()
#line 154
{

  uint32_t statusReg = 0;

   __asm volatile (
  "mrs %0,CPSR\n\t"
  "orr %0,%1,#0xc0\n\t"
  "msr CPSR_c,%1\n\t" : 
  "=r"(statusReg) : 
  "0"(statusReg));

  return;
}

#line 140
static __inline void __nesc_enable_interrupt()
#line 140
{

  uint32_t statusReg = 0;

   __asm volatile (
  "mrs %0,CPSR\n\t"
  "bic %0,%1,#0xc0\n\t"
  "msr CPSR_c, %1" : 
  "=r"(statusReg) : 
  "0"(statusReg));

  return;
}

# 53 "/opt/tinyos-2.1.2/tos/chips/pxa27x/McuSleepC.nc"
static inline void McuSleepC$McuSleep$sleep(void )
#line 53
{

   __asm volatile (
  "mcr p14,0,%0,c7,c0,0" :  : 

  "r"(1));

  __nesc_enable_interrupt();

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
  return;
}

# 76 "/opt/tinyos-2.1.2/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP$McuSleep$sleep(void ){
#line 76
  McuSleepC$McuSleep$sleep();
#line 76
}
#line 76
# 78 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP$popTask(void )
{
  if (SchedulerBasicP$m_head != SchedulerBasicP$NO_TASK) 
    {
      uint8_t id = SchedulerBasicP$m_head;

#line 83
      SchedulerBasicP$m_head = SchedulerBasicP$m_next[SchedulerBasicP$m_head];
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
        }
      SchedulerBasicP$m_next[id] = SchedulerBasicP$NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP$NO_TASK;
    }
}

#line 149
static inline void SchedulerBasicP$Scheduler$taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP$popTask()) == SchedulerBasicP$NO_TASK) 
            {
              SchedulerBasicP$McuSleep$sleep();
            }
        }
#line 161
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP$TaskBasic$runTask(nextTask);
    }
}

# 72 "/opt/tinyos-2.1.2/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$taskLoop(void ){
#line 72
  SchedulerBasicP$Scheduler$taskLoop();
#line 72
}
#line 72
# 68 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
__attribute((interrupt("IRQ")))   void hplarmv_irq(void )
#line 68
{

  uint32_t IRQPending;

  IRQPending = HplPXA27xInterruptM$getICHP();
  IRQPending >>= 16;

  while (IRQPending & (1 << 15)) {
      uint8_t PeripheralID = IRQPending & 0x3f;

#line 77
      HplPXA27xInterruptM$PXA27xIrq$fired(PeripheralID);
      IRQPending = HplPXA27xInterruptM$getICHP();
      IRQPending >>= 16;
    }

  return;
}

# 123 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
static uint32_t HplPXA27xSSPP$HplPXA27xSSP$getSSSR(uint8_t chnl)
#line 123
{
  switch (chnl) {
      case 1: return * (volatile uint32_t *)0x41000008;
#line 125
      break;
      case 2: return * (volatile uint32_t *)0x41700008;
#line 126
      break;
      case 3: return * (volatile uint32_t *)0x41900008;
#line 127
      break;
      default: return 0;
    }
}

#line 114
static void HplPXA27xSSPP$HplPXA27xSSP$setSSSR(uint8_t chnl, uint32_t val)
#line 114
{
  switch (chnl) {
      case 1: * (volatile uint32_t *)0x41000008 = val;
#line 116
      break;
      case 2: * (volatile uint32_t *)0x41700008 = val;
#line 117
      break;
      case 3: * (volatile uint32_t *)0x41900008 = val;
#line 118
      break;
      default: break;
    }
  return;
}

#line 159
static uint32_t HplPXA27xSSPP$HplPXA27xSSP$getSSDR(uint8_t chnl)
#line 159
{
  switch (chnl) {
      case 1: return * (volatile uint32_t *)0x41000010;
#line 161
      break;
      case 2: return * (volatile uint32_t *)0x41700010;
#line 162
      break;
      case 3: return * (volatile uint32_t *)0x41900010;
#line 163
      break;
      default: return 0;
    }
}

#line 150
static void HplPXA27xSSPP$HplPXA27xSSP$setSSDR(uint8_t chnl, uint32_t val)
#line 150
{
  switch (chnl) {
      case 1: * (volatile uint32_t *)0x41000010 = val;
#line 152
      break;
      case 2: * (volatile uint32_t *)0x41700010 = val;
#line 153
      break;
      case 3: * (volatile uint32_t *)0x41900010 = val;
#line 154
      break;
      default: break;
    }
  return;
}

#line 96
static void HplPXA27xSSPP$HplPXA27xSSP$setSSCR1(uint8_t chnl, uint32_t val)
#line 96
{
  switch (chnl) {
      case 1: * (volatile uint32_t *)0x41000004 = val;
#line 98
      break;
      case 2: * (volatile uint32_t *)0x41700004 = val;
#line 99
      break;
      case 3: * (volatile uint32_t *)0x41900004 = val;
#line 100
      break;
      default: break;
    }
  return;
}

# 733 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP$flush(void )
#line 733
{








  CC2420ReceiveP$reset_state();

  CC2420ReceiveP$CSN$set();
  CC2420ReceiveP$CSN$clr();
  CC2420ReceiveP$SFLUSHRX$strobe();
  CC2420ReceiveP$SFLUSHRX$strobe();
  CC2420ReceiveP$CSN$set();
  CC2420ReceiveP$SpiResource$release();
  CC2420ReceiveP$waitForNextPacket();
}

#line 813
static void CC2420ReceiveP$reset_state(void )
#line 813
{
  CC2420ReceiveP$m_bytes_left = CC2420ReceiveP$RXFIFO_SIZE;
  /* atomic removed: atomic calls only */
#line 815
  CC2420ReceiveP$receivingPacket = FALSE;
  CC2420ReceiveP$m_timestamp_head = 0;
  CC2420ReceiveP$m_timestamp_size = 0;
  CC2420ReceiveP$m_missed_packets = 0;
}

# 54 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static void HalPXA27xGeneralIOM$GeneralIO$set(uint8_t pin)
#line 54
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPSRbit(pin);
#line 56
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

static void HalPXA27xGeneralIOM$GeneralIO$clr(uint8_t pin)
#line 60
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPCRbit(pin);
#line 61
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 318 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP$Strobe$strobe(uint8_t addr)
#line 318
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 319
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 321
            0;

            {
#line 321
              __nesc_atomic_end(__nesc_atomic); 
#line 321
              return __nesc_temp;
            }
          }
        }
    }
#line 325
    __nesc_atomic_end(__nesc_atomic); }
#line 325
  return CC2420SpiP$SpiByte$write(addr);
}

# 133 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
static bool StateImplP$State$isState(uint8_t id, uint8_t myState)
#line 133
{
  bool isState;

#line 135
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 135
    isState = StateImplP$state[id] == myState;
#line 135
    __nesc_atomic_end(__nesc_atomic); }
  return isState;
}

# 95 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSpiPioM.nc"
static uint8_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiByte$write(uint8_t tx)
#line 95
{
  volatile uint8_t val;

  while (/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSSR() & (1 << 3)) {
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSDR();
    }

  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSDR(tx);

  while (/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSSR() & (1 << 4)) ;

  val = /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSDR();

  return val;
}

# 149 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP$Resource$release(uint8_t id)
#line 149
{
  uint8_t i;

#line 151
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    {
      if (CC2420SpiP$m_holder != id) {
          {
            unsigned char __nesc_temp = 
#line 153
            FAIL;

            {
#line 153
              __nesc_atomic_end(__nesc_atomic); 
#line 153
              return __nesc_temp;
            }
          }
        }
#line 156
      CC2420SpiP$m_holder = CC2420SpiP$NO_HOLDER;
      if (!CC2420SpiP$m_requests) {
          CC2420SpiP$WorkingState$toIdle();
          CC2420SpiP$attemptRelease();
        }
      else {
          for (i = CC2420SpiP$m_holder + 1; ; i++) {
              i %= CC2420SpiP$RESOURCE_COUNT;

              if (CC2420SpiP$m_requests & (1 << i)) {
                  CC2420SpiP$m_holder = i;
                  CC2420SpiP$m_requests &= ~(1 << i);
                  CC2420SpiP$grant$postTask();
                  {
                    unsigned char __nesc_temp = 
#line 169
                    SUCCESS;

                    {
#line 169
                      __nesc_atomic_end(__nesc_atomic); 
#line 169
                      return __nesc_temp;
                    }
                  }
                }
            }
        }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
#line 175
  return SUCCESS;
}

#line 339
static error_t CC2420SpiP$attemptRelease(void )
#line 339
{


  if ((
#line 340
  CC2420SpiP$m_requests > 0
   || CC2420SpiP$m_holder != CC2420SpiP$NO_HOLDER)
   || !CC2420SpiP$WorkingState$isIdle()) {
      return FAIL;
    }
  /* atomic removed: atomic calls only */
  CC2420SpiP$release = TRUE;
  CC2420SpiP$ChipSpiResource$releasing();
  /* atomic removed: atomic calls only */
#line 348
  {
    if (CC2420SpiP$release) {
        CC2420SpiP$SpiResource$release();
        {
          unsigned char __nesc_temp = 
#line 351
          SUCCESS;

#line 351
          return __nesc_temp;
        }
      }
  }
  return EBUSY;
}

# 170 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 172
    {
#line 172
      {
        unsigned char __nesc_temp = 
#line 172
        SchedulerBasicP$pushTask(id) ? SUCCESS : EBUSY;

        {
#line 172
          __nesc_atomic_end(__nesc_atomic); 
#line 172
          return __nesc_temp;
        }
      }
    }
#line 175
    __nesc_atomic_end(__nesc_atomic); }
}

# 769 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP$waitForNextPacket(void )
#line 769
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 770
    {
      if (CC2420ReceiveP$m_state == CC2420ReceiveP$S_STOPPED) {
          CC2420ReceiveP$SpiResource$release();
          {
#line 773
            __nesc_atomic_end(__nesc_atomic); 
#line 773
            return;
          }
        }
      CC2420ReceiveP$receivingPacket = FALSE;
#line 788
      if ((CC2420ReceiveP$m_missed_packets && CC2420ReceiveP$FIFO$get()) || !CC2420ReceiveP$FIFOP$get()) {

          if (CC2420ReceiveP$m_missed_packets) {
              CC2420ReceiveP$m_missed_packets--;
            }





          CC2420ReceiveP$beginReceive();
        }
      else 
        {

          CC2420ReceiveP$m_state = CC2420ReceiveP$S_STARTED;
          CC2420ReceiveP$m_missed_packets = 0;
          CC2420ReceiveP$SpiResource$release();
        }
    }
#line 807
    __nesc_atomic_end(__nesc_atomic); }
}

# 80 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static bool HplPXA27xGPIOM$HplPXA27xGPIOPin$getGPLRbit(uint8_t pin)
{
  return (*(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00000 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00100) & (1 << (pin & 0x1f))) != 0;
}

# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static bool CC2420ReceiveP$FIFOP$get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = HalPXA27xGeneralIOM$GeneralIO$get(0);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 716 "/opt/tinyos-2.1.2/tos/chips/cc2420/receive/CC2420ReceiveP.nc"
static void CC2420ReceiveP$beginReceive(void )
#line 716
{
  CC2420ReceiveP$m_state = CC2420ReceiveP$S_RX_LENGTH;
  /* atomic removed: atomic calls only */
#line 718
  CC2420ReceiveP$receivingPacket = TRUE;
  if (CC2420ReceiveP$SpiResource$isOwner()) {
      CC2420ReceiveP$receive();
    }
  else {
#line 722
    if (CC2420ReceiveP$SpiResource$immediateRequest() == SUCCESS) {
        CC2420ReceiveP$receive();
      }
    else {
        CC2420ReceiveP$SpiResource$request();
      }
    }
}

#line 759
static void CC2420ReceiveP$receive(void )
#line 759
{
  CC2420ReceiveP$CSN$clr();
  CC2420ReceiveP$RXFIFO$beginRead((uint8_t *)CC2420ReceiveP$CC2420PacketBody$getHeader(CC2420ReceiveP$m_p_rx_buf), 1);
}

# 189 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP$Fifo$beginRead(uint8_t addr, uint8_t *data, 
uint8_t len)
#line 190
{

  cc2420_status_t status = 0;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 194
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 196
            status;

            {
#line 196
              __nesc_atomic_end(__nesc_atomic); 
#line 196
              return __nesc_temp;
            }
          }
        }
    }
#line 200
    __nesc_atomic_end(__nesc_atomic); }
#line 200
  CC2420SpiP$m_addr = addr | 0x40;

  status = CC2420SpiP$SpiByte$write(CC2420SpiP$m_addr);
  CC2420SpiP$Fifo$continueRead(addr, data, len);

  return status;
}

# 111 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HalPXA27xSpiPioM.nc"
static error_t /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SpiPacket$send(uint8_t instance, uint8_t *txBuf, uint8_t *rxBuf, uint16_t len)
#line 111
{
  uint32_t i;


  while (/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSSR() & (1 << 3)) {
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSDR();
    }


  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txCurrentBuf = txBuf;
  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxCurrentBuf = rxBuf;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 122
    /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenCurrent = /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$lenRemain = len;
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$instanceCurrent = instance;

  if (rxBuf == (void *)0) {
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxPtr = (uint8_t *)&/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxBitBucket;
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxInc = 0;
    }
  else {
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxPtr = rxBuf;
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxInc = 1;
    }

  if (txBuf == (void *)0) {
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txPtr = (uint8_t *)&/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txBitBucket;
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txInc = 0;
    }
  else {
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txPtr = txBuf;
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txInc = 1;
    }

  if (txBuf == (void *)0 && 0 == TRUE) {
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR0(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR0);
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR1(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR1 | (1 << 23));
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR0(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR0 | (1 << 7));
      while (len > 0) {
          while (!(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSSR() & (1 << 3))) ;
          */*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxPtr = /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$getSSDR();
          /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxPtr += /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$rxInc;
          len--;
        }
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR0(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR0);
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR1(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR1);
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR0(/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR0 | (1 << 7));
    }
  else {
      uint8_t burst = len < 16 ? len : 16;

#line 159
      for (i = 0; i < burst; i++) {
          /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSDR(*/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txPtr);
          /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txPtr += /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$txInc;
        }
      /*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$SSP$setSSCR1((/*IM2CC2420SpiP.HalPXA27xSpiM.HalPXA27xSpiPioM*/HalPXA27xSpiPioM$0$FLAGS_SSCR1 | (1 << 19)) | (1 << 0));
    }

  return SUCCESS;
}

# 77 "/opt/tinyos-2.1.2/tos/chips/pxa27x/ssp/HplPXA27xSSPP.nc"
static void HplPXA27xSSPP$HplPXA27xSSP$setSSCR0(uint8_t chnl, uint32_t val)
#line 77
{
  switch (chnl) {
      case 1: * (volatile uint32_t *)0x41000000 = val;
#line 79
      break;
      case 2: * (volatile uint32_t *)0x41700000 = val;
#line 80
      break;
      case 3: * (volatile uint32_t *)0x41900000 = val;
#line 81
      break;
      default: break;
    }
  return;
}

# 126 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP$Resource$immediateRequest(uint8_t id)
#line 126
{
  error_t error;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      if (CC2420SpiP$WorkingState$requestState(CC2420SpiP$S_BUSY) != SUCCESS) {
          {
            unsigned char __nesc_temp = 
#line 131
            EBUSY;

            {
#line 131
              __nesc_atomic_end(__nesc_atomic); 
#line 131
              return __nesc_temp;
            }
          }
        }
      if (CC2420SpiP$SpiResource$isOwner()) {
          CC2420SpiP$m_holder = id;
          error = SUCCESS;
        }
      else {
#line 139
        if ((error = CC2420SpiP$SpiResource$immediateRequest()) == SUCCESS) {
            CC2420SpiP$m_holder = id;
          }
        else {
            CC2420SpiP$WorkingState$toIdle();
          }
        }
    }
#line 146
    __nesc_atomic_end(__nesc_atomic); }
#line 146
  return error;
}

# 96 "/opt/tinyos-2.1.2/tos/system/StateImplP.nc"
static error_t StateImplP$State$requestState(uint8_t id, uint8_t reqState)
#line 96
{
  error_t returnVal = FAIL;

#line 98
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 98
    {
      if (reqState == StateImplP$S_IDLE || StateImplP$state[id] == StateImplP$S_IDLE) {
          StateImplP$state[id] = reqState;
          returnVal = SUCCESS;
        }
    }
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return returnVal;
}

# 148 "/opt/tinyos-2.1.2/tos/system/SimpleArbiterP.nc"
static bool /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$Resource$isOwner(uint8_t id)
#line 148
{
  /* atomic removed: atomic calls only */
#line 149
  {
    if (/*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$resId == id && /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$state == /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$RES_BUSY) {
        unsigned char __nesc_temp = 
#line 150
        TRUE;

#line 150
        return __nesc_temp;
      }
    else 
#line 151
      {
        unsigned char __nesc_temp = 
#line 151
        FALSE;

#line 151
        return __nesc_temp;
      }
  }
}

# 107 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static error_t CC2420SpiP$Resource$request(uint8_t id)
#line 107
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 109
    {
      if (CC2420SpiP$WorkingState$requestState(CC2420SpiP$S_BUSY) == SUCCESS) {
          CC2420SpiP$m_holder = id;
          if (CC2420SpiP$SpiResource$isOwner()) {
              CC2420SpiP$grant$postTask();
            }
          else {
              CC2420SpiP$SpiResource$request();
            }
        }
      else {
          CC2420SpiP$m_requests |= 1 << id;
        }
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 302 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static uint16_t CC2420ControlP$CC2420Config$getShortAddr(void )
#line 302
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 303
    {
      unsigned short __nesc_temp = 
#line 303
      CC2420ControlP$m_short_addr;

      {
#line 303
        __nesc_atomic_end(__nesc_atomic); 
#line 303
        return __nesc_temp;
      }
    }
#line 305
    __nesc_atomic_end(__nesc_atomic); }
}

# 171 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static void CC2420PacketP$PacketTimeStamp32khz$clear(message_t *msg)
{
  __nesc_hton_int8(CC2420PacketP$CC2420PacketBody$getMetadata(msg)->timesync.nxdata, FALSE);
  __nesc_hton_uint32(CC2420PacketP$CC2420PacketBody$getMetadata(msg)->timestamp.nxdata, CC2420_INVALID_TIMESTAMP);
}

# 163 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOIERbit(uint8_t chnl_id, bool flag)
{
  if (flag == TRUE) {
      * (volatile uint32_t *)0x40A0001C |= 1 << chnl_id;
    }
  else {
      * (volatile uint32_t *)0x40A0001C &= ~(1 << chnl_id);
    }
  return;
}

# 850 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP$signalDone(error_t err)
#line 850
{
  /* atomic removed: atomic calls only */
#line 851
  CC2420TransmitP$m_state = CC2420TransmitP$S_STARTED;
  CC2420TransmitP$abortSpiRelease = FALSE;
  CC2420TransmitP$ChipSpiResource$attemptRelease();
  CC2420TransmitP$Send$sendDone(CC2420TransmitP$m_msg, err);
}

#line 743
static void CC2420TransmitP$attemptSend(void )
#line 743
{
  uint8_t status;
  bool congestion = TRUE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 747
    {
      if (CC2420TransmitP$m_state == CC2420TransmitP$S_CANCEL) {
          CC2420TransmitP$SFLUSHTX$strobe();
          CC2420TransmitP$releaseSpiResource();
          CC2420TransmitP$CSN$set();
          CC2420TransmitP$m_state = CC2420TransmitP$S_STARTED;
          CC2420TransmitP$Send$sendDone(CC2420TransmitP$m_msg, ECANCEL);
          {
#line 754
            __nesc_atomic_end(__nesc_atomic); 
#line 754
            return;
          }
        }





      CC2420TransmitP$CSN$clr();
      status = CC2420TransmitP$m_cca ? CC2420TransmitP$STXONCCA$strobe() : CC2420TransmitP$STXON$strobe();
      if (!(status & CC2420_STATUS_TX_ACTIVE)) {
          status = CC2420TransmitP$SNOP$strobe();
          if (status & CC2420_STATUS_TX_ACTIVE) {
              congestion = FALSE;
            }
        }

      CC2420TransmitP$m_state = congestion ? CC2420TransmitP$S_SAMPLE_CCA : CC2420TransmitP$S_SFD;
      CC2420TransmitP$CSN$set();
    }
#line 773
    __nesc_atomic_end(__nesc_atomic); }

  if (congestion) {
      CC2420TransmitP$totalCcaChecks = 0;
      CC2420TransmitP$releaseSpiResource();
      CC2420TransmitP$congestionBackoff();
    }
  else 
#line 779
    {
      CC2420TransmitP$BackoffTimer$start(CC2420TransmitP$CC2420_ABORT_PERIOD);
    }
}





static void CC2420TransmitP$congestionBackoff(void )
#line 788
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 789
    {
      CC2420TransmitP$RadioBackoff$requestCongestionBackoff(CC2420TransmitP$m_msg);
      CC2420TransmitP$BackoffTimer$start(CC2420TransmitP$myCongestionBackoff);
    }
#line 792
    __nesc_atomic_end(__nesc_atomic); }
}

# 69 "/opt/tinyos-2.1.2/tos/system/RandomMlcgC.nc"
static uint32_t RandomMlcgC$Random$rand32(void )
#line 69
{
  uint32_t mlcg;
#line 70
  uint32_t p;
#line 70
  uint32_t q;
  uint64_t tmpseed;

  /* atomic removed: atomic calls only */
#line 73
  {
    tmpseed = (uint64_t )33614U * (uint64_t )RandomMlcgC$seed;
    q = tmpseed;
    q = q >> 1;
    p = tmpseed >> 32;
    mlcg = p + q;
    if (mlcg & 0x80000000) {
        mlcg = mlcg & 0x7FFFFFFF;
        mlcg++;
      }
    RandomMlcgC$seed = mlcg;
  }
  return mlcg;
}

# 93 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static void /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$Alarm$start(uint32_t dt)
#line 93
{
  uint32_t t0;
#line 94
  uint32_t t1;
#line 94
  uint32_t tf;

  bool bPending;

#line 97
  if (dt < /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mMinDeltaT) {
#line 97
    dt = /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mMinDeltaT;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {

      t0 = /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$getOSCR();
      tf = t0 + dt;
      /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOIERbit(TRUE);
      /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOSMR(tf);

      /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$mfRunning = TRUE;
      t1 = /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$getOSCR();
      bPending = /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$getOSSRbit();
      if (dt <= t1 - t0 && !bPending) {
          /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$setOIERbit(FALSE);
          /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$lateAlarm$postTask();
        }
    }
#line 113
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 97 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSCR(uint8_t chnl_id)
{
  uint8_t remap_id;
  uint32_t val;

  remap_id = chnl_id < 4 ? 0 : chnl_id;
  val = *(remap_id == 0 ? & * (volatile uint32_t *)0x40A00010 : & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00040 + (uint32_t )((remap_id - 4) << 2)));

  return val;
}

static void HplPXA27xOSTimerM$PXA27xOST$setOSMR(uint8_t chnl_id, uint32_t val)
{
  *(chnl_id < 4 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00000 + (uint32_t )(chnl_id << 2)) : & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00080 + (uint32_t )((chnl_id - 4) << 2))) = val;
  return;
}

#line 138
static bool HplPXA27xOSTimerM$PXA27xOST$getOSSRbit(uint8_t chnl_id)
{
  bool bFlag = FALSE;

  if ((* (volatile uint32_t *)0x40A00014 & (1 << chnl_id)) != 0) {
      bFlag = TRUE;
    }

  return bFlag;
}

# 150 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(uint8_t arg_0x2b4801368020){
#line 150
  HalPXA27xGeneralIOM$HplPXA27xGPIOPin$interruptGPIOPin(arg_0x2b4801368020);
#line 150
  switch (arg_0x2b4801368020) {
#line 150
    case 1:
#line 150
      PMICM$PMICGPIO$interruptGPIOPin();
#line 150
      break;
#line 150
    case 34:
#line 150
      IM2CC2420InitSpiP$SCLK$interruptGPIOPin();
#line 150
      break;
#line 150
    case 35:
#line 150
      IM2CC2420InitSpiP$TXD$interruptGPIOPin();
#line 150
      break;
#line 150
    case 41:
#line 150
      IM2CC2420InitSpiP$RXD$interruptGPIOPin();
#line 150
      break;
#line 150
  }
#line 150
}
#line 150
# 150 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static bool HplPXA27xGPIOM$HplPXA27xGPIOPin$clearGEDRbit(uint8_t pin)
{
  bool flag;

#line 153
  flag = (*(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00048 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00148) & (1 << (pin & 0x1f))) != 0;
  *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00048 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00148) = 1 << (pin & 0x1f);
  return flag;
}

# 129 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$disable(uint8_t pin)
#line 129
{
  /* atomic removed: atomic calls only */
#line 130
  {
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGRERbit(pin, FALSE);
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGFERbit(pin, FALSE);
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$clearGEDRbit(pin);
  }
  return SUCCESS;
}

# 113 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGRERbit(uint8_t pin, bool flag)
{
  if (flag) {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00030 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00130) |= 1 << (pin & 0x1f);
    }
  else {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00030 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00130) &= ~(1 << (pin & 0x1f));
    }
  return;
}






static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGFERbit(uint8_t pin, bool flag)
{
  if (flag) {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E0003C + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E0013C) |= 1 << (pin & 0x1f);
    }
  else {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E0003C + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E0013C) &= ~(1 << (pin & 0x1f));
    }
  return;
}

# 305 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP$Reg$write(uint8_t addr, uint16_t data)
#line 305
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 306
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 308
            0;

            {
#line 308
              __nesc_atomic_end(__nesc_atomic); 
#line 308
              return __nesc_temp;
            }
          }
        }
    }
#line 312
    __nesc_atomic_end(__nesc_atomic); }
#line 311
  CC2420SpiP$SpiByte$write(addr);
  CC2420SpiP$SpiByte$write(data >> 8);
  return CC2420SpiP$SpiByte$write(data & 0xff);
}

# 515 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP$writeId(void )
#line 515
{
  nxle_uint16_t id[6];

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 518
    {

      memcpy((uint8_t *)id, CC2420ControlP$m_ext_addr.data, 8);
      __nesc_hton_leuint16(id[4].nxdata, CC2420ControlP$m_pan);
      __nesc_hton_leuint16(id[5].nxdata, CC2420ControlP$m_short_addr);
    }
#line 523
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP$IEEEADR$write(0, (uint8_t *)&id, 12);
}

# 260 "/opt/tinyos-2.1.2/tos/chips/cc2420/spi/CC2420SpiP.nc"
static cc2420_status_t CC2420SpiP$Ram$write(uint16_t addr, uint8_t offset, 
uint8_t *data, 
uint8_t len)
#line 262
{

  cc2420_status_t status = 0;
  uint8_t tmpLen = len;
  uint8_t * tmpData = (uint8_t * )data;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 268
    {
      if (CC2420SpiP$WorkingState$isIdle()) {
          {
            unsigned char __nesc_temp = 
#line 270
            status;

            {
#line 270
              __nesc_atomic_end(__nesc_atomic); 
#line 270
              return __nesc_temp;
            }
          }
        }
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
#line 274
  addr += offset;

  status = CC2420SpiP$SpiByte$write(addr | 0x80);
  CC2420SpiP$SpiByte$write((addr >> 1) & 0xc0);
  for (; len; len--) {
      CC2420SpiP$SpiByte$write(tmpData[tmpLen - len]);
    }

  return status;
}

# 113 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableFallingEdge(uint8_t pin)
#line 113
{
  /* atomic removed: atomic calls only */
#line 114
  {
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGRERbit(pin, FALSE);
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGFERbit(pin, TRUE);
  }
  return SUCCESS;
}

# 43 "/opt/tinyos-2.1.2/tos/interfaces/GeneralIO.nc"
static bool CC2420TransmitP$SFD$get(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = HalPXA27xGeneralIOM$GeneralIO$get(16);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 105 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableRisingEdge(uint8_t pin)
#line 105
{
  /* atomic removed: atomic calls only */
#line 106
  {
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGRERbit(pin, TRUE);
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGFERbit(pin, FALSE);
  }
  return SUCCESS;
}

# 104 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static error_t PMICM$readPMIC(uint8_t address, uint8_t *value, uint8_t numBytes)
#line 104
{

  if (numBytes > 0) {
      PMICM$PI2C$setIDBR(0x49 << 1);
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 0));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
      while (PMICM$PI2C$getICR() & (1 << 3)) ;


      PMICM$PI2C$setIDBR(address);
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 0));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 1));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
      while (PMICM$PI2C$getICR() & (1 << 3)) ;
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 1));


      PMICM$PI2C$setIDBR((0x49 << 1) | 1);
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 0));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
      while (PMICM$PI2C$getICR() & (1 << 3)) ;
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 0));


      while (numBytes > 1) {
          PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
          while (PMICM$PI2C$getICR() & (1 << 3)) ;
          *value = PMICM$PI2C$getIDBR();
          value++;
          numBytes--;
        }

      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 1));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 2));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
      while (PMICM$PI2C$getICR() & (1 << 3)) ;
      *value = PMICM$PI2C$getIDBR();
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 1));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 2));

      return SUCCESS;
    }
  else {
      return FAIL;
    }
}

# 90 "/opt/tinyos-2.1.2/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setIDBR(uint32_t val)
#line 90
{
  switch (1) {
      case 0: * (volatile uint32_t *)0x40301688 = val;
#line 92
      break;
      case 1: * (volatile uint32_t *)0x40F00188 = val;
#line 93
      break;
      default: break;
    }
  return;
}









static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setICR(uint32_t val)
#line 107
{
  switch (1) {
      case 0: * (volatile uint32_t *)0x40301690 = val;
#line 109
      break;
      case 1: * (volatile uint32_t *)0x40F00190 = val;
#line 110
      break;
      default: break;
    }
  return;
}

static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getICR(void )
#line 116
{
  switch (1) {
      case 0: return * (volatile uint32_t *)0x40301690;
      case 1: return * (volatile uint32_t *)0x40F00190;
      default: return 0;
    }
}

#line 99
static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getIDBR(void )
#line 99
{
  switch (1) {
      case 0: return * (volatile uint32_t *)0x40301688;
      case 1: return * (volatile uint32_t *)0x40F00188;
      default: return 0;
    }
}

# 233 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static void HplPXA27xOSTimerM$PXA27xOST$default$fired(uint8_t chnl_id)
{
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(chnl_id, FALSE);
  HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(chnl_id);
  return;
}

# 139 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$fired(uint8_t arg_0x2b480120a358){
#line 139
  switch (arg_0x2b480120a358) {
#line 139
    case 3:
#line 139
      PlatformP$OST0M3$fired();
#line 139
      break;
#line 139
    case 4:
#line 139
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$fired();
#line 139
      break;
#line 139
    case 5:
#line 139
      /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$fired();
#line 139
      break;
#line 139
    case 6:
#line 139
      /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$OSTChnl$fired();
#line 139
      break;
#line 139
    case 7:
#line 139
      /*Counter32khzC.PhysCounter32khz32*/HalPXA27xCounterM$1$OSTChnl$fired();
#line 139
      break;
#line 139
    case 8:
#line 139
      /*Counter32khz32C.PhysCounter32khz32*/HalPXA27xCounterM$2$OSTChnl$fired();
#line 139
      break;
#line 139
    default:
#line 139
      HplPXA27xOSTimerM$PXA27xOST$default$fired(arg_0x2b480120a358);
#line 139
      break;
#line 139
    }
#line 139
}
#line 139
# 149 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static bool HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(uint8_t chnl_id)
{
  bool bFlag = FALSE;

  if ((* (volatile uint32_t *)0x40A00014 & (1 << chnl_id)) != 0) {
      bFlag = TRUE;
    }


  * (volatile uint32_t *)0x40A00014 = 1 << chnl_id;

  return bFlag;
}

# 498 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP$BackoffTimer$fired(void )
#line 498
{
  /* atomic removed: atomic calls only */
#line 499
  {
    switch (CC2420TransmitP$m_state) {

        case CC2420TransmitP$S_SAMPLE_CCA: 


          if (CC2420TransmitP$CCA$get()) {
              CC2420TransmitP$m_state = CC2420TransmitP$S_BEGIN_TRANSMIT;
              CC2420TransmitP$BackoffTimer$start(CC2420_TIME_ACK_TURNAROUND);
            }
          else {
              CC2420TransmitP$congestionBackoff();
            }
        break;

        case CC2420TransmitP$S_BEGIN_TRANSMIT: 
          case CC2420TransmitP$S_CANCEL: 
            if (CC2420TransmitP$acquireSpiResource() == SUCCESS) {
                CC2420TransmitP$attemptSend();
              }
        break;

        case CC2420TransmitP$S_ACK_WAIT: 
          CC2420TransmitP$signalDone(SUCCESS);
        break;

        case CC2420TransmitP$S_SFD: 


          CC2420TransmitP$SFLUSHTX$strobe();
        CC2420TransmitP$CaptureSFD$captureRisingEdge();
        CC2420TransmitP$releaseSpiResource();
        CC2420TransmitP$signalDone(ERETRY);
        break;

        default: 
          break;
      }
  }
}

#line 795
static error_t CC2420TransmitP$acquireSpiResource(void )
#line 795
{
  error_t error = CC2420TransmitP$SpiResource$immediateRequest();

#line 797
  if (error != SUCCESS) {
      CC2420TransmitP$SpiResource$request();
    }
  return error;
}

# 431 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP$StartupTimer$fired(void )
#line 431
{
  if (CC2420ControlP$m_state == CC2420ControlP$S_VREG_STARTING) {
      CC2420ControlP$m_state = CC2420ControlP$S_VREG_STARTED;
      CC2420ControlP$RSTN$clr();
      CC2420ControlP$RSTN$set();
      CC2420ControlP$CC2420Power$startVRegDone();
    }
}

# 85 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
__attribute((interrupt("FIQ")))   void hplarmv_fiq(void )
#line 85
{
}

# 63 "/opt/tinyos-2.1.2/tos/system/RealMainP.nc"
  int main(void )
#line 63
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 71
      ;

      RealMainP$Scheduler$init();





      RealMainP$PlatformInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;





      RealMainP$SoftwareInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;
    }
#line 88
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP$Boot$booted();


  RealMainP$Scheduler$taskLoop();




  return -1;
}

# 61 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static error_t HplPXA27xOSTimerM$Init$init(void )
{
  bool initflag;

  /* atomic removed: atomic calls only */
#line 64
  {
    initflag = HplPXA27xOSTimerM$gfInitialized;
    HplPXA27xOSTimerM$gfInitialized = TRUE;
  }

  if (!initflag) {
      * (volatile uint32_t *)0x40A0001C = 0x0UL;
      * (volatile uint32_t *)0x40A00014 = 0xFFFFFFFF;
      HplPXA27xOSTimerM$OST0Irq$allocate();
      HplPXA27xOSTimerM$OST1Irq$allocate();
      HplPXA27xOSTimerM$OST2Irq$allocate();
      HplPXA27xOSTimerM$OST3Irq$allocate();
      HplPXA27xOSTimerM$OST4_11Irq$allocate();
      HplPXA27xOSTimerM$OST0Irq$enable();
      HplPXA27xOSTimerM$OST1Irq$enable();
      HplPXA27xOSTimerM$OST2Irq$enable();
      HplPXA27xOSTimerM$OST3Irq$enable();
      HplPXA27xOSTimerM$OST4_11Irq$enable();
    }

  return SUCCESS;
}

# 94 "/opt/tinyos-2.1.2/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
static error_t HplPXA27xInterruptM$allocate(uint8_t id, bool level, uint8_t priority)
{
  uint32_t tmp;
  error_t error = FAIL;

  /* atomic removed: atomic calls only */
#line 99
  {
    uint8_t i;

#line 101
    if (HplPXA27xInterruptM$usedPriorities == 0) {
        uint8_t PriorityTable[40];
#line 102
        uint8_t DuplicateTable[40];

#line 103
        for (i = 0; i < 40; i++) {
            DuplicateTable[i] = PriorityTable[i] = 0xFF;
          }

        for (i = 0; i < 40; i++) 
          if (TOSH_IRP_TABLE[i] != 0xff) {
              if (PriorityTable[TOSH_IRP_TABLE[i]] != 0xFF) {


                DuplicateTable[i] = PriorityTable[TOSH_IRP_TABLE[i]];
                }
              else {
#line 114
                PriorityTable[TOSH_IRP_TABLE[i]] = i;
                }
            }

        for (i = 0; i < 40; i++) {
            if (PriorityTable[i] != 0xff) {
                PriorityTable[HplPXA27xInterruptM$usedPriorities] = PriorityTable[i];
                if (i != HplPXA27xInterruptM$usedPriorities) {
                  PriorityTable[i] = 0xFF;
                  }
#line 123
                HplPXA27xInterruptM$usedPriorities++;
              }
          }

        for (i = 0; i < 40; i++) 
          if (DuplicateTable[i] != 0xFF) {
              uint8_t j;
#line 129
              uint8_t ExtraTable[40];

#line 130
              for (j = 0; DuplicateTable[i] != PriorityTable[j]; j++) ;
              memcpy(ExtraTable + j + 1, PriorityTable + j, HplPXA27xInterruptM$usedPriorities - j);
              memcpy(PriorityTable + j + 1, ExtraTable + j + 1, 
              HplPXA27xInterruptM$usedPriorities - j);
              PriorityTable[j] = i;
              HplPXA27xInterruptM$usedPriorities++;
            }

        for (i = 0; i < HplPXA27xInterruptM$usedPriorities; i++) {
            * (volatile uint32_t *)((uint32_t )0x40D0001C + (uint32_t )((i & 0x1F) << 2)) = (1 << 31) | PriorityTable[i];
            tmp = * (volatile uint32_t *)((uint32_t )0x40D0001C + (uint32_t )((i & 0x1F) << 2));
          }
      }

    if (id < 34) {
        if (priority == 0xff) {
            priority = HplPXA27xInterruptM$usedPriorities;
            HplPXA27xInterruptM$usedPriorities++;
            * (volatile uint32_t *)((uint32_t )0x40D0001C + (uint32_t )((priority & 0x1F) << 2)) = (1 << 31) | id;
            tmp = * (volatile uint32_t *)((uint32_t )0x40D0001C + (uint32_t )((priority & 0x1F) << 2));
          }
        if (level) {
            *(id & 0x20 ? & * (volatile uint32_t *)0x40D000A4 : & * (volatile uint32_t *)0x40D00008) |= 1 << (id & 0x1f);
            tmp = *(id & 0x20 ? & * (volatile uint32_t *)0x40D000A4 : & * (volatile uint32_t *)0x40D00008);
          }

        error = SUCCESS;
      }
  }
  return error;
}

static void HplPXA27xInterruptM$enable(uint8_t id)
{
  uint32_t tmp;

  /* atomic removed: atomic calls only */
#line 165
  {
    if (id < 34) {
        *(id & 0x20 ? & * (volatile uint32_t *)0x40D000A0 : & * (volatile uint32_t *)0x40D00004) |= 1 << (id & 0x1f);
        tmp = *(id & 0x20 ? & * (volatile uint32_t *)0x40D000A0 : & * (volatile uint32_t *)0x40D00004);
      }
  }
  return;
}

# 121 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOMCR(uint8_t chnl_id, uint32_t val)
{
  if (chnl_id > 3) {
      * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A000C0 + (uint32_t )((chnl_id - 4) << 2)) = val;
    }
  return;
}

#line 87
static void HplPXA27xOSTimerM$PXA27xOST$setOSCR(uint8_t chnl_id, uint32_t val)
{
  uint8_t remap_id;

  remap_id = chnl_id < 4 ? 0 : chnl_id;
  *(remap_id == 0 ? & * (volatile uint32_t *)0x40A00010 : & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00040 + (uint32_t )((remap_id - 4) << 2))) = val;

  return;
}

# 158 "/opt/tinyos-2.1.2/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(uint8_t pin, uint8_t func)
{
  func &= 0x3;
  * (volatile uint32_t *)((uint32_t )0x40E00054 + (uint32_t )((pin & 0x70) >> 2)) = (* (volatile uint32_t *)((uint32_t )0x40E00054 + (uint32_t )((pin & 0x70) >> 2)) & ~(3 << ((pin & 0x0f) << 1))) | (func << ((pin & 0x0f) << 1));
  return;
}

#line 85
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(uint8_t pin, bool dir)
{
  if (dir) {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E0000C + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E0010C) |= 1 << (pin & 0x1f);
    }
  else {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E0000C + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E0010C) &= ~(1 << (pin & 0x1f));
    }
  return;
}

# 151 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static error_t PMICM$writePMIC(uint8_t address, uint8_t value)
#line 151
{
  PMICM$PI2C$setIDBR(0x49 << 1);
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 0));
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
  while (PMICM$PI2C$getICR() & (1 << 3)) ;

  * (volatile uint32_t *)0x40F00188 = address;
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 0));
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
  while (PMICM$PI2C$getICR() & (1 << 3)) ;

  * (volatile uint32_t *)0x40F00188 = value;
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 1));
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
  while (PMICM$PI2C$getICR() & (1 << 3)) ;
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 1));
#line 166
  * (volatile uint32_t *)0x40F00190 &= ~(1 << 1);

  return SUCCESS;
}

# 134 "/opt/tinyos-2.1.2/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP$Scheduler$runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 138
  {
    nextTask = SchedulerBasicP$popTask();
    if (nextTask == SchedulerBasicP$NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 142
          FALSE;

#line 142
          return __nesc_temp;
        }
      }
  }
#line 145
  SchedulerBasicP$TaskBasic$runTask(nextTask);
  return TRUE;
}

#line 175
static void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id)
{
}

# 75 "/opt/tinyos-2.1.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(uint8_t arg_0x2b4800f867d8){
#line 75
  switch (arg_0x2b4800f867d8) {
#line 75
    case /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer:
#line 75
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired:
#line 75
      /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$runTask();
#line 75
      break;
#line 75
    case /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm:
#line 75
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP$startDone_task:
#line 75
      CC2420CsmaP$startDone_task$runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP$stopDone_task:
#line 75
      CC2420CsmaP$stopDone_task$runTask();
#line 75
      break;
#line 75
    case CC2420CsmaP$sendDone_task:
#line 75
      CC2420CsmaP$sendDone_task$runTask();
#line 75
      break;
#line 75
    case CC2420ControlP$sync:
#line 75
      CC2420ControlP$sync$runTask();
#line 75
      break;
#line 75
    case CC2420ControlP$syncDone:
#line 75
      CC2420ControlP$syncDone$runTask();
#line 75
      break;
#line 75
    case /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$lateAlarm:
#line 75
      /*AlarmMultiplexC.Alarm.Alarm32khzC.PhysAlarm32khz*/HalPXA27xAlarmM$1$lateAlarm$runTask();
#line 75
      break;
#line 75
    case CC2420SpiP$grant:
#line 75
      CC2420SpiP$grant$runTask();
#line 75
      break;
#line 75
    case /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask:
#line 75
      /*IM2CC2420SpiP.FcfsArbiterC.Arbiter*/SimpleArbiterP$0$grantedTask$runTask();
#line 75
      break;
#line 75
    case CC2420ReceiveP$receiveDone_task:
#line 75
      CC2420ReceiveP$receiveDone_task$runTask();
#line 75
      break;
#line 75
    case CC2420TinyosNetworkP$grantTask:
#line 75
      CC2420TinyosNetworkP$grantTask$runTask();
#line 75
      break;
#line 75
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask:
#line 75
      /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask$runTask();
#line 75
      break;
#line 75
    case /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask:
#line 75
      /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$runTask();
#line 75
      break;
#line 75
    default:
#line 75
      SchedulerBasicP$TaskBasic$default$runTask(arg_0x2b4800f867d8);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 163 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$sendDone(uint8_t last, message_t * msg, error_t err)
#line 163
{
  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[last].msg = (void *)0;
  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$tryToSend();
  /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$sendDone(last, msg, err);
}

# 139 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static am_addr_t CC2420ActiveMessageP$AMPacket$destination(message_t *amsg)
#line 139
{
  cc2420_header_t *header = CC2420ActiveMessageP$CC2420PacketBody$getHeader(amsg);

#line 141
  return __nesc_ntoh_leuint16(header->dest.nxdata);
}

#line 87
static error_t CC2420ActiveMessageP$AMSend$send(am_id_t id, am_addr_t addr, 
message_t *msg, 
uint8_t len)
#line 89
{
  unsigned char *__nesc_temp48;
#line 90
  cc2420_header_t *header = CC2420ActiveMessageP$CC2420PacketBody$getHeader(msg);

  if (len > CC2420ActiveMessageP$Packet$maxPayloadLength()) {
      return ESIZE;
    }

  __nesc_hton_leuint8(header->type.nxdata, id);
  __nesc_hton_leuint16(header->dest.nxdata, addr);
  __nesc_hton_leuint16(header->destpan.nxdata, CC2420ActiveMessageP$CC2420Config$getPanAddr());
  __nesc_hton_leuint16(header->src.nxdata, CC2420ActiveMessageP$AMPacket$address());
  (__nesc_temp48 = header->fcf.nxdata, __nesc_hton_leuint16(__nesc_temp48, __nesc_ntoh_leuint16(__nesc_temp48) | (((1 << IEEE154_FCF_INTRAPAN) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE)) | (
  IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE))));
  __nesc_hton_leuint8(header->length.nxdata, len + CC2420_SIZE);

  if (CC2420ActiveMessageP$RadioResource$immediateRequest() == SUCCESS) {
      error_t rc;

#line 107
      CC2420ActiveMessageP$SendNotifier$aboutToSend(id, addr, msg);

      rc = CC2420ActiveMessageP$SubSend$send(msg, len);
      if (rc != SUCCESS) {
          CC2420ActiveMessageP$RadioResource$release();
        }

      return rc;
    }
  else 
#line 115
    {
      CC2420ActiveMessageP$pending_length = len;
      CC2420ActiveMessageP$pending_message = msg;
      return CC2420ActiveMessageP$RadioResource$request();
    }
}

# 106 "/opt/tinyos-2.1.2/tos/system/ActiveMessageAddressC.nc"
static am_addr_t ActiveMessageAddressC$amAddress(void )
#line 106
{
  am_addr_t myAddr;

#line 108
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 108
    myAddr = ActiveMessageAddressC$addr;
#line 108
    __nesc_atomic_end(__nesc_atomic); }
  return myAddr;
}

# 60 "/opt/tinyos-2.1.2/tos/system/FcfsResourceQueueC.nc"
static bool /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void )
#line 60
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
    {
      unsigned char __nesc_temp = 
#line 61
      /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$qHead == /*CC2420TinyosNetworkC.FcfsResourceQueueC*/FcfsResourceQueueC$0$NO_ENTRY;

      {
#line 61
        __nesc_atomic_end(__nesc_atomic); 
#line 61
        return __nesc_temp;
      }
    }
#line 63
    __nesc_atomic_end(__nesc_atomic); }
}

# 80 "/opt/tinyos-2.1.2/tos/chips/cc2420/lowpan/CC2420TinyosNetworkP.nc"
static error_t CC2420TinyosNetworkP$ActiveSend$send(message_t *msg, uint8_t len)
#line 80
{
  CC2420TinyosNetworkP$CC2420Packet$setNetwork(msg, 0x3f);
  CC2420TinyosNetworkP$m_busy_client = CC2420TinyosNetworkP$CLIENT_AM;
  return CC2420TinyosNetworkP$SubSend$send(msg, len);
}

# 90 "/opt/tinyos-2.1.2/tos/chips/cc2420/packet/CC2420PacketP.nc"
static uint8_t * CC2420PacketP$getNetwork(message_t * msg)
#line 90
{
  cc2420_header_t *hdr = CC2420PacketP$CC2420PacketBody$getHeader(msg);
  int offset;

  offset = CC2420PacketP$getAddressLength((__nesc_ntoh_leuint16(hdr->fcf.nxdata) >> IEEE154_FCF_DEST_ADDR_MODE) & 0x3) + 
  CC2420PacketP$getAddressLength((__nesc_ntoh_leuint16(hdr->fcf.nxdata) >> IEEE154_FCF_SRC_ADDR_MODE) & 0x3) + 
  (size_t )& ((cc2420_header_t *)0)->dest;

  return (uint8_t *)hdr + offset;
}

# 825 "/opt/tinyos-2.1.2/tos/chips/cc2420/transmit/CC2420TransmitP.nc"
static void CC2420TransmitP$loadTXFIFO(void )
#line 825
{
  cc2420_header_t *header = CC2420TransmitP$CC2420PacketBody$getHeader(CC2420TransmitP$m_msg);
  uint8_t tx_power = __nesc_ntoh_uint8(CC2420TransmitP$CC2420PacketBody$getMetadata(CC2420TransmitP$m_msg)->tx_power.nxdata);

  if (!tx_power) {
      tx_power = 31;
    }

  CC2420TransmitP$CSN$clr();

  if (CC2420TransmitP$m_tx_power != tx_power) {
      CC2420TransmitP$TXCTRL$write((((2 << CC2420_TXCTRL_TXMIXBUF_CUR) | (
      3 << CC2420_TXCTRL_PA_CURRENT)) | (
      1 << CC2420_TXCTRL_RESERVED)) | ((
      tx_power & 0x1F) << CC2420_TXCTRL_PA_LEVEL));
    }

  CC2420TransmitP$m_tx_power = tx_power;

  {
    uint8_t tmpLen __attribute((unused))  = __nesc_ntoh_leuint8(header->length.nxdata) - 1;

#line 846
    CC2420TransmitP$TXFIFO$write((uint8_t * )header, __nesc_ntoh_leuint8(header->length.nxdata) - 1);
  }
}

# 56 "/opt/tinyos-2.1.2/tos/interfaces/State.nc"
static void UniqueSendP$State$toIdle(void ){
#line 56
  StateImplP$State$toIdle(2U);
#line 56
}
#line 56
# 74 "/opt/tinyos-2.1.2/tos/chips/cc2420/CC2420ActiveMessageP.nc"
static void CC2420ActiveMessageP$RadioResource$granted(void )
#line 74
{
  uint8_t rc;
  cc2420_header_t *header = CC2420ActiveMessageP$CC2420PacketBody$getHeader(CC2420ActiveMessageP$pending_message);

  CC2420ActiveMessageP$SendNotifier$aboutToSend(__nesc_ntoh_leuint8(header->type.nxdata), __nesc_ntoh_leuint16(header->dest.nxdata), CC2420ActiveMessageP$pending_message);
  rc = CC2420ActiveMessageP$SubSend$send(CC2420ActiveMessageP$pending_message, CC2420ActiveMessageP$pending_length);
  if (rc != SUCCESS) {
      CC2420ActiveMessageP$RadioResource$release();
      CC2420ActiveMessageP$AMSend$sendDone(__nesc_ntoh_leuint8(header->type.nxdata), CC2420ActiveMessageP$pending_message, rc);
    }
}

# 189 "/opt/tinyos-2.1.2/tos/system/AMQueueImplP.nc"
static void /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$sendDone(am_id_t id, message_t *msg, error_t err)
#line 189
{





  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current >= 1) {
      return;
    }
  if (/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current].msg == msg) {
      /*AMQueueP.AMQueueImplP*/AMQueueImplP$0$sendDone(/*AMQueueP.AMQueueImplP*/AMQueueImplP$0$current, msg, err);
    }
  else {
      ;
    }
}

# 479 "/opt/tinyos-2.1.2/tos/chips/cc2420/control/CC2420ControlP.nc"
static void CC2420ControlP$writeFsctrl(void )
#line 479
{
  uint8_t channel;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 482
    {
      channel = CC2420ControlP$m_channel;
    }
#line 484
    __nesc_atomic_end(__nesc_atomic); }

  CC2420ControlP$FSCTRL$write((1 << CC2420_FSCTRL_LOCK_THR) | (((
  channel - 11) * 5 + 357) << CC2420_FSCTRL_FREQ));
}







static void CC2420ControlP$writeMdmctrl0(void )
#line 496
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 497
    {
      CC2420ControlP$MDMCTRL0$write((((((((1 << CC2420_MDMCTRL0_RESERVED_FRAME_MODE) | ((
      CC2420ControlP$addressRecognition && CC2420ControlP$hwAddressRecognition ? 1 : 0) << CC2420_MDMCTRL0_ADR_DECODE)) | (
      2 << CC2420_MDMCTRL0_CCA_HYST)) | (
      3 << CC2420_MDMCTRL0_CCA_MOD)) | (
      1 << CC2420_MDMCTRL0_AUTOCRC)) | ((
      CC2420ControlP$autoAckEnabled && CC2420ControlP$hwAutoAckDefault) << CC2420_MDMCTRL0_AUTOACK)) | (
      0 << CC2420_MDMCTRL0_AUTOACK)) | (
      2 << CC2420_MDMCTRL0_PREAMBLE_LENGTH));
    }
#line 506
    __nesc_atomic_end(__nesc_atomic); }
}

# 81 "/opt/tinyos-2.1.2/tos/chips/cc2420/csma/CC2420CsmaP.nc"
static error_t CC2420CsmaP$SplitControl$start(void )
#line 81
{
  if (CC2420CsmaP$SplitControlState$requestState(CC2420CsmaP$S_STARTING) == SUCCESS) {
      CC2420CsmaP$CC2420Power$startVReg();
      return SUCCESS;
    }
  else {
#line 86
    if (CC2420CsmaP$SplitControlState$isState(CC2420CsmaP$S_STARTED)) {
        return EALREADY;
      }
    else {
#line 89
      if (CC2420CsmaP$SplitControlState$isState(CC2420CsmaP$S_STARTING)) {
          return SUCCESS;
        }
      }
    }
#line 93
  return EBUSY;
}

# 132 "/opt/tinyos-2.1.2/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$startAt(uint32_t t0, uint32_t dt)
#line 132
{
  uint32_t tf;
#line 133
  uint32_t t1;
  bool bPending;

#line 135
  tf = t0 + dt;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 137
    {
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(TRUE);
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSMR(tf);
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning = TRUE;
      t1 = /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSCR();
      bPending = /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSSRbit();
      if (dt <= t1 - t0 && !bPending) {
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(FALSE);
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$postTask();
        }
    }
#line 147
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

# 73 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$fireTimers(uint32_t now)
{
  uint16_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 90
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 334 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static error_t PMICM$getPMICADCVal(uint8_t channel, uint8_t *val)
#line 334
{
  uint8_t oldval;
  error_t rval;


  rval = PMICM$readPMIC(0x30, &oldval, 1);
  if (rval == SUCCESS) {
      rval = PMICM$writePMIC(0x30, ((channel & 0x7) | (
      1 << 3)) | (1 << 4));
    }
  if (rval == SUCCESS) {
      rval = PMICM$readPMIC(0x40, val, 1);
    }
  if (rval == SUCCESS) {

      rval = PMICM$writePMIC(0x30, oldval);
    }

  return rval;
}

# 144 "/opt/tinyos-2.1.2/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$m_timers[num];

#line 147
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 43 "/opt/tinyos-2.1.2/tos/platforms/intelmote2/LocalIeeeEui64C.nc"
static ieee_eui64_t LocalIeeeEui64C$LocalIeeeEui64$getId(void )
#line 43
{



  ieee_eui64_t id;

  id.data[0] = 0x00;
  id.data[1] = 0x12;
  id.data[2] = 0x6d;
  id.data[3] = 'L';
  id.data[4] = 'O';
  id.data[5] = 0;
  id.data[6] = TOS_NODE_ID >> 8;
  id.data[7] = TOS_NODE_ID & 0xff;
  return id;
}

