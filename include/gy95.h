typedef struct gy95_t {
    int port;
    int ctrl_pin;
    uint8_t buf[GY95_MSG_LEN];
    int cursor;
    int start_reg;
    int length;
    uint8_t addr;
    bool flag;
} gy95_t;

void gy95_msp_init(gy95_t* p_gy);

void gy95_init(gy95_t* p_gy, int port, int ctrl_pin, int addr);

void gy95_send(gy95_t* p_gy, uint8_t* msg, int len);

void gy95_setup(gy95_t* p_gy);

void gy95_calibrate(gy95_t* p_gy);

void gy95_reset(gy95_t* p_gy);

bool gy95_chksum(gy95_t* p_gy);

void gy95_read(gy95_t* p_gy);

void gy95_enable(void);

void gy95_disable(void);