typedef enum {
    POS_MODE_HALL = 0,
    POS_MODE_BEMF
} PosMode_t;

typedef struct {
    float elec_angle;   // electrical angle [rad]
    float elec_speed;   // electrical speed [rad/s]
    float mech_speed;   // mech speed [rpm]
    uint8_t sector;     // 0..5 for 6-step
    bool valid;
} PosEst_t;

void PosEst_init(PosMode_t mode);
void PosEst_setMode(PosMode_t mode);

// called every fast-control step
void PosEst_update(void);

PosEst_t PosEst_get(void);