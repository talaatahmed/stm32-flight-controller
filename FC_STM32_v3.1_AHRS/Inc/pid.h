typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float integral_limit;   /* anti-windup clamp */
    float output_limit;     /* max output magnitude */
} PID_t;

/* Prototypes */
void  PID_Init(PID_t *pid,
               float Kp, float Ki, float Kd,
               float integral_limit,
               float output_limit);

float PID_Update(PID_t *pid,
                 float setpoint,
                 float measurement,
                 float dt);

void  PID_Reset(PID_t *pid);
