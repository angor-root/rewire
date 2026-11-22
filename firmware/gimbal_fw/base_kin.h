/*
 * base_kin — cinematica de la base diferencial del DSM y seguimiento de trayectorias (C puro, sin Arduino)
 *
 * Frame de la base (igual que la sim, robot_cfg.py): +X adelante (lado de las omni), +Y izquierda, +Z arriba.
 * Punto de referencia = punto medio entre las dos ruedas motrices (las omni no imponen restricciones).
 * Ruedas: + = hacia ADELANTE en ambas (convencion del firmware, MOTOR_SIGN ya la aplica).
 *
 *   IK (cuerpo -> ruedas):  wL = (v - w*b/2) / r      wR = (v + w*b/2) / r
 *   FK (ruedas -> cuerpo):  v  = r*(wR + wL)/2        w  = r*(wR - wL)/b
 *   r = radio de rueda, b = via (distancia entre centros de contacto de las ruedas motrices)
 *
 * Seguimiento: ley de Kanayama (1990) sobre la odometria de encoders:
 *   e = R(-th) (p_ref - p),  v = v_r cos(e_th) + kx e_x,  w = w_r + v_r (ky e_y + kth sin(e_th))
 * Es la que se usa en bases diferenciales: estable para v_r > 0 y no necesita el modelo dinamico.
 *
 * Todo en SI (m, rad, s). Lo usa gimbal_fw y la prueba de PC test/test_base_kin.c.
 */
#ifndef BASE_KIN_H
#define BASE_KIN_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float r;      /* radio de rueda (m) */
  float track;  /* via efectiva (m) */
  float w_max;  /* |velocidad de rueda| maxima (rad/s): se escala v y w juntos para no deformar la curva */
} BaseGeom;

/* 11-nov: r 0.0514 y via 0.335 medidos por Caleb en el robot (los mismos que debe usar el VI del myRIO). Antes
   0.0488 / 0.364 del URDF (sim/dsm_rl_lab/robot_cfg.py, que sigue con esos). 10 rad/s = 0.51 m/s. */
#define BASE_GEOM_DEFAULT {0.0514f, 0.335f, 10.0f}

typedef struct { float x, y, th; } Pose2;

/* modo de la base en gimbal_fw (aqui y no en el .ino: el preprocesador de Arduino declara las funciones antes) */
typedef enum { BASE_OFF, BASE_TWIST, BASE_CIRCLE, BASE_TELEOP } BaseMode;

/* cuerpo (v, w) -> ruedas (wL, wR) en rad/s, con saturacion proporcional a w_max. Devuelve 1 si satura. */
int base_ik(const BaseGeom* g, float v, float w, float* wl, float* wr);
/* ruedas (rad/s) -> cuerpo */
void base_fk(const BaseGeom* g, float wl, float wr, float* v, float* w);
/* odometria: integra desplazamientos de rueda (rad) con el punto medio del arco */
void base_odom(const BaseGeom* g, Pose2* p, float dphi_l, float dphi_r);
float wrap_pi(float a);

/* ---------------- trayectoria: circulo ---------------- */
typedef struct {
  /* parametros */
  float radius;   /* m */
  float speed;    /* m/s sobre la trayectoria */
  float accel;    /* m/s^2, rampa de arranque y frenado (protege la pelota y evita patinar) */
  int dir;        /* +1 antihorario (izquierda), -1 horario */
  float laps;     /* vueltas; <= 0 = sin fin */
  float kx, ky, kth;  /* ganancias de Kanayama: 1/s, 1/m^2, 1/m (w = w_r + v_r (ky e_y + kth sin e_th)) */
  /* estado */
  float cx, cy, th0;  /* centro y rumbo inicial (fijados en circle_start con la pose del robot) */
  float s, sdot;      /* arco recorrido y velocidad de referencia */
  int done;
  Pose2 ref;          /* pose de referencia del ultimo paso */
  float v_ref, w_ref; /* feedforward del ultimo paso */
} CircleTraj;

void circle_init(CircleTraj* c, float radius, float speed, int dir, float laps);
/* fija el circulo tangente a la pose actual (el robot arranca SOBRE la trayectoria, mirando a lo largo de ella) */
void circle_start(CircleTraj* c, const Pose2* p);
/* avanza dt y devuelve (v, w) de mando; con done = 1 manda 0 */
void circle_step(CircleTraj* c, const Pose2* p, float dt, float* v, float* w);
/* frena con la rampa (en vez de cortar en seco) */
void circle_finish(CircleTraj* c);

#ifdef __cplusplus
}
#endif
#endif
