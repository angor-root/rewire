// IMU MPU6050 del chasis en el puerto I2C de expansion de la PRIZM (Wire: SDA GPIO 8, SCL GPIO 9). 27-oct.
#pragma once
#include <stdint.h>

extern bool imu_ok;          // MPU respondiendo
extern float imu_u[3];       // "arriba" del mundo en ejes del chasis (x adelante, y izquierda, z arriba)
extern float imu_w[3];       // giro del chasis sin sesgo (rad/s)
extern float imu_alin[3];    // aceleracion del chasis sin gravedad, filtrada (m/s2, ejes del chasis)
extern float imu_psi;        // rumbo acumulado desde el arranque (rad, + = izquierda)
extern float imu_aorg[3];    // aceleracion sin gravedad en el ORIGEN del chasis (m/s2), la de la odometria (28-oct)
extern bool imu_aorg_ok;     // la ultima 'E X' la trajo (imu_pi nuevo) y esta fresca
extern bool imu_still;       // lo pone loop(): ruedas quietas -> se puede recalibrar el sesgo del giro

void imu_begin();            // despues de PrizmBegin (abre Wire en 8/9). Robot QUIETO ~1 s: mide el sesgo del giro
void imu_tick();             // en loop(): lee a 200 Hz y filtra
float imu_take_dpsi();       // giro alrededor de la vertical desde la llamada anterior (rad)
float imu_pitch();           // rad, + = nariz arriba
float imu_roll();            // rad, + = lado izquierdo arriba
void imu_cmd(char* a);       // comandos 'E'
