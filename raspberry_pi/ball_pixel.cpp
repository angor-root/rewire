// ball_pixel — control de la pelota SOLO con la imagen (sin ArUco ni pose del plato), 20-oct.
//
// Camara -> pelota blanca en pixeles -> error respecto a un centro fijo de la imagen -> metros con una escala fija ->
// PD (+ I) aqui mismo -> 'H T <roll> <pitch>' a la PRIZM, que lo pasa por la IK exacta del plato de rotulas a los servos.
// (El PD de gimbal_fw, 'H 1', sigue disponible: este programa tambien manda 'O' con la pelota.)
// Ejes (medidos 20-oct con el tamano de los tags): roll + -> la pelota rueda a +x de la imagen; pitch + -> a +y (arriba).
// Reemplaza a vision_node + prizm_bridge: pararlos antes (usan la camara y el puerto).
//
//   gcc -O2 -c policy.c && g++ -O2 -pthread -o ~/ball_pixel ball_pixel.cpp policy.o $(pkg-config --cflags --libs opencv4)
//   (policy.c, policy.h y obs_balltrack.h de deploy/c junto a este archivo)
// Politica de RL (31-oct, DSM-BallTrack-v0): --policy balltrack.bin la corre en cada cuadro en SOMBRA (filas
//   "s,<t>,<u>,<v>,<vis>,<a0..a3>,<roll>,<pitch>" en el log, sin mover nada); "POL 1" por la FIFO le da el plato ("POL K 0.5" escala su accion)
//   (las ruedas quedan en 0, robot quieto), "POL 0" vuelve al PD. Ejes: --cam_m a b c d  (u = a*x + b*y, v = c*x + d*y,
//   x/y de la imagen en m -> u adelante / v izquierda de la base, como en la sim) y --plate_m e f g h
//   (roll_H = e*pitch_sim + f*roll_sim, pitch_H = g*pitch_sim + h*roll_sim, grados). Por defecto identidad: SIN calibrar.
//   ~/ball_pixel [--cx 318] [--cy 232] [--escala 2050] [--puerto /dev/ttyACMx] [--exp 50] [--vmin 60] [--roi 90 0 540 440] [--http 8080]
//   Video en vivo (31-oct): http://<ip de la Pi>:8080 en el navegador (camara con la pelota | mascara, ~10 cuadros/s;
//   solo gasta CPU con alguien mirando). Sin PRIZM arranca igual (solo camara).
//
// FIFO /tmp/ball_cmd:  "PD 1" / "PD 0" (control on / off y plato nivelado),  "PD K <kp> <kd> <ki> <lim>" (grados de plato
// por m, por m/s, por m*s, limite en grados),  "PD E <emax m> <slew grados/s>" (saturacion del error y de la velocidad
// de inclinacion: menos agresivo en los bordes),  "REC 1" / "REC 0" (video de la camara con la pelota marcada y la hora
// en /tmp/ball_rec.avi, MJPG; cada cuadro deja una fila "r,<t>,<n>" en el log). Cualquier otra linea va a la PRIZM.
// Log: /tmp/ball.log (pelota medida + todo lo que responde la PRIZM como "p,<t>,<linea>", 28-oct; t = steady_clock, el
// mismo reloj que imu_pi). Imagen: /tmp/ball.jpg cada 1 s.  deploy/tools/sesion.py graba y grafica una sesion.
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <glob.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include "obs_balltrack.h"
#define BP_HIST 10  // historial del ball-and-plate (sim/ballplate/bp_env.py HIST)
extern "C" {
#include "policy.h"  // runtime en C (deploy/c/policy.c)
}
#include <netinet/in.h>
#include <sys/socket.h>

static double now_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

static int open_serial(const char* path) {
  const int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;
  termios t{};
  tcgetattr(fd, &t);
  cfmakeraw(&t);
  cfsetspeed(&t, B115200);
  tcsetattr(fd, TCSANOW, &t);
  return fd;
}

int main(int argc, char** argv) {
  double cx = 318, cy = 232, escala = 2050, exp = 50, vmin = 60;  // vmin: brillo minimo de la pelota (0-255)
  int roi[4] = {90, 0, 540, 440};  // solo dentro del plato (x0 y0 x1 y1, px): afuera hay cables y conectores blancos  // escala: px por metro (tags a 155 mm = 318 px)
  int http_port = 8080;  // 0 = sin video en vivo
  const char* pol_path = nullptr;
  // IMU del chasis (31-oct): imu_pi --udp 127.0.0.1:5007 manda "E X ux uy uz ..." (vertical en ejes del chasis). La
  // inclinacion del chasis se compensa sumando al plato  M (u - u_ref)  (grados), tanto al PD como a la politica.
  // M sale de una regresion (deploy/tools/imu_ejes.py); FIFO: "IMU ON" / "IMU OFF" / "IMU REF" / "IMU M a b c d".
  float imu_m[4] = {0, 0, 0, 0};
  float cam_m[4] = {1, 0, 0, 1}, plate_m[4] = {1, 0, 0, 1};
  std::string puerto;  // vacio = el primer /dev/ttyACM*
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    if (k == "--cx") cx = atof(argv[i + 1]);
    else if (k == "--cy") cy = atof(argv[i + 1]);
    else if (k == "--escala") escala = atof(argv[i + 1]);
    else if (k == "--puerto") puerto = argv[i + 1];
    else if (k == "--exp") exp = atof(argv[i + 1]);
    else if (k == "--vmin") vmin = atof(argv[i + 1]);
    else if (k == "--http") http_port = atoi(argv[i + 1]);
    else if (k == "--policy") pol_path = argv[i + 1];
    else if (k == "--imu_m" && i + 4 < argc) { for (int j = 0; j < 4; ++j) imu_m[j] = atof(argv[i + 1 + j]); i += 3; }
    else if ((k == "--cam_m" || k == "--plate_m") && i + 4 < argc) {
      for (int j = 0; j < 4; ++j) (k == "--cam_m" ? cam_m : plate_m)[j] = atof(argv[i + 1 + j]);
      i += 3;
    }
    else if (k == "--roi" && i + 4 < argc) { for (int j = 0; j < 4; ++j) roi[j] = atoi(argv[i + 1 + j]); i += 3; }
  }
  glob_t g;
  std::string cam = "/dev/video0";
  if (glob("/dev/v4l/by-id/*-video-index0", 0, nullptr, &g) == 0 && g.gl_pathc) cam = g.gl_pathv[0];
  cv::VideoCapture cap(cam, cv::CAP_V4L2);
  std::mutex cam_mx;  // ultima imagen del hilo lector
  std::condition_variable cam_cv;
  cv::Mat cam_frame;
  unsigned long cam_seq = 0, cam_used = 0;
  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap.set(cv::CAP_PROP_FPS, 30);
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // V4L2: 1 = manual
  cap.set(cv::CAP_PROP_EXPOSURE, exp);     // en 100 us (50 = 5 ms, igual que vision_node)
  if (!cap.isOpened()) { fprintf(stderr, "no abre la camara %s\n", cam.c_str()); return 1; }
  // 31-oct: hilo lector. Antes el lazo (22-25 Hz) leia de la cola del driver mientras la camara entregaba 30 fps: la
  // cola quedaba llena y siempre se procesaba la imagen mas vieja (identificacion: ~0.34 s de retardo total; con 1
  // buffer bajo a ~0.19 s pero el lazo cayo a 15 Hz). Ahora el hilo vacia la cola sin parar y el lazo toma la ultima.
  std::thread([&] {
    cv::Mat f;
    for (;;) {
      if (!cap.read(f) || f.empty()) { usleep(10000); continue; }
      { std::lock_guard<std::mutex> lk(cam_mx); f.copyTo(cam_frame); ++cam_seq; }
      cam_cv.notify_one();
    }
  }).detach();
  if (puerto.empty()) {  // la PRIZM aparece como ttyACM0 o ttyACM1 segun el orden en que se conecta / reinicia
    glob_t gp;
    puerto = glob("/dev/ttyACM*", 0, nullptr, &gp) == 0 && gp.gl_pathc ? gp.gl_pathv[0] : "/dev/ttyACM0";
  }
  int ser = open_serial(puerto.c_str());
  // shortcut: sin PRIZM sigue solo con la camara (para mirar el video); si se conecta despues, reiniciar ball_pixel
  if (ser < 0) { fprintf(stderr, "AVISO no abre %s: sin PRIZM, solo camara\n", puerto.c_str()); ser = open("/dev/null", O_RDWR); }
  int srv = -1;
  std::vector<int> viewers;
  if (http_port > 0) {
    srv = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    const int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(http_port);
    if (bind(srv, (sockaddr*)&a, sizeof a) || listen(srv, 4)) { fprintf(stderr, "AVISO sin video: puerto %d\n", http_port); srv = -1; }
  }
  double t_http = 0;
  {  // 31-oct: si un script escribio antes de que exista la FIFO, "echo >" dejo un archivo normal: borrarlo y crearla
    struct stat st;
    if (stat("/tmp/ball_cmd", &st) == 0 && !S_ISFIFO(st.st_mode)) unlink("/tmp/ball_cmd");
  }
  mkfifo("/tmp/ball_cmd", 0666);
  const int fifo = open("/tmp/ball_cmd", O_RDONLY | O_NONBLOCK);
  FILE* log = fopen("/tmp/ball.log", "w");
  fprintf(stderr, "ball_pixel: %s, centro (%.0f, %.0f) px, %.0f px/m\n", cam.c_str(), cx, cy, escala);

  cv::Mat frame, small, hsv, mask;
  const cv::Mat k5 = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5});
  unsigned long seq = 0;
  bool pd_on = false, have = false;
  Policy pol{};
  bool pol_ok = false, pol_on = false, pol_bp = false;
  // ball-and-plate: [x*10, y*10, visible, accion anterior (2)] (+ v4: acel x y en m/s2, giro x y en rad/s), [0] = mas viejo
  float bp_h[BP_HIST][9];
  bool pol_imu = false, pol_tr = false;
  // objetivo de los trucos (mismas formulas que tr_env.py._target): modo 0 punto, 1 circulo, 2 ocho, 3 cuadrado
  int tr_mode = 0;
  double tr_p[3] = {0, 0, 0}, tr_t0 = now_s();  // v4 (02-nov, bp_env.py imu=True): la politica ve la IMU de frente; no se suma M (u - u_ref)
  bool bp_empty = true;
  float bp_tgt[2] = {0, 0};  // consigna tras el limitador de 40 grados/s (sim/ballplate v2 tiene el mismo)
  double bp_t = 0;
  if (pol_path) {
    char err[200];
    pol_ok = policy_load(&pol, pol_path, err, sizeof err) == 0;
    // dos tipos: DSM-BallTrack (90 -> 4) o el ball-and-plate de sim/ballplate (BP_HIST*5 -> 2, ejes del firmware)
    // trucos (09-nov, sim/ballplate/trucos): las 90 de v4 + objetivo [tp*10, tv*5, ta*2] = 96 -> 2; "TR ..." lo mueve
    pol_tr = pol_ok && pol.dims[0] == BP_HIST * 9 + 6 && pol.dims[pol.n_layers] == 2;
    pol_imu = pol_ok && (pol.dims[0] == BP_HIST * 9 || pol_tr) && pol.dims[pol.n_layers] == 2;
    pol_bp = pol_ok && (pol.dims[0] == BP_HIST * 5 || pol_imu) && pol.dims[pol.n_layers] == 2;
    pol_ok = pol_ok && (pol_bp || (pol.dims[0] == OBT_DIM && pol.dims[pol.n_layers] == 4));
    if (pol_imu) fprintf(stderr, "politica v4 con IMU: entra acel + giro de imu_pi, sin la compensacion M\n");
    fprintf(stderr, pol_ok ? "politica %s cargada (sombra; POL 1 para usarla)\n" : "AVISO politica %s NO cargada: %s\n",
            pol_path, pol_ok ? "" : err);
  }
  ObsBT obs_bt;
  obs_bt_reset(&obs_bt);
  float pol_trim[2] = {0, 0};  // "POL T <roll> <pitch>": cero del plato (desnivel que el PD sostiene con su integral)
  float pol_k = 1;  // "POL K <escala>": escala de la accion del plato (diagnostico sim->real, 31-oct)
  float pol_u = 0, pol_v = 0, pol_a[4] = {0, 0, 0, 0}, pol_roll = 0, pol_pitch = 0;
  double t_pol_seen = 0;
  // "EXC <amplitud grados> <segundos>" (31-oct): excitacion pseudoaleatoria +-amp por eje SUMADA a lo que pide el PD,
  // para identificar el plato (sim/ballplate/identificar.py); cada eje cambia de signo cada 0.08-0.5 s al azar
  int imu_sock = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(5007);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(imu_sock, (sockaddr*)&a, sizeof a)) { fprintf(stderr, "AVISO sin IMU: puerto 5007\n"); imu_sock = -1; }
  }
  bool imu_on = false, imu_have_ref = false;
  double imu_t = -1;
  float imu_u[3] = {0, 0, 1}, imu_ref[2] = {0, 0}, imu_c[2] = {0, 0}, imu_w[3] = {0, 0, 0}, imu_al[3] = {0, 0, 0};
  // 09-nov: la pelota siente g (u - u_ref) + a_lin (al acelerar la base se va como en una rampa); "IMU A k" escala la
  // parte de aceleracion (0 = como antes). Filtro de 1er orden (alfa 0.5 por cuadro) contra los picos de vibracion
  float imu_ka = 1.0f, imu_alf[2] = {0, 0};
  double exc_amp = 0, exc_end = 0, exc_next[2] = {0, 0};
  double exc_val[2] = {0, 0};
  double kp = 30, kd = 22, ki = 6, lim = 8;  // barrido 31-oct: ki 6 = mediana 0.7 cm (ki 15: 1.2 cm; 40/28/6 oscila a 1.7 s)
  double px = 0, py = 0, vx = 0, vy = 0, ix = 0, iy = 0, t_seen = 0;
  bool leveled = true;
  double emax = 0.05;      // m: el error de posicion se satura aqui (lejos del centro no empuja mas fuerte)
  double slew = 40;        // grados/s: cambio maximo de la inclinacion
  double roll_c = 0, pitch_c = 0;
  // LQR con retardo (02-nov, sim/ballplate/lqr.py): "PD L <n> L1..Ln" suma L_j * (consigna de hace j cuadros) a la ley
  // del PD; asi la ley ve lo que el plato ya tiene pedido y en camino (~0.19 s de retardo). n = 0 -> PD normal.
  int n_l = 0;
  double pd_l[16] = {0}, h_roll[16] = {0}, h_pitch[16] = {0};
  int n_win = 0, n_ok = 0;
  double t_win = now_s(), t_img = 0;
  std::string rx, cmdbuf;
  char buf[512];
  cv::VideoWriter rec;
  std::atomic<bool> rec_busy{false};  // 31-oct: el video se escribe en otro hilo (en este lazo bajaba el control a 6 Hz)
  long n_rec = 0;
  while (true) {
    {  // la imagen mas nueva del hilo lector (ver abajo); si no hay una nueva todavia, esperar
      std::unique_lock<std::mutex> lk(cam_mx);
      cam_cv.wait_for(lk, std::chrono::milliseconds(200), [&] { return cam_seq != cam_used; });
      if (cam_seq == cam_used) continue;
      cam_used = cam_seq;
      cam_frame.copyTo(frame);
    }
    const double t = now_s();
    // pelota blanca: poco saturada y clara; se elige el blob mas grande y redondo de 25-70 px de radio (los bordes
    // blancos de los tags son cuadrados de ~100 px)
    cv::resize(frame, small, {}, 0.5, 0.5, cv::INTER_AREA);
    cv::cvtColor(small, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, vmin), cv::Scalar(180, 120, 255), mask);  // S<=120 (antes 60): con poca luz la pelota crema da S~67 y se partia (11-nov)
    cv::Mat keep = cv::Mat::zeros(mask.size(), CV_8U);
    keep(cv::Rect(cv::Point(roi[0] / 2, roi[1] / 2), cv::Point(roi[2] / 2, roi[3] / 2)) & cv::Rect({}, mask.size())) = 255;
    mask &= keep;
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k5);
    std::vector<std::vector<cv::Point>> cs;
    cv::findContours(mask, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double best = 0, bu = 0, bv = 0, br = 0, cand_r = 0, cand_fill = 0;  // cand: el blob mas grande, aunque se rechace
    for (const auto& c : cs) {
      const double a = cv::contourArea(c);
      cv::Point2f ctr;
      float r;
      cv::minEnclosingCircle(c, ctr, r);
      if (2 * r > cand_r) { cand_r = 2 * r; cand_fill = a / (CV_PI * r * r); }
      if (2 * r < 25 || 2 * r > 70 || a / (CV_PI * r * r) < 0.70) continue;
      if (a > best) { best = a; bu = 2 * ctr.x; bv = 2 * ctr.y; br = 2 * r; }
    }
    const bool vis = best > 0;
    const double bx = vis ? (bu - cx) / escala : 0, by = vis ? -(bv - cy) / escala : 0;  // y de imagen hacia arriba
    // O <seq> <bx> <by> <vis> <pitch> <roll> <gx> <gy> <gz> <vx> <wz>  (lo demas no lo usa el modo H)
    const int n = snprintf(buf, sizeof buf, "O %lu %.5f %.5f %d 0 0 0 0 -1 0 0\n", ++seq, bx, by, vis ? 1 : 0);
    if (write(ser, buf, n) != n) fprintf(stderr, "error de escritura\n");
    for (char ib[256]; imu_sock >= 0;) {  // la ultima linea de imu_pi
      const ssize_t r = recv(imu_sock, ib, sizeof ib - 1, 0);
      if (r <= 0) break;
      ib[r] = 0;
      if (sscanf(ib, "E X %f %f %f %f %f %f %f %f %f", &imu_u[0], &imu_u[1], &imu_u[2], &imu_w[0], &imu_w[1], &imu_w[2],
                 &imu_al[0], &imu_al[1], &imu_al[2]) == 9) imu_t = t;
    }
    const bool imu_fresh = imu_t > 0 && t - imu_t < 0.2;
    if (imu_fresh && !imu_have_ref) { imu_ref[0] = imu_u[0]; imu_ref[1] = imu_u[1]; imu_have_ref = true; }
    for (int k = 0; k < 2; ++k)
      imu_alf[k] = imu_fresh ? 0.5f * imu_alf[k] + 0.5f * imu_al[k] : 0;
    for (int k = 0; k < 2; ++k)
      imu_c[k] = imu_on && imu_fresh ? imu_m[2 * k] * (imu_u[0] - imu_ref[0] + imu_ka * imu_alf[0] / 9.81f) +
                                       imu_m[2 * k + 1] * (imu_u[1] - imu_ref[1] + imu_ka * imu_alf[1] / 9.81f) : 0;
    if (imu_fresh) fprintf(log, "i,%.3f,%.5f,%.5f,%.5f,%.2f,%.2f\n", t, imu_u[0], imu_u[1], imu_u[2], imu_c[0], imu_c[1]);
    if (pol_ok) {  // politica: en sombra siempre; con POL 1 manda el plato
      if (vis) {  // sin pelota se repite la ultima posicion con visible = 0, como CameraMeasurement en la sim
        pol_u = cam_m[0] * bx + cam_m[1] * by;
        pol_v = cam_m[2] * bx + cam_m[3] * by;
        t_pol_seen = t;
      }
      // robot quieto en piso plano: gravedad (0, 0, -1), ruedas 0, trayectoria "quieto" (error 0, cos 1, v = w = 0)
      if (pol_bp) {  // sim/ballplate: historial por paso (el mas nuevo al final), salida [roll_H, pitch_H] en +-1 x 12 grados
        // v4: acel = g (u - u_ref) + alin (lo que mide el acelerometro menos el arranque) y giro; sin IMU fresca, ceros
        const float g_on = imu_fresh ? 1.0f : 0.0f;
        const float st[9] = {pol_u * 10, pol_v * 10, vis ? 1.0f : 0.0f, pol_a[0], pol_a[1],
                             g_on * (9.81f * (imu_u[0] - imu_ref[0]) + imu_al[0]),
                             g_on * (9.81f * (imu_u[1] - imu_ref[1]) + imu_al[1]), g_on * imu_w[0], g_on * imu_w[1]};
        const int S = pol_imu ? 9 : 5;
        float* h = &bp_h[0][0];  // historial contiguo de BP_HIST x S
        if (bp_empty) { for (int r = 0; r < BP_HIST; ++r) memcpy(h + r * S, st, sizeof(float) * S); bp_tgt[0] = bp_tgt[1] = 0; bp_t = t; }
        else { memmove(h, h + S, sizeof(float) * S * (BP_HIST - 1)); memcpy(h + S * (BP_HIST - 1), st, sizeof(float) * S); }
        bp_empty = false;
        float a[2], o[BP_HIST * 9 + 6];
        if (pol_tr) {
          const double tt = t - tr_t0, A = tr_p[0], B = tr_p[1], W = tr_p[2];
          double tp[2] = {0, 0}, tv[2] = {0, 0}, ta[2] = {0, 0};
          if (tr_mode == 0) { tp[0] = A; tp[1] = B; }
          else if (tr_mode == 1) {  // circulo radio A, W rad/s
            const double c = cos(W * tt), sn = sin(W * tt);
            tp[0] = A * c; tp[1] = A * sn; tv[0] = -W * A * sn; tv[1] = W * A * c; ta[0] = -W * W * tp[0]; ta[1] = -W * W * tp[1];
          } else if (tr_mode == 2) {  // ocho: Lissajous 1:2, semiejes A B
            const double ph = W * tt;
            tp[0] = A * sin(ph); tp[1] = B * sin(2 * ph); tv[0] = A * W * cos(ph); tv[1] = 2 * B * W * cos(2 * ph);
            ta[0] = -W * W * tp[0]; ta[1] = -4 * W * W * tp[1];
          } else {  // cuadrado de semilado A recorrido a B m/s (signo = sentido)
            static const int C[4][2] = {{1, 1}, {-1, 1}, {-1, -1}, {1, -1}}, D[4][2] = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
            const double u = fmod(fabs(B) * tt, 8 * A), mir = B < 0 ? -1 : 1;
            const int k = std::min((int)(u / (2 * A)), 3);
            const double f = u - k * 2 * A;
            tp[0] = (C[k][0] * A + D[k][0] * f) * mir; tp[1] = C[k][1] * A + D[k][1] * f;
            tv[0] = D[k][0] * fabs(B) * mir; tv[1] = D[k][1] * fabs(B);
          }
          memcpy(o, h, sizeof(float) * BP_HIST * 9);
          for (int k = 0; k < 2; ++k) {
            o[BP_HIST * 9 + k] = tp[k] * 10; o[BP_HIST * 9 + 2 + k] = tv[k] * 5; o[BP_HIST * 9 + 4 + k] = ta[k] * 2;
          }
          fprintf(log, "tr,%.3f,%d,%.4f,%.4f\n", t, tr_mode, tp[0], tp[1]);
        }
        policy_run(&pol, pol_tr ? o : h, a);
        for (int k = 0; k < 2; ++k) pol_a[k] = std::min(std::max(a[k], -1.0f), 1.0f);
        const float dmax = 40.0f * std::min(std::max(t - bp_t, 0.01), 0.1);
        bp_t = t;
        for (int k = 0; k < 2; ++k) bp_tgt[k] += std::min(std::max(pol_a[k] * 12 - bp_tgt[k], -dmax), dmax);
        pol_roll = bp_tgt[0] * pol_k + pol_trim[0];
        pol_pitch = bp_tgt[1] * pol_k + pol_trim[1];
        fprintf(log, "s,%.3f,%.4f,%.4f,%d,%.3f,%.3f,0,0,%.2f,%.2f\n", t, pol_u, pol_v, vis ? 1 : 0, a[0], a[1], pol_roll,
                pol_pitch);
      } else {
      const float step[OBT_STEP] = {pol_u, pol_v, vis ? 1.0f : 0.0f, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0,
                                    0, 0, pol_a[2], pol_a[3]};  // ultima accion aplicada: ruedas en 0
      float o[OBT_DIM], a[4];
      obs_bt_push(&obs_bt, step, o);
      policy_run(&pol, o, a);
      for (int k = 0; k < 4; ++k) pol_a[k] = a[k];
      // limitador de 40 grados/s en convencion de la sim, igual que ServoPlateAction (slew_deg_s) desde el 31-oct
      if (bp_t == 0) bp_t = t;  // primer paso tras un reinicio
      const float dm = 40.0f * std::min(std::max(t - bp_t, 0.01), 0.1);
      bp_t = t;
      bp_tgt[0] += std::min(std::max(std::min(std::max(a[2], -1.0f), 1.0f) * 15 - bp_tgt[0], -dm), dm);
      bp_tgt[1] += std::min(std::max(std::min(std::max(a[3], -1.0f), 1.0f) * 15 - bp_tgt[1], -dm), dm);
      const float ps = bp_tgt[0] * pol_k, rs = bp_tgt[1] * pol_k;
      pol_roll = plate_m[0] * ps + plate_m[1] * rs + pol_trim[0];
      pol_pitch = plate_m[2] * ps + plate_m[3] * rs + pol_trim[1];
      fprintf(log, "s,%.3f,%.4f,%.4f,%d,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n", t, pol_u, pol_v, vis ? 1 : 0, a[0], a[1], a[2],
              a[3], pol_roll, pol_pitch);
      }
    }
    if (pol_on && t - t_pol_seen < 0.5) {
      const float ic = pol_imu ? 0.0f : 1.0f;  // la politica v4 ya ve la IMU: no compensar dos veces
      const int k = snprintf(buf, sizeof buf, "H T %.2f %.2f\n", pol_roll + ic * imu_c[0], pol_pitch + ic * imu_c[1]);
      if (write(ser, buf, k) != k) fprintf(stderr, "error de escritura\n");
      leveled = false;
    } else if (pol_on && !leveled) {  // sin pelota medio segundo: nivelar y empezar de nuevo
      if (write(ser, "H T 0 0\n", 8) != 8) fprintf(stderr, "error de escritura\n");
      leveled = true;
      obs_bt_reset(&obs_bt);
      bp_empty = true;
      bp_tgt[0] = bp_tgt[1] = 0;
      bp_t = 0;
      for (float& x : pol_a) x = 0;
    } else if (!pol_on && pd_on && vis) {
      const double dt = std::min(std::max(t - t_seen, 0.01), 0.2);
      const bool fresh = !have || t - t_seen > 0.15;  // reaparece: sin derivada (el salto no es velocidad)
      vx = fresh ? 0 : 0.6 * vx + 0.4 * (bx - px) / dt;
      vy = fresh ? 0 : 0.6 * vy + 0.4 * (by - py) / dt;
      if (ki > 0) {
        ix = std::min(std::max(ix + std::min(std::max(bx, -emax), emax) * dt, -lim / ki), lim / ki);
        iy = std::min(std::max(iy + std::min(std::max(by, -emax), emax) * dt, -lim / ki), lim / ki);
      }
      px = bx; py = by; have = true; t_seen = t; leveled = false;
      const double ex = std::min(std::max(bx, -emax), emax), ey = std::min(std::max(by, -emax), emax);
      double pr = 0, pp = 0;
      for (int j = 0; j < n_l; ++j) { pr += pd_l[j] * h_roll[j]; pp += pd_l[j] * h_pitch[j]; }
      const double roll_t = std::min(std::max(-(kp * ex + kd * vx + ki * ix + pr), -lim), lim);
      const double pitch_t = std::min(std::max(-(kp * ey + kd * vy + ki * iy + pp), -lim), lim);
      const double dmax = slew * dt;
      roll_c += std::min(std::max(roll_t - roll_c, -dmax), dmax);
      pitch_c += std::min(std::max(pitch_t - pitch_c, -dmax), dmax);
      for (int j = 15; j > 0; --j) { h_roll[j] = h_roll[j - 1]; h_pitch[j] = h_pitch[j - 1]; }
      h_roll[0] = roll_c; h_pitch[0] = pitch_c;  // lo pedido tras el limitador (sin IMU ni excitacion), como en lqr.py
      if (t < exc_end) {
        for (int k = 0; k < 2; ++k)
          if (t >= exc_next[k]) { exc_val[k] = (rand() & 1) ? exc_amp : -exc_amp; exc_next[k] = t + 0.08 + 0.42 * rand() / (double)RAND_MAX; }
        fprintf(log, "e,%.3f,%.2f,%.2f\n", t, exc_val[0], exc_val[1]);
      } else exc_val[0] = exc_val[1] = 0;
      const double roll = roll_c + exc_val[0] + imu_c[0], pitch = pitch_c + exc_val[1] + imu_c[1];
      const int k = snprintf(buf, sizeof buf, "H T %.2f %.2f\n", roll, pitch);
      if (write(ser, buf, k) != k) fprintf(stderr, "error de escritura\n");
      fprintf(log, "c,%.3f,%.4f,%.4f,%.3f,%.3f,%.2f,%.2f\n", t, bx, by, vx, vy, roll, pitch);
    } else if (pd_on && t - t_seen > 0.5 && !leveled) {  // sin pelota medio segundo: nivelar y olvidar
      if (write(ser, "H T 0 0\n", 8) != 8) fprintf(stderr, "error de escritura\n");
      leveled = true; have = false; ix = iy = 0; roll_c = pitch_c = 0;
      for (int j = 0; j < 16; ++j) h_roll[j] = h_pitch[j] = 0;
    }
    fprintf(log, "b,%.3f,%d,%.1f,%.1f,%.4f,%.4f,%.0f,%.2f\n", t, vis, bu, bv, bx, by, cand_r, cand_fill);
    static double t_lost = 0;
    static int n_lost = 0;
    if (!vis && t - t_lost > 0.7) {  // diagnostico: por que no la ve
      t_lost = t;
      // 11-nov: en otro hilo; el JPEG en este lazo metia un hueco de >80 ms cada 0.7 s justo al perder la pelota
      std::thread([f = frame.clone(), mk = mask.clone(), n = n_lost++ % 10] {
        cv::Mat m3, both;
        cv::cvtColor(mk, m3, cv::COLOR_GRAY2BGR);
        cv::resize(m3, m3, f.size());
        cv::hconcat(f, m3, both);
        cv::imwrite("/tmp/perdida_" + std::to_string(n) + ".jpg", both);
      }).detach();
    }
    ++n_win;
    n_ok += vis;

    // comandos de la FIFO -> PRIZM
    ssize_t m;
    while (fifo >= 0 && (m = read(fifo, buf, sizeof buf)) > 0) {
      cmdbuf.append(buf, m);
      size_t p;
      while ((p = cmdbuf.find('\n')) != std::string::npos) {
        const std::string l = cmdbuf.substr(0, p + 1);
        cmdbuf.erase(0, p + 1);
        if (l.rfind("PD", 0) == 0) {
          double a, b2, c, d;
          if (sscanf(l.c_str(), "PD K %lf %lf %lf %lf", &a, &b2, &c, &d) == 4) { kp = a; kd = b2; ki = c; lim = d; }
          else if (sscanf(l.c_str(), "PD E %lf %lf", &a, &b2) == 2) { emax = a; slew = b2; }
          else if (l.rfind("PD L", 0) == 0) {
            std::istringstream in(l.substr(4));
            int m = 0;
            in >> m;
            m = std::min(std::max(m, 0), 16);
            int got = 0;
            for (; got < m && (in >> pd_l[got]); ++got) {}
            n_l = got == m ? m : 0;  // incompleto -> PD normal
            for (int j = 0; j < 16; ++j) h_roll[j] = h_pitch[j] = 0;
            fprintf(log, "OK PD L n=%d%s\n", n_l, got == m ? "" : " (incompleto: PD normal)");
          }
          else if (l[3] == '1') { pd_on = true; have = false; ix = iy = 0; }
          else if (l[3] == '0') { pd_on = false; roll_c = pitch_c = 0; if (write(ser, "H T 0 0\n", 8) != 8) fprintf(stderr, "error\n"); }
          fprintf(log, "OK PD on=%d kp=%.1f kd=%.1f ki=%.1f lim=%.1f emax=%.3f slew=%.0f\n", pd_on, kp, kd, ki, lim, emax, slew);
        } else if (l.rfind("IMU", 0) == 0) {
          if (l.rfind("IMU ON", 0) == 0) imu_on = true;
          else if (l.rfind("IMU OFF", 0) == 0) imu_on = false;
          else if (l.rfind("IMU REF", 0) == 0) {  // "IMU REF" = como esta ahora; "IMU REF ux uy" = fija (09-nov)
            if (sscanf(l.c_str() + 7, "%f %f", &imu_ref[0], &imu_ref[1]) != 2) { imu_ref[0] = imu_u[0]; imu_ref[1] = imu_u[1]; }
            imu_have_ref = true;
          }
          else if (l.rfind("IMU A", 0) == 0) imu_ka = std::min(std::max((float)atof(l.c_str() + 5), 0.0f), 1.5f);
          else if (l.rfind("IMU M", 0) == 0) sscanf(l.c_str() + 5, "%f %f %f %f", &imu_m[0], &imu_m[1], &imu_m[2], &imu_m[3]);
          fprintf(log, "OK IMU A=%.2f\n", imu_ka);
          fprintf(log, "OK IMU on=%d recibe=%d M=%.1f %.1f %.1f %.1f ref=%.4f %.4f\n", imu_on, imu_t > 0 && t - imu_t < 0.2,
                  imu_m[0], imu_m[1], imu_m[2], imu_m[3], imu_ref[0], imu_ref[1]);
        } else if (l.rfind("EXC", 0) == 0) {
          double amp = 0, dur = 0;
          sscanf(l.c_str() + 3, "%lf %lf", &amp, &dur);
          exc_amp = std::min(std::max(amp, 0.0), 4.0);
          exc_end = t + dur;
          fprintf(log, "OK EXC %.2f grados %.1f s\n", exc_amp, dur);
        } else if (l.rfind("TR", 0) == 0) {
          // TR P x y (cm) | TR C r(cm) w(rad/s) | TR 8 a(cm) b(cm) w(rad/s) | TR Q semilado(cm) v(cm/s); rangos de la sim
          char c = 'P';
          double a = 0, b2 = 0, w = 0;
          const int got = sscanf(l.c_str() + 2, " %c %lf %lf %lf", &c, &a, &b2, &w);
          const auto cl = [](double x, double lo, double hi) { return std::min(std::max(x, lo), hi); };
          if (c == 'C' && got >= 4 - 1) { tr_mode = 1; tr_p[0] = cl(a, 2, 5) / 100; tr_p[2] = cl(b2, -3, 3); }
          else if (c == '8' && got == 4) { tr_mode = 2; tr_p[0] = cl(a, 3, 5) / 100; tr_p[1] = cl(b2, 2, 3.5) / 100; tr_p[2] = cl(w, -1.8, 1.8); }
          else if (c == 'Q' && got >= 3) { tr_mode = 3; tr_p[0] = cl(a, 2.5, 5) / 100; tr_p[1] = cl(b2, -8, 8) / 100; }
          else { tr_mode = 0; tr_p[0] = got >= 2 ? cl(a, -5, 5) / 100 : 0; tr_p[1] = got >= 3 ? cl(b2, -5, 5) / 100 : 0; }
          tr_t0 = t;
          fprintf(log, "OK TR modo=%d %.3f %.3f %.2f (politica de trucos: %d)\n", tr_mode, tr_p[0], tr_p[1], tr_p[2], pol_tr);
        } else if (l.rfind("POL T", 0) == 0) {
          sscanf(l.c_str() + 5, "%f %f", &pol_trim[0], &pol_trim[1]);
          fprintf(log, "OK POL T %.2f %.2f\n", pol_trim[0], pol_trim[1]);
        } else if (l.rfind("POL K", 0) == 0) {
          pol_k = std::min(std::max((float)atof(l.c_str() + 5), 0.0f), 1.25f);  // 1.25 = 15 grados y 50 grados/s (v5, 10-nov)
          fprintf(log, "OK POL K %.2f\n", pol_k);
        } else if (l.rfind("POL", 0) == 0) {
          pol_on = pol_ok && l.size() > 4 && l[4] == '1';
          obs_bt_reset(&obs_bt);
          bp_empty = true;
          bp_tgt[0] = bp_tgt[1] = 0;
          bp_t = 0;
          for (float& x : pol_a) x = 0;
          if (!pol_on && write(ser, "H T 0 0\n", 8) != 8) fprintf(stderr, "error\n");
          fprintf(log, "OK POL on=%d cargada=%d\n", pol_on, pol_ok);
        } else if (l.rfind("REC", 0) == 0) {
          if (l.size() > 4 && l[4] == '1' && !rec.isOpened()) {
            rec.open("/tmp/ball_rec.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, frame.size());
            n_rec = 0;
          } else if (l.size() > 4 && l[4] == '0') {
            while (rec_busy) usleep(1000);  // que termine el cuadro en curso
            rec.release();
          }
          fprintf(log, "OK REC %d %.3f\n", rec.isOpened() ? 1 : 0, t);
        } else if (write(ser, l.data(), l.size()) < 0) fprintf(stderr, "error de escritura\n");
        fprintf(log, "> %s", l.c_str());
      }
    }
    // respuestas de la PRIZM -> log (sin la telemetria periodica)
    while ((m = read(ser, buf, sizeof buf)) > 0) {
      rx.append(buf, m);
      size_t p;
      while ((p = rx.find('\n')) != std::string::npos) {
        const std::string l = rx.substr(0, p);
        rx.erase(0, p + 1);
        if (l.rfind("T,", 0) && l.rfind("E,", 0)) fprintf(log, "p,%.3f,%s\n", now_s(), l.c_str());
      }
    }
    fflush(log);
    // video: pelota, centro, zona y la hora (la misma de los logs); si el hilo sigue ocupado se salta el cuadro
    if (rec.isOpened() && !rec_busy) {
      cv::Mat v = frame.clone();
      cv::drawMarker(v, {int(cx), int(cy)}, {0, 0, 255}, cv::MARKER_CROSS, 30, 2);
      cv::rectangle(v, {roi[0], roi[1]}, {roi[2], roi[3]}, {255, 200, 0}, 1);
      if (vis) cv::circle(v, {int(bu), int(bv)}, int(br), {0, 255, 0}, 2);
      char txt[96];
      snprintf(txt, sizeof txt, "t=%.3f  x=%+.1f y=%+.1f cm", t, 100 * bx, 100 * by);
      cv::putText(v, txt, {8, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2);
      rec_busy = true;
      std::thread([&rec, &rec_busy, log, v, t, n = n_rec++] {
        rec.write(v);
        fprintf(log, "r,%.3f,%ld\n", t, n);
        rec_busy = false;
      }).detach();
    }
    for (int c; srv >= 0 && (c = accept4(srv, nullptr, nullptr, SOCK_NONBLOCK)) >= 0;) {
      const char* h = "HTTP/1.0 200 OK\r\nCache-Control: no-cache\r\nContent-Type: multipart/x-mixed-replace; boundary=f\r\n\r\n";
      if (send(c, h, strlen(h), MSG_NOSIGNAL) > 0) viewers.push_back(c); else close(c);
    }
    if (!viewers.empty() && t - t_http > 0.1) {  // video en vivo: cuadro con la pelota | mascara, a media resolucion
      t_http = t;
      cv::Mat v, m3, both;
      cv::resize(frame, v, {}, 0.5, 0.5, cv::INTER_AREA);
      cv::drawMarker(v, {int(cx / 2), int(cy / 2)}, {0, 0, 255}, cv::MARKER_CROSS, 16, 1);
      cv::rectangle(v, {roi[0] / 2, roi[1] / 2}, {roi[2] / 2, roi[3] / 2}, {255, 200, 0}, 1);
      if (vis) cv::circle(v, {int(bu / 2), int(bv / 2)}, int(br / 2), {0, 255, 0}, 2);
      char txt[96];
      snprintf(txt, sizeof txt, vis ? "x %+.1f y %+.1f cm" : "NO SE VE", 100 * bx, 100 * by);
      cv::putText(v, txt, {6, 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 1);
      snprintf(txt, sizeof txt, "PD %s %.0f/%.0f/%.0f  vmin %.0f", pd_on ? "ON" : "off", kp, kd, ki, vmin);
      cv::putText(v, txt, {6, 232}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 255, 255}, 1);
      cv::cvtColor(mask, m3, cv::COLOR_GRAY2BGR);
      cv::hconcat(v, m3, both);
      std::vector<uchar> jpg;
      cv::imencode(".jpg", both, jpg, {cv::IMWRITE_JPEG_QUALITY, 60});
      char hd[96];
      const int hn = snprintf(hd, sizeof hd, "--f\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", jpg.size());
      for (size_t i = 0; i < viewers.size();) {  // un cuadro a medias rompe el flujo: ese visor se cierra (recargar la pagina)
        const int c = viewers[i];
        const bool ok = send(c, hd, hn, MSG_NOSIGNAL) == hn &&
                        send(c, jpg.data(), jpg.size(), MSG_NOSIGNAL) == (ssize_t)jpg.size() &&
                        send(c, "\r\n", 2, MSG_NOSIGNAL) == 2;
        if (ok) ++i; else { close(c); viewers.erase(viewers.begin() + i); }
      }
    }
    if (t - t_img > 1.0) {  // imagen para revisar desde la laptop
      t_img = t;
      cv::drawMarker(frame, {int(cx), int(cy)}, {0, 0, 255}, cv::MARKER_CROSS, 30, 2);
      cv::rectangle(frame, {roi[0], roi[1]}, {roi[2], roi[3]}, {255, 200, 0}, 1);
      if (vis) cv::circle(frame, {int(bu), int(bv)}, int(br), {0, 255, 0}, 2);
      std::thread([f = frame.clone()] { cv::imwrite("/tmp/ball.jpg", f); }).detach();  // fuera del lazo de control
    }
    if (t - t_win > 2.0) {
      fprintf(stderr, "%.1f Hz, pelota vista %d%%, ultima (%+.1f, %+.1f) cm\n", n_win / (t - t_win), 100 * n_ok / n_win,
              100 * bx, 100 * by);
      n_win = n_ok = 0;
      t_win = t;
    }
  }
}
