/** @file estimator.h
 *  @brief ESKF 15 sai so, Hamilton, body FRD sang navigation z xuong.
 */
#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "quaternion.h"
#include <stdbool.h>
#include <stdint.h>

#define ESTIMATOR_ERROR_SIZE 15

typedef struct
{
    float gyro_noise;              /* rad/s/sqrt(Hz), mat do nhieu. */
    float accel_noise;             /* m/s2/sqrt(Hz), mat do nhieu. */
    float gyro_bias_walk;          /* rad/s/sqrt(s), random walk bias. */
    float accel_bias_walk;         /* m/s2/sqrt(s), random walk bias. */
    float gravity_std;             /* m/s2, do lech chuan phep do gia trong luc. */
    float mag_std_ut;              /* uT, do lech chuan phep do tu truong. */
    float baro_std_m;              /* m, do lech chuan do cao. */
    float accel_norm_gate;         /* m/s2, nguong lech khoi g. */
    float mag_norm_gate;           /* Ti le thay doi bien do so voi luc khoi tao. */
    float innovation_gate;         /* So sigma, gate tung thanh phan. */
    float declination_rad;         /* Lech tu duong ve Dong; 0 la Bac tu. */
    float max_dt_s;                /* Khong ep dt loi ve nguong. */
} Estimator_Config_t;

typedef struct
{
    Quaternion_t q_nb;
    float velocity_n_mps[3];
    float position_n_m[3];
    float gyro_bias_rad_s[3];
    float accel_bias_mps2[3];
    float body_rate_rad_s[3];
    float height_m;                /* -position_n_m[2], duong len. */
    float climb_rate_mps;          /* -velocity_n_mps[2], duong len. */
    bool numerical_ok;             /* Khong phai co cho phep bay. */
    bool tilt_aided;               /* Phep do gravity duoc nhan gan day. */
    bool yaw_aided;                /* Phep do mag duoc nhan gan day. */
    bool height_aided;             /* Phep do baro duoc nhan gan day. */
    bool horizontal_aided;         /* Luon false o ban nay. */
} Estimator_Output_t;

/* Dat bien nay static/global, khong dat tren stack task IMU. */
/* Moi instance chi duoc mot task so huu. Khong goi tu ISR. */
typedef struct
{
    Quaternion_t q_nb;
    float v_n[3];
    float p_n[3];
    float bg[3];
    float ba[3];
    float last_gyro[3];
    float p[ESTIMATOR_ERROR_SIZE][ESTIMATOR_ERROR_SIZE];
    Estimator_Config_t config;
    float mag_reference_n[3];
    float mag_reference_norm;
    float baro_reference_m;
    float accel_age_s;
    float mag_age_s;
    float baro_age_s;
    float last_nis;
    uint32_t rejected_accel;
    uint32_t rejected_mag;
    uint32_t rejected_baro;
    bool initialized;
    bool healthy;
    bool mag_enabled;
    /* Bo nho lam viec dung lai, khong malloc. */
    float matrix[ESTIMATOR_ERROR_SIZE][ESTIMATOR_ERROR_SIZE];
    float temp[ESTIMATOR_ERROR_SIZE][ESTIMATOR_ERROR_SIZE];
    float work_p[ESTIMATOR_ERROR_SIZE][ESTIMATOR_ERROR_SIZE];
} Estimator_t;

void Estimator_DefaultConfig(Estimator_Config_t *config);
/* acc/gyro la trung binh khi dung yen, FRD va SI. mag=NULL de dung yaw cuc bo. */
/* Tu ke phai da hieu chuan va doi sang FRD; altitude_m cung moc voi UpdateBaro. */
bool Estimator_Init(Estimator_t *est, const Estimator_Config_t *config, const float accel_mps2[3], const float gyro_rad_s[3], const float mag_body_ut[3], float altitude_m);
bool Estimator_Predict(Estimator_t *est, const float gyro_rad_s[3], const float accel_mps2[3], float dt_s);
/* Chi goi voi mau accel MOI, tan so goi goi y 50 Hz. low_dynamics do caller xac nhan. */
bool Estimator_UpdateAccel(Estimator_t *est, const float accel_mps2[3], bool low_dynamics);
bool Estimator_UpdateMag(Estimator_t *est, const float mag_body_ut[3]);
bool Estimator_UpdateBaro(Estimator_t *est, float altitude_m);
bool Estimator_GetOutput(const Estimator_t *est, Estimator_Output_t *out);

#endif
