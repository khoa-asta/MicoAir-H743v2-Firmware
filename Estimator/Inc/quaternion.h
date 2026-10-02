/**
 *  Quaternion Hamilton [w, x, y, z].
 */
#ifndef QUATERNION_H
#define QUATERNION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* q_nb doi vector body FRD sang navigation NED hoac he cuc bo da khai bao. */
/* Goc dung radian; toc do goc dung rad/s; thoi gian dung giay. */
/* Ham bool tra false khi loi va giu nguyen dau ra. */
/* Dau ra quaternion duoc phep trung dau vao quaternion. */
/* Hai ham quay vector cho phep vector dau ra trung vector dau vao. */
/* Khong ho tro chong lan mot phan hoac chong lan du lieu khac kieu. */

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternion_t;

/** Tao [1, 0, 0, 0], bieu dien hai he truc trung nhau. */
bool Quaternion_Identity(Quaternion_t *out);

/** Tra norm; NAN neu dau vao loi, INFINITY neu norm tran float. */
float Quaternion_Norm(const Quaternion_t *q);

/** Chuan hoa ve norm bang 1; tu choi norm nho hon 1e-6. */
bool Quaternion_Normalize(const Quaternion_t *q, Quaternion_t *out);

/** Lay lien hop [w, -x, -y, -z], khong chuan hoa. */
bool Quaternion_Conjugate(const Quaternion_t *q, Quaternion_t *out);

/** Lay lien hop chia norm binh phuong; tu choi norm nho hon 1e-6. */
bool Quaternion_Inverse(const Quaternion_t *q, Quaternion_t *out);

/** @brief Tinh out = a nhan b theo Hamilton, khong tu chuan hoa. */
bool Quaternion_Multiply(const Quaternion_t *a, const Quaternion_t *b, Quaternion_t *out);

/** @brief Exp: vector quay radian sang quaternion; truyen toan bo goc quay. */
bool Quaternion_FromRotationVector(const float rotation_rad[3], Quaternion_t *out);

/** @brief Log: vector quay ngan nhat, goc trong [0, pi]; tu chuan hoa q. */
bool Quaternion_ToRotationVector(const Quaternion_t *q, float rotation_rad[3]);

/** @brief Tao R_nb theo hang r[3 * row + col]; tu chuan hoa q. */
bool Quaternion_ToRotationMatrix(const Quaternion_t *q_nb, float r_nb[9]);

/** @brief Kiem tra R_nb voi dung sai 1e-3, tao q don vi co w khong am. */
bool Quaternion_FromRotationMatrix(const float r_nb[9], Quaternion_t *out);

/** @brief Tinh v_n = R_nb * v_b; tu chuan hoa q, giu don vi vector. */
bool Quaternion_RotateVector(const Quaternion_t *q_nb, const float v_b[3], float v_n[3]);

/** @brief Tinh v_b = R_nb chuyen vi nhan v_n; tu chuan hoa q. */
bool Quaternion_RotateVectorInverse(const Quaternion_t *q_nb, const float v_n[3], float v_b[3]);

/** @brief Tao q theo Rz(yaw) * Ry(pitch) * Rx(roll), cac goc dung radian. */
bool Quaternion_FromEulerZYX(float roll_rad, float pitch_rad, float yaw_rad, Quaternion_t *out);

/** @brief Tinh q nhan Exp(omega * dt); omega body da tru bg, dt khong am. */
bool Quaternion_IntegrateBodyRate(const Quaternion_t *q_nb, const float omega_body_rad_s[3], float dt_s, Quaternion_t *out);

/** @brief Tinh Exp(delta_theta_n) nhan q; chi sua tu the, chua reset P. */
bool Quaternion_ApplyGlobalError(const Quaternion_t *q_hat, const float delta_theta_n_rad[3], Quaternion_t *out);

#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_H */
