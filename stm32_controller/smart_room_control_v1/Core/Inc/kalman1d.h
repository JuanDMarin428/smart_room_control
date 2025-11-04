/**
 * Kalman1D - Minimal scalar Kalman filter (random-walk model)
 *
 * x_k = x_{k-1} + w_k,      w_k ~ N(0, Q)
 * z_k = x_k       + v_k,    v_k ~ N(0, R)
 *
 * Tuning:
 *  - R: sensor variance (ruido del sensor). A mayor R, más suaviza.
 *  - Q: modelo (cuánto creemos que cambia el valor real entre muestras).
 *       Si tu señal cambia rápido, sube Q; si es lenta, baja Q.
 *
 * @module kalman1d
 */

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;   // estimated value
    float P;   // estimation covariance
    float Q;   // process noise variance
    float R;   // measurement noise variance
    uint8_t inited;
} kf1d_t;

/**
 * Initialize filter with initial guess.
 *
 * @param kf    Filter struct
 * @param x0    Initial value
 * @param P0    Initial covariance (e.g. 1.0f)
 * @param Q     Process noise variance
 * @param R     Measurement noise variance
 */
static inline void kf1d_init(kf1d_t *kf, float x0, float P0, float Q, float R) {
    kf->x = x0; kf->P = P0; kf->Q = Q; kf->R = R; kf->inited = 1u;
}

/**
 * Update with a new measurement z. If not initialized, seeds with z.
 *
 * @param kf    Filter
 * @param z     Measurement
 * @return      Current estimate after update
 */
static inline float kf1d_update(kf1d_t *kf, float z) {
    if (!kf->inited) {
        kf1d_init(kf, z, 1.0f, kf->Q > 0 ? kf->Q : 1e-4f, kf->R > 0 ? kf->R : 1e-2f);
        return kf->x;
    }
    // Predict
    float x_pred = kf->x;          // A=1
    float P_pred = kf->P + kf->Q;  // P + Q

    // Update
    float y  = z - x_pred;         // innovation
    float S  = P_pred + kf->R;     // innovation covariance
    float K  = P_pred / S;         // Kalman gain

    kf->x = x_pred + K * y;
    kf->P = (1.0f - K) * P_pred;
    return kf->x;
}

/** Optional helpers to retune on the fly */
static inline void kf1d_set_QR(kf1d_t *kf, float Q, float R){ kf->Q = Q; kf->R = R; }
static inline float kf1d_get_x(const kf1d_t *kf){ return kf->x; }

#ifdef __cplusplus
}
#endif
