#pragma once
#include <string.h>  // memset
#include <stdint.h>

typedef struct {
    float x[3];       // estado: [T, w, c]
    float P[3][3];    // covarianza
    float Q[3][3];    // proceso
    float R[3][3];    // medición
    uint8_t inited;
} kf3_t;

static inline void mat3_add(float A[3][3], const float B[3][3]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) A[i][j]+=B[i][j];
}
static inline void mat3_sub(float A[3][3], const float B[3][3]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) A[i][j]-=B[i][j];
}
static inline void mat3_mul(const float A[3][3], const float B[3][3], float C[3][3]) {
    float t[3][3]; memset(t,0,sizeof(t));
    for (int i=0;i<3;i++) for (int k=0;k<3;k++) for (int j=0;j<3;j++) t[i][j]+=A[i][k]*B[k][j];
    memcpy(C,t,sizeof(t));
}
static inline void mat3_vec(const float A[3][3], const float v[3], float y[3]) {
    float t[3]={0,0,0};
    for (int i=0;i<3;i++) for (int k=0;k<3;k++) t[i]+=A[i][k]*v[k];
    y[0]=t[0]; y[1]=t[1]; y[2]=t[2];
}
static inline void mat3_T(const float A[3][3], float AT[3][3]) {
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) AT[j][i]=A[i][j];
}
static inline float clampf(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }

static inline void kf3_init(kf3_t* kf, const float x0[3], float P0_diag, float Q_diag[3], float R_diag[3]) {
    memcpy(kf->x, x0, 3*sizeof(float));
    memset(kf->P, 0, sizeof(kf->P));
    for (int i=0;i<3;i++) kf->P[i][i]=P0_diag;

    memset(kf->Q, 0, sizeof(kf->Q));
    memset(kf->R, 0, sizeof(kf->R));
    for (int i=0;i<3;i++){ kf->Q[i][i]=Q_diag[i]; kf->R[i][i]=R_diag[i]; }
    kf->inited = 1u;
}

/* Predicción: x = A x + B u + E d  ;  P = A P A^T + Q  */
static inline void kf3_predict(kf3_t* kf,
                               const float A[3][3],
                               const float B[3][2], const float u[2],
                               const float E[3][4], const float d[4])
{
    float Ax[3]; mat3_vec(A, kf->x, Ax);

    float Bu[3] = {0,0,0};
    for (int i=0;i<3;i++) for (int j=0;j<2;j++) Bu[i]+=B[i][j]*u[j];

    float Ed[3] = {0,0,0};
    for (int i=0;i<3;i++) for (int j=0;j<4;j++) Ed[i]+=E[i][j]*d[j];

    for (int i=0;i<3;i++) kf->x[i] = Ax[i] + Bu[i] + Ed[i];

    float AP[3][3]; mat3_mul(A, kf->P, AP);
    float AT[3][3]; mat3_T(A, AT);
    float APA[3][3]; mat3_mul(AP, AT, APA);
    mat3_add(APA, kf->Q);
    memcpy(kf->P, APA, sizeof(APA));
}

/* Actualización (C = I): z = x + v  */
static inline void kf3_update(kf3_t* kf, const float z[3]) {
    // y = z - x
    float y[3]; for(int i=0;i<3;i++) y[i]=z[i]-kf->x[i];

    // S = P + R
    float S[3][3]; memcpy(S, kf->P, sizeof(S)); mat3_add(S, kf->R);

    // Inversa de 3x3 (fórmula cerrada)
    float a=S[0][0], b=S[0][1], c=S[0][2];
    float d=S[1][0], e=S[1][1], f=S[1][2];
    float g=S[2][0], h=S[2][1], i=S[2][2];
    float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    if (det == 0.0f) return; // evita NaN en caso degenerado
    float inv = 1.0f/det;
    float Sinv[3][3] = {
        { (e*i - f*h)*inv, (c*h - b*i)*inv, (b*f - c*e)*inv },
        { (f*g - d*i)*inv, (a*i - c*g)*inv, (c*d - a*f)*inv },
        { (d*h - e*g)*inv, (b*g - a*h)*inv, (a*e - b*d)*inv }
    };

    // K = P * S^{-1}
    float K[3][3]; mat3_mul(kf->P, Sinv, K);

    // x = x + K y
    float Ky[3] = {0,0,0};
    for (int r=0;r<3;r++) for (int c=0;c<3;c++) Ky[r]+=K[r][c]*y[c];
    for (int j=0;j<3;j++) kf->x[j]+=Ky[j];

    // P = (I-K) P  (forma de Joseph simplificada con C=I)
    float IminusK[3][3] = {
        {1-K[0][0], -K[0][1], -K[0][2]},
        {-K[1][0], 1-K[1][1], -K[1][2]},
        {-K[2][0], -K[2][1], 1-K[2][2]}
    };
    float newP[3][3]; mat3_mul(IminusK, kf->P, newP);
    memcpy(kf->P, newP, sizeof(newP));
}
