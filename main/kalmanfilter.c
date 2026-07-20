#include "kalmanfilter.h"

#include "esp_log.h"

static const char *TAG = "KALMAN";
void kalman_roll_update(kalman_filter_t* kf, float accel_angle, float angular_gyro, float dt) {
    //Parameters for the Kalman filter
    // Process noise covariance
    float Q[3][3] ={
        {0.000005, 0, 0},
        {0, 0.05, 0},
        {0, 0, 0.00001}
    }; 

    // Measurement noise covariance from datasheet
    float R[2][2] ={
        {0.0002, 0},
        {0, 0.003}
    };
    
    // State transition matrix
    float A[3][3] = {
        {1, dt, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    // Observation matrix
    float H[2][3] = {
        {1, 0, 0},
        {0, 1, 1}
    };

    float xk_minus[3]= {0.0f, 0.0f, 0.0f}; // Predicted state
    float P_minus[3][3]; // Predicted covariance
    float K[3][2]; // Kalman gain

    //Predict the next state xk-=Axk-1
    for (int i = 0; i < 3; i++) {
        xk_minus[i] = A[i][0] * kf->xk[0] + A[i][1] * kf->xk[1] + A[i][2] * kf->xk[2];
    }

    // Predict the covariance pk-=Apk-1A^T + Q
    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_minus[i][j] = Q[i][j];
            for(int k = 0; k < 3; k++) {
                for(int l = 0; l < 3; l++) {
                    P_minus[i][j] += A[i][l] * kf->P[l][k] * A[j][k];
                }
            }
        }
    }

    //Compute the Kalman gain K=pk-H^T(HPkH^T+R)^-1
        // Compute the innovation covariance S=HPkH^T+R
    float S[2][2]; // Innovation covariance
    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 2; j++) {
            S[i][j] = R[i][j];
            for(int k = 0; k < 3; k++) {
                for(int l = 0; l < 3; l++) {
                    S[i][j] += H[i][l] * P_minus[l][k] * H[j][k];
                }
            }
        }
    }

    float S_inv[2][2]; // Inverse of innovation covariance
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (det != 0) {
        S_inv[0][0] = S[1][1] / det;
        S_inv[0][1] = -S[0][1] / det;
        S_inv[1][0] = -S[1][0] / det;
        S_inv[1][1] = S[0][0] / det;
    } else {
        // Handle singular matrix case
        ESP_LOGE(TAG, "Singular matrix in Kalman filter update");
        return;
    }

    // Compute the Kalman gain K=pkH^TS^-1
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 2; j++) {
            K[i][j] = 0;
            for(int k=0; k < 2; k++) {
                for(int l = 0; l < 3; l++) {
                    K[i][j] += P_minus[i][l] * H[k][l]*S_inv[k][j];
                }
            }
        }
    }

    //Predict Xk=Xk_minus + K(zk-HXk_minus)
    int k=sizeof(H)/sizeof(H[0]);
    float h_xk[k]; 

    for(int i=0; i<k; i++){
        h_xk[i] = H[i][0] * xk_minus[0] + H[i][1] * xk_minus[1]+ H[i][2] * xk_minus[2];
    }

    float zk_hxk[2] = {accel_angle-h_xk[0], angular_gyro-h_xk[1]}; // Measurement vector

    for(int i = 0; i < 3; i++) {
        kf->xk[i] = xk_minus[i];
        for(int j = 0; j < 2; j++) {
            kf->xk[i] += K[i][j] * zk_hxk[j];
        }
    }

    // Update the covariance Pk=(I-KH)Pk_minus
    float I[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    float i_minus_kh[3][3];

    for(int i=0; i < 3; i++) {
        for(int j=0; j < 3; j++) {
            i_minus_kh[i][j] = I[i][j];
            for(int k=0; k < 2; k++) {
                i_minus_kh[i][j] -= K[i][k] * H[k][j];
            }
        }
    }

    for(int i=0; i < 3; i++) {
        for(int j=0; j < 3; j++) {
            kf->P[i][j] = 0;
            for(int k=0; k < 3; k++) {
                kf->P[i][j] += i_minus_kh[i][k] * P_minus[k][j];
            }
        }
    }




}