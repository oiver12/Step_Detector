/*
 * peakDetection.h
 *
 *  Created on: Dec 12, 2024
 *      Author: olive
 */

#ifndef INC_PEAKDETECTION_H_
#define INC_PEAKDETECTION_H_

typedef struct {
    double delta;
    int is_detecting_emi;  /* 1 if detecting emission peak, 0 if detecting absorption */
    double mx;
    double mn;
    int mx_pos;
    int mn_pos;

    int *emi_peaks;
    int num_emi_peaks;
    int max_emi_peaks;

    int *absop_peaks;
    int num_absop_peaks;
    int max_absop_peaks;

    int current_index; // Tracks the index of the incoming samples
} PeakDetector;

void peak_detector_init(
    PeakDetector *pd,
    double delta,
    int *emi_peaks, int max_emi_peaks,
    int *absop_peaks, int max_absop_peaks,
    int emi_first
);
int peak_detector_add_measurement(PeakDetector *pd, double measurement);

#endif /* INC_PEAKDETECTION_H_ */
