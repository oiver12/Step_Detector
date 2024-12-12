/*
 * peakDetection.c
 *
 *  Created on: Dec 12, 2024
 *      Author: olive
 */
#include "peakDetection.h"

/* Initialize the peak detector state */
void peak_detector_init(
    PeakDetector *pd,
    double delta,
    int *emi_peaks, int max_emi_peaks,
    int *absop_peaks, int max_absop_peaks,
    int emi_first
) {
    pd->delta = delta;
    pd->is_detecting_emi = emi_first ? 1 : 0;

    // Initialize these after the first sample arrives
    pd->mx = 0.0;
    pd->mn = 0.0;
    pd->mx_pos = 0;
    pd->mn_pos = 0;

    pd->emi_peaks = emi_peaks;
    pd->absop_peaks = absop_peaks;

    pd->num_emi_peaks = 0;
    pd->num_absop_peaks = 0;

    pd->max_emi_peaks = max_emi_peaks;
    pd->max_absop_peaks = max_absop_peaks;

    pd->current_index = -1; // No samples yet
}

/* Add a single measurement to the detector and update peaks if found */
int peak_detector_add_measurement(PeakDetector *pd, double measurement) {
    pd->current_index++;
    int i = pd->current_index;

    // If this is the first data point, initialize mx and mn
    if (i == 0) {
        pd->mx = measurement;
        pd->mn = measurement;
        pd->mx_pos = i;
        pd->mn_pos = i;
        return 0; // No peaks detected yet
    }

    // Update max and min if needed
    if (measurement > pd->mx) {
        pd->mx = measurement;
        pd->mx_pos = i;
    }

    if (measurement < pd->mn) {
        pd->mn = measurement;
        pd->mn_pos = i;
    }

    // Check conditions for peak detection
    if (pd->is_detecting_emi && (measurement < pd->mx - pd->delta)) {
        // Found an emission peak
        if (pd->num_emi_peaks < pd->max_emi_peaks) {
            pd->emi_peaks[pd->num_emi_peaks++] = pd->mx_pos;
        } else {
            return 1; // Error: not enough space for emission peaks
        }

        // Switch mode to absorption detection
        pd->is_detecting_emi = 0;
        // Reset the minimum reference based on the peak just found
        pd->mn = pd->mx;
        pd->mn_pos = pd->mx_pos;
    } else if (!pd->is_detecting_emi && (measurement > pd->mn + pd->delta)) {
        // Found an absorption peak
        if (pd->num_absop_peaks < pd->max_absop_peaks) {
            pd->absop_peaks[pd->num_absop_peaks++] = pd->mn_pos;
        } else {
            return 2; // Error: not enough space for absorption peaks
        }

        // Switch mode to emission detection
        pd->is_detecting_emi = 1;
        // Reset the maximum reference based on the trough just found
        pd->mx = pd->mn;
        pd->mx_pos = pd->mn_pos;
    }

    return 0;
}

