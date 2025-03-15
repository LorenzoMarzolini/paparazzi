/* optical_flow_obs.c
 *
 * Questo modulo calcola l’optic flow mediante una versione semplificata del metodo Lucas-Kanade 
 * e individua ostacoli usando una finestra scorrevole. Solo la fascia centrale (3/5 dell’altezza)
 * viene considerata. Per evitare calcoli costosi (radici quadrate) si confronta il quadrato della
 * magnitudine.
 */

#include "OpticalFlow_Obstacle.h"
#include "modules/computer_vision/cv.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include <float.h>
#include <limits.h>

// --- DEFINIZIONI MACRO ---
#define IMAGE_WIDTH         520
#define IMAGE_HEIGHT        240
#define POINT_SPACING       20    // Distanza in pixel tra i punti per il calcolo dell'optic flow
#define WINDOW_SIZE         6
#define OBSTACLE_THRESHOLD  4.0f
#define OBSTACLE_THRESHOLD_SQ (OBSTACLE_THRESHOLD * OBSTACLE_THRESHOLD)
#define SLIDING_WINDOW_WIDTH (IMAGE_WIDTH / 6)
#define SLIDING_WINDOW_STEP  4
#define STEERING_GAIN        0.1f   // Fattore di scala per l'angolo in modalità SAFE (gradi per pixel)
#define SPIN_GAIN            0.2f   // Fattore di scala per l'angolo in modalità SPIN (gradi per pixel)
#define VERTICAL_MARGIN_TOP  (IMAGE_HEIGHT / 5)
#define VERTICAL_MARGIN_BOT  ((4 * IMAGE_HEIGHT) / 5)

// --- STRUTTURE DATI ---
typedef struct {
    int x;
    int y;
    float u;
    float v;
} flow_vector_t;

#define MAX_FLOW_VECTORS 1000
static flow_vector_t flow_vectors[MAX_FLOW_VECTORS];
static int flow_count = 0;

static uint8_t prev_gray[IMAGE_HEIGHT * IMAGE_WIDTH];
static int first_frame = 1;

// Il messaggio ora trasporta anche un angolo (in gradi)
typedef struct {
    uint8_t cmd;         // 1: SAFE (go forward), 4: SPIN (ambiente troppo ostacolato)
    int16_t angle_deg;   // Angolo di sterzata in gradi (positivo = sinistra, negativo = destra)
    bool updated;
} flow_msg_t;

static flow_msg_t flow_move = {0, 0, false};
static pthread_mutex_t flow_mutex = PTHREAD_MUTEX_INITIALIZER;

// --- FUNZIONI DI UTILITÀ ---

/* 
 * convert_to_gray: converte un’immagine YUV422 in scala di grigi estraendo il canale Y.
 */
static void convert_to_gray(uint8_t *src, uint8_t *gray, int w, int h) {
    int num_pixels = w * h;
    int src_index = 0, gray_index = 0;
    while (gray_index < num_pixels) {
        gray[gray_index++] = src[src_index + 1]; // Componente Y del primo pixel
        if (gray_index < num_pixels)
            gray[gray_index++] = src[src_index + 3]; // Componente Y del secondo pixel
        src_index += 4;
    }
}

/*
 * compute_gradients: calcola i gradienti orizzontale (Ix) e verticale (Iy) usando differenze centrali.
 */
static void compute_gradients(uint8_t *gray, int w, int h, float *Ix, float *Iy) {
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            int idx = y * w + x;
            Ix[idx] = ((float)gray[y * w + (x + 1)] - (float)gray[y * w + (x - 1)]) * 0.5f;
            Iy[idx] = ((float)gray[(y + 1) * w + x] - (float)gray[(y - 1) * w + x]) * 0.5f;
        }
    }
}

/*
 * compute_optical_flow: calcola l’optic flow usando una griglia regolare (con STEP pari a POINT_SPACING)
 * e il metodo semplificato di Lucas-Kanade.
 */
static void compute_optical_flow(uint8_t *prev, uint8_t *curr, int w, int h) {
    float Ix[IMAGE_HEIGHT * IMAGE_WIDTH] = {0};
    float Iy[IMAGE_HEIGHT * IMAGE_WIDTH] = {0};
    memset(Ix, 0, sizeof(Ix));
    memset(Iy, 0, sizeof(Iy));

    compute_gradients(prev, w, h, Ix, Iy);
    
    flow_count = 0;
    int margin = WINDOW_SIZE / 2;
    for (int y = margin; y < h - margin; y += POINT_SPACING) {
        for (int x = margin; x < w - margin; x += POINT_SPACING) {
            float sum_Ix2 = 0.0f, sum_Iy2 = 0.0f, sum_IxIy = 0.0f;
            float sum_IxIt = 0.0f, sum_IyIt = 0.0f;
            for (int j = -margin; j <= margin; j++) {
                for (int i = -margin; i <= margin; i++) {
                    int idx = (y + j) * w + (x + i);
                    float ix = Ix[idx];
                    float iy = Iy[idx];
                    float it = (float)curr[idx] - (float)prev[idx];
                    sum_Ix2 += ix * ix;
                    sum_Iy2 += iy * iy;
                    sum_IxIy += ix * iy;
                    sum_IxIt += ix * it;
                    sum_IyIt += iy * it;
                }
            }
            float det = sum_Ix2 * sum_Iy2 - sum_IxIy * sum_IxIy;
            float u = 0.0f, v = 0.0f;
            if (det != 0.0f) {
                u = (-sum_Iy2 * sum_IxIt + sum_IxIy * sum_IyIt) / det;
                v = (-sum_IxIy * sum_IxIt + sum_Ix2 * sum_IyIt) / det;
            }
            if (flow_count < MAX_FLOW_VECTORS) {
                flow_vectors[flow_count].x = x;
                flow_vectors[flow_count].y = y;
                flow_vectors[flow_count].u = u;
                flow_vectors[flow_count].v = v;
                flow_count++;
            }
        }
    }
}

/*
 * analyze_obstacles: utilizza una finestra scorrevole orizzontale nella fascia centrale 
 * dell’immagine per determinare l’area con minor “rischio” (ovvero, con flusso medio minore).
 * Viene calcolato l’offset tra il centro della finestra più sicura e il centro dell’immagine.
 */
static void analyze_obstacles(int w, int h, int *best_center, float *min_avg) {
    *min_avg = FLT_MAX;
    *best_center = w / 2; // default: centro

    for (int x_start = 0; x_start <= w - SLIDING_WINDOW_WIDTH; x_start += SLIDING_WINDOW_STEP) {
        int count = 0;
        float sum_sq = 0.0f;
        for (int i = 0; i < flow_count; i++) {
            int x = flow_vectors[i].x;
            int y = flow_vectors[i].y;
            if (x >= x_start && x < x_start + SLIDING_WINDOW_WIDTH &&
                y >= VERTICAL_MARGIN_TOP && y < VERTICAL_MARGIN_BOT) {
                float sq_mag = flow_vectors[i].u * flow_vectors[i].u + flow_vectors[i].v * flow_vectors[i].v;
                sum_sq += sq_mag;
                count++;
            }
        }
        float avg_sq = (count > 0) ? (sum_sq / count) : OBSTACLE_THRESHOLD_SQ * 2; // Se nessun vettore, penalizza
        if (avg_sq < *min_avg) {
            *min_avg = avg_sq;
            *best_center = x_start + SLIDING_WINDOW_WIDTH / 2;
        }
    }
}

/*
 * optical_flow_obs_func: funzione principale chiamata dalla camera.
 * Calcola l’optic flow, analizza le finestre per il rilevamento ostacoli e calcola
 * un angolo di sterzata proporzionale allo spazio libero.
 * Il comando inviato via ABI include:
 *   - cmd = 1 (SAFE) se il rischio è accettabile,
 *   - cmd = 4 (SPIN) se l’ambiente è troppo ostacolato.
 * L’angolo di sterzata (angle_deg) è calcolato in base alla differenza fra il centro della
 * finestra più “sicura” e il centro dell’immagine.
 */
static struct image_t *optical_flow_obs_func(struct image_t *img, uint8_t camera_id __attribute__((unused))) {
    uint8_t curr_gray[IMAGE_HEIGHT * IMAGE_WIDTH];
    convert_to_gray(img->buf, curr_gray, IMAGE_WIDTH, IMAGE_HEIGHT);
    
    if (first_frame) {
        memcpy(prev_gray, curr_gray, IMAGE_WIDTH * IMAGE_HEIGHT);
        first_frame = 0;
        return img;
    }
    
    compute_optical_flow(prev_gray, curr_gray, IMAGE_WIDTH, IMAGE_HEIGHT);
    
    int best_window_center = IMAGE_WIDTH / 2;
    float min_avg = 0.0f;
    analyze_obstacles(IMAGE_WIDTH, IMAGE_HEIGHT, &best_window_center, &min_avg);
    
    // Calcola offset: se free-space è a sinistra l’offset risulterà negativo
    int offset = best_window_center - (IMAGE_WIDTH / 2);
    int16_t angle = 0;
    uint8_t cmd = 1; // SAFE per default

    if (min_avg < OBSTACLE_THRESHOLD_SQ) {
        // Modalità SAFE: avanza con correzione proporzionale
        angle = -(int16_t)(offset * STEERING_GAIN);
        cmd = 1;
    } else {
        // Ambiente troppo ostacolato: esegue uno spin per cercare spazio libero
        angle = -(int16_t)(offset * SPIN_GAIN);
        cmd = 4;
    }
    
    // Aggiorna il messaggio globale in modo thread-safe
    pthread_mutex_lock(&flow_mutex);
    flow_move.cmd = cmd;
    flow_move.angle_deg = angle;
    flow_move.updated = true;
    pthread_mutex_unlock(&flow_mutex);
    
    // Aggiorna il frame precedente per la prossima iterazione.
    memcpy(prev_gray, curr_gray, IMAGE_WIDTH * IMAGE_HEIGHT);
    
    return img;
}

/*
 * optical_flow_obs_periodic: invia periodicamente il comando calcolato tramite ABI.
 */
void optical_flow_obs_periodic(void) {
    flow_msg_t local_move;
    pthread_mutex_lock(&flow_mutex);
    local_move = flow_move;
    if (flow_move.updated) {
        flow_move.updated = false;
    }
    pthread_mutex_unlock(&flow_mutex);
    
    if (local_move.updated) {
        // Il campo "extra" ora trasporta l'angolo di sterzata (in gradi)
        AbiSendMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, local_move.cmd, local_move.angle_deg);
    }
}

/*
 * optical_flow_obs_init: inizializza il modulo registrando la callback della camera.
 */
void optical_flow_obs_init(void) {
    first_frame = 1;
    cv_add_to_device(&FLOOR_DET_CAMERA, optical_flow_obs_func, 0, 0);
    printf("Optical Flow Obstacle Detection Initialized\n");
}
