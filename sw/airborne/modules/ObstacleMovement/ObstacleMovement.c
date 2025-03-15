/* obstacle_movement.c
 *
 * Questo modulo riceve i comandi di evitamento ostacoli via ABI e aggiorna la guida
 * del drone. Il messaggio contiene:
 *   - cmd: 1 per modalità SAFE (avanzamento con correzione) oppure 4 per SPIN (ambiente troppo ostacolato)
 *   - angle_deg: angolo di sterzata in gradi (positivo = curva a sinistra, negativo = curva a destra)
 */

#include "ObstacleMovement.h"
#include <stdio.h>
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"

// Parametri di guida
float oag_max_speed = 1.0;
float oag_default_heading_rate_deg = 0.0; // Impostato dinamicamente dal comando

// Nuova struttura per il comando ricevuto
typedef struct {
    uint8_t cmd;
    int16_t angle_deg; // Angolo in gradi ricevuto via ABI
} obstacle_cmd_t;

static obstacle_cmd_t current_cmd = {0, 0};
static volatile bool cmd_updated = false;

#ifndef TCT_FLOOR_DETECTION_ID
#error Define TCT_FLOOR_DETECTION_ID (e.g., COLOR_OBJECT_DETECTION1_ID) in your airframe configuration.
#endif

static abi_event obstacle_movement_ev;

/*
 * obstacle_movement_listener: callback per i messaggi ABI.
 * Mappa il messaggio ricevuto aggiornando il comando globale.
 */
static void obstacle_movement_listener(uint8_t __attribute__((unused)) sender_id,
                                         int8_t setting,
                                         int16_t extra) {
    // In questa implementazione:
    //   - setting: codice comando (1: SAFE, 4: SPIN)
    //   - extra: angolo di sterzata in gradi
    current_cmd.cmd = (uint8_t) setting;
    current_cmd.angle_deg = extra;
    cmd_updated = true;
}

/*
 * obstacle_move_init: inizializza il modulo associando la callback ABI.
 */
void obstacle_move_init(void) {
    printf("Obstacle Movement Listener Setup\n");
    AbiBindMsgTCT_AP_Direct(TCT_FLOOR_DETECTION_ID, &obstacle_movement_ev, obstacle_movement_listener);
    printf("Obstacle Movement Listener Ready\n");
}

/*
 * obstacle_move_periodic: aggiorna periodicamente la guida del drone in base al comando ricevuto.
 *
 * Modalità SAFE:
 *   - Velocità in avanti impostata a oag_max_speed.
 *   - Tasso di sterzata proporzionale all'angolo ricevuto (convertito in radianti).
 *
 * Modalità SPIN:
 *   - Velocità in avanti a 0.
 *   - Il drone ruota in base all'angolo ricevuto.
 */
void obstacle_move_periodic(void) {
    if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
        guidance_h_set_body_vel(0, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
        return;
    }
    
    // Se non c'è un nuovo comando, mantieni il comando corrente.
    if (!cmd_updated) {
        // Puoi decidere di mantenere la precedente direzione o resettare i comandi.
        // In questo esempio, non vengono applicate modifiche se non c'è aggiornamento.
        return;
    }
    cmd_updated = false;
    
    // Imposta la guida in base al comando ricevuto
    if (current_cmd.cmd == 1) { 
        // Modalità SAFE: avanza e corregge la traiettoria in base all'angolo.
        guidance_h_set_body_vel(oag_max_speed, 0);
        guidance_h_set_heading_rate(RadOfDeg(current_cmd.angle_deg));
    } else if (current_cmd.cmd == 4) {
        // Modalità SPIN: il drone ruota per cercare spazio libero.
        guidance_h_set_body_vel(0, 0);
        guidance_h_set_heading_rate(RadOfDeg(current_cmd.angle_deg));
    } else {
        // Altri comandi (se presenti) possono essere gestiti qui.
        guidance_h_set_body_vel(0, 0);
        guidance_h_set_heading_rate(RadOfDeg(0));
    }
}
