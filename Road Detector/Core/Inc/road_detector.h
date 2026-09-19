/**
  ******************************************************************************
  * @file           : road_detector.h
  * @brief          : Header del modulo di elaborazione e rilevamento asperità
  *                   stradali con soglia adattiva, buffer 400 campioni, giroscopio
  *                   e macro-storico a 30s per i 3 scenari applicativi avanzati.
  ******************************************************************************
  */

#ifndef ROAD_DETECTOR_H
#define ROAD_DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

/* ========================================================================== */
/* 1. ENUMERAZIONI                                                            */
/* ========================================================================== */

/**
  * @brief Stati della macchina a stati del rilevatore di eventi
  */
typedef enum {
    DETECTOR_STATE_IDLE = 0,        /**< Monitoraggio continuo e stima rumore di fondo */
    DETECTOR_STATE_CAPTURING,       /**< Evento innescato, raccolta campioni sopra soglia */
    DETECTOR_STATE_CLASSIFY,        /**< Evento concluso, calcolo feature e classificazione */
    DETECTOR_STATE_REFRACTORY       /**< Periodo refrattario (accorpa urto ruota posteriore) */
} detector_state_t;

/**
  * @brief Classificazione fine del tipo di evento stradale (multiasse a 6 gradi di libertà)
  */
typedef enum {
    EVENT_ANOMALY          = 0,     /**< Anomalia generica o impatto misto */
    EVENT_POTHOLE_LEFT     = 1,     /**< Buca ruota sinistra (picco giù + rollio Sx) */
    EVENT_POTHOLE_RIGHT    = 2,     /**< Buca ruota destra (picco giù + rollio Dx) */
    EVENT_POTHOLE_CENTER   = 3,     /**< Buca simmetrica / gradino in discesa (rollio nullo) */
    EVENT_BUMP             = 4,     /**< Dosso stradale simmetrico in salita */
    EVENT_DIP              = 5,     /**< Cunetta prolungata morbida in discesa e salita */
    EVENT_ROUGH            = 6,     /**< Tratto a marcato dissesto diffuso prolungato */
    EVENT_LATERAL_TILT     = 7,     /**< Avvallamento laterale prolungato (dominanza giroscopica) */
    EVENT_ROUGH_TRANSITION = 8,     /**< Cambio manto stradale brusco (es. ingresso sampietrini) */
    EVENT_UNKNOWN          = 255    /**< Evento non classificabile con certezza */
} event_class_t;

/* ========================================================================== */
/* 2. STRUTTURE DATI                                                          */
/* ========================================================================== */

/**
 * @brief Singolo campione memorizzato nel buffer circolare ad alta risoluzione (100 Hz)
  */
typedef struct {
    uint32_t timestamp_ms;          /**< Timestamp in millisecondi */
    int16_t  ax_mg;                 /**< Accelerazione X grezza in milli-g */
    int16_t  ay_mg;                 /**< Accelerazione Y grezza in milli-g */
    int16_t  az_mg;                 /**< Accelerazione Z grezza in milli-g */
    int32_t  az_filtered_mg;        /**< Accelerazione Z dinamica, depurata da gravità/salita/discesa */
    int32_t  jerk_mg_s;             /**< Jerk (derivata di az_filtered nel tempo) in mg/s */
    int16_t  gx_dps;                /**< Giroscopio X (Rollio) in gradi/s */
    int16_t  gy_dps;                /**< Giroscopio Y (Beccheggio) in gradi/s */
    int16_t  gz_dps;                /**< Giroscopio Z (Imbardata/Curva) in gradi/s */
} imu_sample_t;

/**
  * @brief Elemento della coda di macro-contesto storico (Sintesi di 1 secondo)
  */
typedef struct {
    uint32_t timestamp_sec;         /**< Timestamp in secondi dall'avvio */
    int16_t  mean_sigma_noise_mg;   /**< Rumore medio di fondo (rugosità) del secondo */
    int16_t  max_az_dyn_mg;         /**< Picco massimo di accelerazione verticale del secondo */
    int16_t  mean_roll_gx_dps;      /**< Rollio medio del secondo */
    int16_t  mean_pitch_gy_dps;     /**< Beccheggio medio del secondo */
} macro_context_t;

/**
  * @brief Struttura che descrive un evento di asperità stradale classificato
  */
typedef struct {
    uint32_t      timestamp_ms;          /**< Timestamp di innesco dell'evento */
    event_class_t event_class;           /**< Classe fine identificata (POTHOLE_LEFT, BUMP, ecc.) */
    uint8_t       severity;              /**< Severità proporzionale alla soglia (0 - 100) */
    uint8_t       confidence;            /**< Confidenza della classificazione (0 - 100) */
    int32_t       peak_pos_mg;           /**< Massimo picco positivo verticale durante l'evento */
    int32_t       peak_neg_mg;           /**< Massimo picco negativo verticale durante l'evento */
    int32_t       peak_z_mg;             /**< Picco principale assoluto (con segno originale) */
    int16_t       peak_gx_dps;           /**< Picco di Rollio (Giroscopio X, distingue Sx vs Dx) */
    int16_t       peak_gy_dps;           /**< Picco di Beccheggio (Giroscopio Y, ruota ant/post) */
    int16_t       peak_gz_dps;           /**< Picco di Imbardata (Giroscopio Z, check curva) */
    uint16_t      duration_ms;           /**< Durata totale della finestra evento in ms */
    int32_t       jerk_max_mg_s;         /**< Jerk massimo rilevato durante l'evento */
    uint32_t      energy_score;          /**< Energia accumulata verticale e angolare */
    int32_t       threshold_mg;          /**< Soglia dinamica attiva al momento dell'innesco */
    uint32_t      sequence_num;          /**< Numero di sequenza incrementale di evento */
} road_event_t;

/**
  * @brief Struttura riassuntiva periodica (1 Hz) dello stato medio e trend del manto stradale
  */
typedef struct {
    uint32_t         uptime_ms;            /**< Tempo trascorso dall'avvio del sistema */
    int32_t          sigma_noise_mg;       /**< Rumore di fondo istantaneo dell'ultimo secondo */
    int32_t          sigma_noise_10s_mg;   /**< Media mobile del rumore degli ultimi 10 secondi (Scenario 1) */
    int32_t          sigma_noise_30s_mg;   /**< Media mobile del rumore degli ultimi 30 secondi (Scenario 1) */
    uint8_t          roughness_score;      /**< Indice di qualità istantaneo da 0 (pessimo) a 100 (liscio) */
    uint32_t         total_events_count;   /**< Contatore totale di eventi da boot */
    detector_state_t current_state;        /**< Stato attuale della macchina a stati */
    int32_t          current_threshold_mg; /**< Soglia dinamica di innesco in corso */
} road_status_t;

/* ========================================================================== */
/* 3. PROTOTIPI DI FUNZIONE                                                   */
/* ========================================================================== */

/**
  * @brief Inizializza il modulo road_detector, azzerando buffer circolare,
  *        macro-storico, stati e filtri.
  */
void RoadDetector_Init(void);

/**
 * @brief Aggiorna l'algoritmo con un nuovo campione IMU a frequenza costante (100 Hz).
  *        Questa funzione è interamente indipendente dall'hardware (nessuna chiamata HAL).
  */
void RoadDetector_Update(uint32_t now_ms, int16_t ax_mg, int16_t ay_mg, int16_t az_mg,
                         int16_t gx_dps, int16_t gy_dps, int16_t gz_dps);

/**
  * @brief Restituisce e consuma un evento se presente in coda di uscita.
  */
bool RoadDetector_GetPendingEvent(road_event_t *out_event);

/**
  * @brief Controlla e restituisce il report di stato a 1 Hz con le medie a 10s e 30s.
  */
bool RoadDetector_Get1HzStatus(uint32_t now_ms, road_status_t *out_status);

/**
  * @brief Copia il macro-storico degli ultimi N secondi validi.
  * @param out_history Array di destinazione (deve poter contenere 30 elementi)
  * @param out_count   Numero di secondi validi copiati (0 - 30)
  * @return true se il macro-storico contiene almeno 1 secondo valido
  */
bool RoadDetector_GetMacroHistory(macro_context_t *out_history, uint8_t *out_count);

/**
  * @brief Formatta una riga seriale per lo scarico (Dump) di un record del macro-storico (Scenario 3).
  *        Consente di trasmettere l'andamento degli ultimi 30 secondi via UART/Bluetooth su richiesta.
  *
  * @param index        Indice del record nello storico (0 = più recente, count-1 = più vecchio)
  * @param out_buffer   Stringa di destinazione formattata ($MACRO_DUMP,sec_ago,noise,max_pk,gx,gy\r\n)
  * @param max_len      Dimensione del buffer
  * @return true se l'indice era valido e la riga è stata formattata, false altrimenti
  */
bool RoadDetector_FormatMacroDumpItem(uint8_t index, char *out_buffer, uint16_t max_len);

/**
  * @brief Restituisce il numero di secondi attulmente disponibili nel macro-storico (da 0 a 30).
  */
uint8_t RoadDetector_GetMacroCount(void);

/**
  * @brief Funzioni accessorie di telemetria e debug per la macchina a stati
  */
detector_state_t RoadDetector_GetState(void);
int32_t RoadDetector_GetCurrentThreshold(void);
int32_t RoadDetector_GetSigmaNoise(void);
uint32_t RoadDetector_GetTotalEventsCount(void);

#ifdef __cplusplus
}
#endif

#endif /* ROAD_DETECTOR_H */
