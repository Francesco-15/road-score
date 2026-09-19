/**
  ******************************************************************************
  * @file           : app_config.h
  * @brief          : Parametri e configurazioni centralizzate del progetto
  *                   Rilevamento Manto Stradale (NUCLEO-F401RE + MPU-6050)
  ******************************************************************************
  */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================== */
/* 1. CONFIGURAZIONE CAMPIONAMENTO E BUFFER                                   */
/* ========================================================================== */
#define APP_SAMPLE_HZ              100U
#define APP_SAMPLE_PERIOD_MS       10U
#define APP_BUFFER_SIZE_SAMPLES    400U  /* 4 secondi di storico ad alta risoluzione */
#define APP_MACRO_HISTORY_SECONDS  30U   /* 30 secondi di macro-contesto (1 record / sec) */

/* ========================================================================== */
/* 2. PARAMETRI HARDWARE MPU-6050                                             */
/* ========================================================================== */
#define MPU_I2C_ADDR               0x68U
#define MPU_ACCEL_FS_G             4.0f
#define MPU_GYRO_FS_DPS            500.0f
#define MPU_ACCEL_LSB_PER_G        8192.0f
#define MPU_GYRO_LSB_PER_DPS       65.5f

#define CALIBRATION_SAMPLES        200U  /* Campioni per calibrazione iniziale a fermo */

/* ========================================================================== */
/* 3. SOGLIE E PARAMETRI ALGORITMO ADATTIVO (SOGLIA E RUMORE)                 */
/* ========================================================================== */
/*
 * In stato IDLE viene stimato dinamicamente il livello di rumore di fondo
 * (sigma_noise_mg). La soglia per innescare un evento (CAPTURING) su a_z è:
 * La soglia segue una curva concava, simile a un logaritmo:
 *     soglia_inizio_az = MIN + SPAN * sigma_noise / (sigma_noise + KNEE)
 * Cresce rapidamente all'inizio, poi si appiattisce: il rumore di una strada
 * molto dissestata non può quindi portarla a 500-600 mg.
 */
#define APP_TH_MIN_START_MG        100L    /* Soglia verticale minima di sicurezza in mg */
#define APP_TH_ADAPT_SPAN_MG       240L    /* Incremento massimo oltre alla soglia minima */
#define APP_TH_ADAPT_KNEE_MG       75L     /* Punto di flesso della curva adattiva */
#define APP_TH_END_RATIO           0.50f   /* Rapporto soglia di fine rispetto a inizio */

/*
 * Soglie del giroscopio per la classificazione multiasse (destra/sinistra e beccheggio)
 */
#define APP_TH_ROLL_GX_DPS         8L     /* Soglia rollio per distinguere buca Sx vs Dx vs centro */
#define APP_TH_PITCH_GY_DPS        8L     /* Soglia beccheggio per confermare urto ruota ant/post */
#define APP_POTHOLE_NEG_DOMINANCE_PERCENT 110L /* Picco verso il basso richiesto per una buca */ //ATTENZIONE

/*
 * Moltiplicatore per il calcolo della Severità Proporzionale alla soglia:
 *     Severity = min(100, (peak_to_peak - threshold) * 100 / (threshold * APP_SEV_SCALE_RATIO))
 * Se RATIO = 3.0, un urto che supera la soglia del 300% (4x la soglia) vale 100/100 di severità.
 */
#define APP_SEV_SCALE_RATIO        3.0f

/*
 * Fattore di smorzamento EMA a 100 Hz: mantengono le stesse costanti di tempo
 * della versione a 200 Hz (circa 0.25 s per il rumore e 1 s per la gravità).
 */
#define APP_NOISE_EMA_ALPHA        0.04f
#define APP_GRAVITY_LPF_ALPHA      0.01f

/* ========================================================================== */
/* 4. TEMPI DELLA MACCHINA A STATI E CLASSIFICAZIONE EVENTI                   */
/* ========================================================================== */
#define EVENT_MIN_DURATION_MS      60U     /* Durata minima evento (esclude glitch) */
#define EVENT_MAX_DURATION_MS      1500U   /* Durata massima assoluta evento esteso */
#define EVENT_REFRACTORY_MS        1500U   /* Raggruppa salita/discesa e le due coppie di ruote in un solo evento */

/*
 * Limite di tempo per riconoscere una transizione brusca a fondo rumoroso (es. ingresso sampietrini).
 * Se dopo 300 ms il segnale continua a oscillare senza calma, non è una buca ma cambio manto.
 */
#define APP_TRANSITION_CHECK_MS    300U
#define APP_TRANSITION_MIN_ZC      10U     /* Zero crossings minimi in 300 ms per cambio manto */

/* ========================================================================== */
/* 5. TELEMETRIA E REPORTING                                                  */
/* ========================================================================== */
#define STATUS_PERIOD_MS           1000U   /* Frequenza invio report stato medio (1 Hz) */

/* ========================================================================== */
/* 6. MODALITÀ DI MONTAGGIO (MOUNT MODE)                                      */
/* ========================================================================== */
#define MOUNT_MODE_TUNNEL          0U
#define MOUNT_MODE_ROOF            1U
#define APP_MOUNT_MODE             MOUNT_MODE_TUNNEL

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
