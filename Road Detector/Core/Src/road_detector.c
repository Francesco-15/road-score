/**
  ******************************************************************************
  * @file           : road_detector.c
  * @brief          : Implementazione dell'algoritmo di rilevamento asperità
  *                   con i 3 scenari applicativi del macro-storico a 30 secondi:
  *                   1. Trend a lungo termine (10s e 30s) nel pacchetto status
  *                   2. Macro-Context Awareness nel classificatore buche/dossi
  *                   3. Scarico (Dump) su richiesta via seriale/Bluetooth
  ******************************************************************************
  */

#include "road_detector.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ========================================================================== */
/* 1. VARIABILI E STRUTTURE INTERNE AL MODULO                                 */
/* ========================================================================== */

/* Buffer circolare ad alta risoluzione (ultimi 400 campioni a 100 Hz = 4 s) */
static imu_sample_t s_buffer[APP_BUFFER_SIZE_SAMPLES];
static uint16_t     s_head = 0;
static bool         s_buffer_primed = false;

/* Macro-storico a lungo termine (ultimi 30 secondi = 1 record/s) */
static macro_context_t s_macro_history[APP_MACRO_HISTORY_SECONDS];
static uint8_t         s_macro_head = 0;
static uint8_t         s_macro_count = 0;

/* Accumulatori per la media del secondo corrente */
static uint32_t        s_sec_sample_count = 0;
static int32_t         s_sec_sigma_sum = 0;
static int16_t         s_sec_max_az_dyn = 0;
static int32_t         s_sec_gx_sum = 0;
static int32_t         s_sec_gy_sum = 0;

/* Stato e parametri dell'algoritmo */
static detector_state_t s_state = DETECTOR_STATE_IDLE;
static float            s_az_gravity_est = 0.0f;
static float            s_sigma_noise = 25.0f;
static int32_t          s_current_threshold = APP_TH_MIN_START_MG;

/* Variabili di tracciamento per l'evento in corso (CAPTURING) */
static uint32_t         s_event_start_time = 0;
static int32_t          s_peak_pos = 0;
static int32_t          s_peak_neg = 0;
static int32_t          s_peak_main = 0;
static int16_t          s_peak_gx = 0;
static int16_t          s_peak_gy = 0;
static int16_t          s_peak_gz = 0;
static int32_t          s_jerk_max = 0;
static uint64_t         s_energy_sum = 0;
static uint32_t         s_quiet_count = 0;
static uint32_t         s_zero_crossings = 0;
static int32_t          s_last_sign_az = 0;
static uint32_t         s_refractory_start_time = 0;

/* Output asincrono: evento pendente in attesa di trasmissione */
static road_event_t     s_pending_event;
static bool             s_pending_event_valid = false;
static uint32_t         s_sequence_num = 0;
static uint32_t         s_total_events = 0;

/* Output periodico: timestamp ultimo report 1 Hz */
static uint32_t         s_last_status_ms = 0;
static int32_t          s_prev_az_dyn = 0;

/* ========================================================================== */
/* 2. FUNZIONI HELPER INTERNE                                                 */
/* ========================================================================== */

static int32_t abs_int32(int32_t val)
{
    return (val >= 0) ? val : -val;
}

static uint32_t min_uint32(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

static uint32_t max_uint32(uint32_t a, uint32_t b)
{
    return (a > b) ? a : b;
}

/* Curva concava/saturante: l'equivalente discreto di una soglia logaritmica.
 * Evita che il rumore continuo di pavé o sterrato renda cieco il rilevatore. */
static int32_t calculate_adaptive_threshold(float sigma_noise)
{
    int32_t noise_mg = (sigma_noise > 0.0f) ? (int32_t)sigma_noise : 0L;
    int32_t denominator = noise_mg + APP_TH_ADAPT_KNEE_MG;
    int32_t adaptive_part = (APP_TH_ADAPT_SPAN_MG * noise_mg) / denominator;

    return APP_TH_MIN_START_MG + adaptive_part;
}

/* ========================================================================== */
/* 3. IMPLEMENTAZIONE DELLE API DEL MODULO                                    */
/* ========================================================================== */

void RoadDetector_Init(void)
{
    memset(s_buffer, 0, sizeof(s_buffer));
    s_head = 0;
    s_buffer_primed = false;

    memset(s_macro_history, 0, sizeof(s_macro_history));
    s_macro_head = 0;
    s_macro_count = 0;
    s_sec_sample_count = 0;
    s_sec_sigma_sum = 0;
    s_sec_max_az_dyn = 0;
    s_sec_gx_sum = 0;
    s_sec_gy_sum = 0;

    s_state = DETECTOR_STATE_IDLE;
    /* L'accelerometro è azzerato in calibrazione a veicolo fermo. */
    s_az_gravity_est = 0.0f;
    s_sigma_noise = 25.0f;
    s_current_threshold = APP_TH_MIN_START_MG;

    s_pending_event_valid = false;
    s_sequence_num = 0;
    s_total_events = 0;
    s_last_status_ms = 0;
    s_prev_az_dyn = 0;
}

void RoadDetector_Update(uint32_t now_ms, int16_t ax_mg, int16_t ay_mg, int16_t az_mg,
                         int16_t gx_dps, int16_t gy_dps, int16_t gz_dps)
{
    /* 1. Filtro Passa-Basso per inseguire la gravità e la pendenza stradale in salita/discesa */
    s_az_gravity_est = s_az_gravity_est + APP_GRAVITY_LPF_ALPHA * ((float)az_mg - s_az_gravity_est);

    /* 2. Calcolo accelerazione verticale dinamica depurata (Filtro Passa-Alto implicito) */
    int32_t az_dyn = (int32_t)az_mg - (int32_t)s_az_gravity_est;

    /* 3. Calcolo del Jerk in mg/s */
    int32_t jerk = ((az_dyn - s_prev_az_dyn) * 1000L) /
                   (int32_t)APP_SAMPLE_PERIOD_MS;
    s_prev_az_dyn = az_dyn;

    /* 4. Salvataggio nel buffer circolare ad alta risoluzione */
    s_head = (s_head + 1U) % APP_BUFFER_SIZE_SAMPLES;
    if (s_head == 0U)
    {
        s_buffer_primed = true;
    }
    imu_sample_t *s = &s_buffer[s_head];
    s->timestamp_ms   = now_ms;
    s->ax_mg          = ax_mg;
    s->ay_mg          = ay_mg;
    s->az_mg          = az_mg;
    s->az_filtered_mg = az_dyn;
    s->jerk_mg_s      = jerk;
    s->gx_dps         = gx_dps;
    s->gy_dps         = gy_dps;
    s->gz_dps         = gz_dps;

    /* 4b. Accumulo per la sintesi del secondo corrente (per alimentare s_macro_history) */
    s_sec_sample_count++;
    s_sec_sigma_sum += (int32_t)s_sigma_noise;
    if (abs_int32(az_dyn) > abs_int32(s_sec_max_az_dyn))
    {
        s_sec_max_az_dyn = (int16_t)az_dyn;
    }
    s_sec_gx_sum += (int32_t)gx_dps;
    s_sec_gy_sum += (int32_t)gy_dps;

    /* 5. Macchina a stati principale */
    switch (s_state)
    {
        case DETECTOR_STATE_IDLE:
        {
            /* In IDLE aggiorniamo costantemente la stima del rumore di fondo (EMA) */
            int32_t abs_dyn = abs_int32(az_dyn);
            s_sigma_noise = (1.0f - APP_NOISE_EMA_ALPHA) * s_sigma_noise + APP_NOISE_EMA_ALPHA * (float)abs_dyn;

            s_current_threshold = calculate_adaptive_threshold(s_sigma_noise);

            /* Controllo innesco su a_z oppure su forte impulso giroscopico */
            if ((abs_dyn > s_current_threshold) ||
                ((abs_int32(gx_dps) > (APP_TH_ROLL_GX_DPS * 2L)) && (abs_dyn > (s_current_threshold / 2L))))
            {
                s_state = DETECTOR_STATE_CAPTURING;
                s_event_start_time = now_ms;
                s_peak_pos = az_dyn;
                s_peak_neg = az_dyn;
                s_peak_main = az_dyn;
                s_peak_gx = gx_dps;
                s_peak_gy = gy_dps;
                s_peak_gz = gz_dps;
                s_jerk_max = abs_int32(jerk);
                s_energy_sum = (uint64_t)(az_dyn * az_dyn) * APP_SAMPLE_PERIOD_MS;
                s_quiet_count = 0;
                s_zero_crossings = 0;
                s_last_sign_az = (az_dyn >= 0) ? 1 : -1;
            }
            break;
        }

        case DETECTOR_STATE_CAPTURING:
        {
            if (az_dyn > s_peak_pos) { s_peak_pos = az_dyn; }
            if (az_dyn < s_peak_neg) { s_peak_neg = az_dyn; }
            if (abs_int32(az_dyn) > abs_int32(s_peak_main)) { s_peak_main = az_dyn; }
            if (abs_int32(gx_dps) > abs_int32(s_peak_gx))   { s_peak_gx = gx_dps; }
            if (abs_int32(gy_dps) > abs_int32(s_peak_gy))   { s_peak_gy = gy_dps; }
            if (abs_int32(gz_dps) > abs_int32(s_peak_gz))   { s_peak_gz = gz_dps; }
            if (abs_int32(jerk) > s_jerk_max)               { s_jerk_max = abs_int32(jerk); }
            s_energy_sum += (uint64_t)(az_dyn * az_dyn) * APP_SAMPLE_PERIOD_MS;

            int32_t curr_sign = (az_dyn >= 0) ? 1 : -1;
            if (curr_sign != s_last_sign_az)
            {
                s_zero_crossings++;
                s_last_sign_az = curr_sign;
            }

            int32_t th_end = (int32_t)(s_current_threshold * APP_TH_END_RATIO);
            if (abs_int32(az_dyn) <= th_end)
            {
                s_quiet_count++;
            }
            else
            {
                s_quiet_count = 0;
            }

            uint32_t quiet_time_ms = s_quiet_count * APP_SAMPLE_PERIOD_MS;
            uint32_t duration_ms = now_ms - s_event_start_time;

            /* Check Transizione di Manto (Ingresso su Sampietrini/Pavé) */
            if ((duration_ms >= APP_TRANSITION_CHECK_MS) && (s_zero_crossings >= APP_TRANSITION_MIN_ZC) && (quiet_time_ms < 30U))
            {
                s_sigma_noise = 100.0f;
                s_current_threshold = calculate_adaptive_threshold(s_sigma_noise);

                s_pending_event.timestamp_ms         = s_event_start_time;
                s_pending_event.event_class          = EVENT_ROUGH_TRANSITION;
                s_pending_event.severity             = 20;
                s_pending_event.confidence           = 90;
                s_pending_event.peak_pos_mg          = s_peak_pos;
                s_pending_event.peak_neg_mg          = s_peak_neg;
                s_pending_event.peak_z_mg            = s_peak_main;
                s_pending_event.peak_gx_dps          = s_peak_gx;
                s_pending_event.peak_gy_dps          = s_peak_gy;
                s_pending_event.peak_gz_dps          = s_peak_gz;
                s_pending_event.duration_ms          = (uint16_t)duration_ms;
                s_pending_event.jerk_max_mg_s        = s_jerk_max;
                s_pending_event.energy_score         = (uint32_t)(s_energy_sum / 1000ULL);
                s_pending_event.threshold_mg         = s_current_threshold;
                s_pending_event.sequence_num         = ++s_sequence_num;

                s_pending_event_valid = true;
                s_total_events++;

                s_state = DETECTOR_STATE_REFRACTORY;
                s_refractory_start_time = now_ms;
                break;
            }

            if ((quiet_time_ms >= 50U) || (duration_ms >= EVENT_MAX_DURATION_MS))
            {
                s_state = DETECTOR_STATE_CLASSIFY;
            }
            break;
        }

        case DETECTOR_STATE_CLASSIFY:
        {
            uint32_t duration_ms = now_ms - s_event_start_time;
            int32_t peak_to_peak = s_peak_pos - s_peak_neg;

            if ((duration_ms >= EVENT_MIN_DURATION_MS) &&
                ((peak_to_peak >= s_current_threshold) || (abs_int32(s_peak_gx) >= APP_TH_ROLL_GX_DPS)))
            {
                /* ============================================================== */
                /* SCENARIO 2: MACRO-CONTEXT AWARENESS                            */
                /* Prima di decidere, consultiamo s_macro_history per verificare  */
                /* la rugosità media stradale degli ultimi 15 secondi!           */
                /* ============================================================== */
                int32_t macro_noise_mean = (int32_t)s_sigma_noise;
                if (s_macro_count > 0U)
                {
                    uint8_t check_count = min_uint32((uint32_t)s_macro_count, 15U);
                    int32_t sum_noise = 0;
                    for (uint8_t i = 0; i < check_count; i++)
                    {
                        uint8_t idx = (s_macro_head + APP_MACRO_HISTORY_SECONDS - 1U - i) % APP_MACRO_HISTORY_SECONDS;
                        sum_noise += s_macro_history[idx].mean_sigma_noise_mg;
                    }
                    macro_noise_mean = sum_noise / (int32_t)check_count;
                }

                event_class_t eclass = EVENT_ANOMALY;
                bool is_rough_context = (macro_noise_mean > 120L); /* Siamo su sterrato o pavé pesante da secondi */

                /* Classificazione multiasse (destra/sinistra, dosso, avvallamento) */
                if (abs_int32(s_peak_neg) > (abs_int32(s_peak_pos) * APP_POTHOLE_NEG_DOMINANCE_PERCENT / 100L))
                {
                    if (abs_int32(s_peak_gx) >= APP_TH_ROLL_GX_DPS)
                    {
                        if (s_peak_gx < 0) { eclass = EVENT_POTHOLE_LEFT; }
                        else               { eclass = EVENT_POTHOLE_RIGHT; }
                    }
                    else
                    {
                        eclass = EVENT_POTHOLE_CENTER;
                    }
                }
                else if (abs_int32(s_peak_pos) > (abs_int32(s_peak_neg) * 12L / 10L))
                {
                    eclass = EVENT_BUMP;
                }
                else if ((abs_int32(s_peak_gx) >= APP_TH_ROLL_GX_DPS) &&
                         (peak_to_peak < (s_current_threshold * 15L / 10L)))
                {
                    eclass = EVENT_LATERAL_TILT;
                }
                else if (duration_ms >= 500U)
                {
                    eclass = EVENT_ROUGH;
                }

                /* Calcolo Severità Proporzionale */
                uint32_t denom = (uint32_t)s_current_threshold * (uint32_t)APP_SEV_SCALE_RATIO;
                if (denom < 1U) { denom = 1U; }

                uint32_t sev_calc = 0;
                if (peak_to_peak > s_current_threshold)
                {
                    sev_calc = ((uint32_t)(peak_to_peak - s_current_threshold) * 100U) / denom;
                }
                if ((eclass == EVENT_LATERAL_TILT) && (abs_int32(s_peak_gx) > APP_TH_ROLL_GX_DPS))
                {
                    sev_calc = max_uint32(sev_calc, (uint32_t)(abs_int32(s_peak_gx) * 100L / 150L));
                }
                uint8_t severity = (uint8_t)min_uint32(100U, sev_calc);

                /* Calcolo Confidenza e applicazione della protezione Macro-Context Awareness:
                 * Se siamo in un contesto prolungato di sterrato/pavé (is_rough_context) e l'evento
                 * è marginale o non ha forte beccheggio, penalizziamo la confidenza per proteggere la mappa! */
                uint32_t conf_calc = 75U + (severity / 4U);
                if (abs_int32(s_peak_gy) >= APP_TH_PITCH_GY_DPS)
                {
                    conf_calc += 10U;
                }
                if (is_rough_context)
                {
                    if (severity < 30U)
                    {
                        conf_calc = (conf_calc > 25U) ? (conf_calc - 25U) : 0U;
                    }
                }
                uint8_t confidence = (uint8_t)min_uint32(100U, conf_calc);

                /* Emettiamo l'evento solo se la confidenza è sufficiente */
                if (confidence >= 40U)
                {
                    s_pending_event.timestamp_ms         = s_event_start_time;
                    s_pending_event.event_class          = eclass;
                    s_pending_event.severity             = severity;
                    s_pending_event.confidence           = confidence;
                    s_pending_event.peak_pos_mg          = s_peak_pos;
                    s_pending_event.peak_neg_mg          = s_peak_neg;
                    s_pending_event.peak_z_mg            = s_peak_main;
                    s_pending_event.peak_gx_dps          = s_peak_gx;
                    s_pending_event.peak_gy_dps          = s_peak_gy;
                    s_pending_event.peak_gz_dps          = s_peak_gz;
                    s_pending_event.duration_ms          = (uint16_t)duration_ms;
                    s_pending_event.jerk_max_mg_s        = s_jerk_max;
                    s_pending_event.energy_score         = (uint32_t)(s_energy_sum / 1000ULL);
                    s_pending_event.threshold_mg         = s_current_threshold;
                    s_pending_event.sequence_num         = ++s_sequence_num;

                    s_pending_event_valid = true;
                    s_total_events++;
                }
            }

            s_state = DETECTOR_STATE_REFRACTORY;
            s_refractory_start_time = now_ms;
            break;
        }

        case DETECTOR_STATE_REFRACTORY:
        {
            if ((now_ms - s_refractory_start_time) >= EVENT_REFRACTORY_MS)
            {
                s_state = DETECTOR_STATE_IDLE;
            }
            break;
        }

        default:
            s_state = DETECTOR_STATE_IDLE;
            break;
    }
}

bool RoadDetector_GetPendingEvent(road_event_t *out_event)
{
    if (s_pending_event_valid && (out_event != NULL))
    {
        *out_event = s_pending_event;
        s_pending_event_valid = false;
        return true;
    }
    return false;
}

bool RoadDetector_Get1HzStatus(uint32_t now_ms, road_status_t *out_status)
{
    if ((now_ms - s_last_status_ms) >= STATUS_PERIOD_MS)
    {
        /* 1. Popolamento e salvataggio del record nel Macro-Storico (30 secondi) */
        if (s_sec_sample_count > 0U)
        {
            macro_context_t *mc = &s_macro_history[s_macro_head];
            mc->timestamp_sec       = now_ms / 1000U;
            mc->mean_sigma_noise_mg = (int16_t)(s_sec_sigma_sum / (int32_t)s_sec_sample_count);
            mc->max_az_dyn_mg       = s_sec_max_az_dyn;
            mc->mean_roll_gx_dps    = (int16_t)(s_sec_gx_sum / (int32_t)s_sec_sample_count);
            mc->mean_pitch_gy_dps   = (int16_t)(s_sec_gy_sum / (int32_t)s_sec_sample_count);

            s_macro_head = (s_macro_head + 1U) % APP_MACRO_HISTORY_SECONDS;
            if (s_macro_count < APP_MACRO_HISTORY_SECONDS)
            {
                s_macro_count++;
            }
        }
        s_sec_sample_count = 0;
        s_sec_sigma_sum = 0;
        s_sec_max_az_dyn = 0;
        s_sec_gx_sum = 0;
        s_sec_gy_sum = 0;

        /* 2. Composizione del report 1 Hz con SCENARIO 1: Calcolo Trend a 10s e 30s! */
        if (out_status != NULL)
        {
            out_status->uptime_ms            = now_ms;
            out_status->sigma_noise_mg       = (int32_t)s_sigma_noise;
            out_status->total_events_count   = s_total_events;
            out_status->current_state        = s_state;
            out_status->current_threshold_mg = s_current_threshold;

            /* Calcolo medie su 10 secondi e su 30 secondi prelevandole da s_macro_history */
            int32_t sum_10s = (int32_t)s_sigma_noise;
            int32_t sum_30s = (int32_t)s_sigma_noise;
            uint8_t count_10s = 1U;
            uint8_t count_30s = 1U;

            for (uint8_t i = 0; i < s_macro_count; i++)
            {
                uint8_t idx = (s_macro_head + APP_MACRO_HISTORY_SECONDS - 1U - i) % APP_MACRO_HISTORY_SECONDS;
                int32_t noise_val = s_macro_history[idx].mean_sigma_noise_mg;
                sum_30s += noise_val;
                count_30s++;
                if (i < 10U)
                {
                    sum_10s += noise_val;
                    count_10s++;
                }
            }
            out_status->sigma_noise_10s_mg = sum_10s / (int32_t)count_10s;
            out_status->sigma_noise_30s_mg = sum_30s / (int32_t)count_30s;

            /* Indice di qualità da 0 a 100 */
            int32_t score = 100L - ((int32_t)s_sigma_noise - 25L) * 100L / (200L - 25L);
            if (score > 100L) { score = 100L; }
            if (score < 0L)   { score = 0L; }
            out_status->roughness_score = (uint8_t)score;
        }
        s_last_status_ms = now_ms;
        return true;
    }
    return false;
}

bool RoadDetector_GetMacroHistory(macro_context_t *out_history, uint8_t *out_count)
{
    if ((out_history == NULL) || (out_count == NULL) || (s_macro_count == 0U))
    {
        return false;
    }

    *out_count = s_macro_count;
    for (uint8_t i = 0; i < s_macro_count; i++)
    {
        uint8_t idx = (s_macro_head + APP_MACRO_HISTORY_SECONDS - s_macro_count + i) % APP_MACRO_HISTORY_SECONDS;
        out_history[i] = s_macro_history[idx];
    }
    return true;
}

bool RoadDetector_FormatMacroDumpItem(uint8_t index, char *out_buffer, uint16_t max_len)
{
    if ((out_buffer == NULL) || (index >= s_macro_count) || (max_len < 32U))
    {
        return false;
    }

    /* 0 = più recente (ultimo secondo), 1 = due secondi fa, ecc. */
    uint8_t idx = (s_macro_head + APP_MACRO_HISTORY_SECONDS - 1U - index) % APP_MACRO_HISTORY_SECONDS;
    macro_context_t *mc = &s_macro_history[idx];

    snprintf(out_buffer, max_len,
             "$MACRO_DUMP,%u,%lu,NOISE:%d,MAX_PK:%d,GX:%d,GY:%d\r\n",
             (unsigned int)(index + 1U),
             (unsigned long)mc->timestamp_sec,
             mc->mean_sigma_noise_mg,
             mc->max_az_dyn_mg,
             mc->mean_roll_gx_dps,
             mc->mean_pitch_gy_dps);
    return true;
}

uint8_t RoadDetector_GetMacroCount(void)
{
    return s_macro_count;
}

detector_state_t RoadDetector_GetState(void)
{
    return s_state;
}

int32_t RoadDetector_GetCurrentThreshold(void)
{
    return s_current_threshold;
}

int32_t RoadDetector_GetSigmaNoise(void)
{
    return (int32_t)s_sigma_noise;
}

uint32_t RoadDetector_GetTotalEventsCount(void)
{
    return s_total_events;
}
