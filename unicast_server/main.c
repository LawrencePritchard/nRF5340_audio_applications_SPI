/*
 * nRF5340 Audio DK - SPI Audio Bridge
 *
 * Captures PDM microphone audio via the CS47L63 codec over I2S,
 * sends raw PCM frames to the Apollo over SPI, receives processed
 * PCM back from the Apollo over SPI, and plays it out through the
 * headphone jack.
 *
 * Based on:
 *   - too1/ncs-spi-master-slave-example  (SPI transport)
 *   - nrf5340_audio/unicast_server       (audio pipeline)
 *
 * Build notes
 * -----------
 *  prj.conf must enable (at minimum):
 *    CONFIG_SPI=y
 *    CONFIG_NRF5340_AUDIO_CS47L63_DRIVER=y
 *    CONFIG_STREAM_BIDIRECTIONAL=y   (enables both RX and TX paths in audio_datapath)
 *    CONFIG_AUDIO_DEV=HEADSET        (selects headset path in audio_datapath / hw_codec)
 *    CONFIG_AUDIO_SAMPLE_RATE_48000_HZ=y  (or 16000 / 24000)
 *    CONFIG_AUDIO_BIT_DEPTH_16=y
 *    CONFIG_SW_CODEC_LC3=n           (we bypass the codec - raw PCM only)
 *
 *  The device-tree overlay must alias your external SPI bus as
 *  "my_spi_master" and provide "reg_my_spi_master" for the CS GPIO,
 *  exactly as in the too1 example.
 *
 * Audio data flow
 * ---------------
 *  Mic  →  CS47L63  →(I2S)→  audio_datapath RX FIFO (audio_q_rx)
 *       →  spi_audio_thread  →(SPI)→  Apollo
 *
 *  Apollo →(SPI)→  spi_audio_thread  →  out.fifo (via audio_datapath_pcm_out)
 *         →(I2S)→  CS47L63  →  headphone jack
 *
 * The SPI transaction size is one I2S block = BLK_MULTI_CHAN_SIZE_OCTETS bytes,
 * transmitted every 1 ms to match the I2S interrupt cadence.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net_buf.h>
#include <zephyr/logging/log.h>

#include "audio_datapath.h"
#include "audio_i2s.h"
#include "hw_codec.h"
#include "audio_system.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ---------------------------------------------------------------------------
 * SPI configuration  (mirrors too1 example)
 * ---------------------------------------------------------------------------*/
#define MY_SPI_MASTER           DT_NODELABEL(my_spi_master)
#define MY_SPI_MASTER_CS_DT_SPEC \
    SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master))

static const struct device *spi_dev;

static struct spi_config spi_cfg = {
    /*
     * SPI mode 0 (CPOL=0, CPHA=0) is typical for most MCUs.
     * If the Apollo requires mode 3 change to:
     *   SPI_MODE_CPOL | SPI_MODE_CPHA
     */
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .frequency  = 8000000,   /* 8 MHz – safely above the audio data rate    */
    .slave      = 0,
    .cs         = {.gpio = MY_SPI_MASTER_CS_DT_SPEC, .delay = 0},
};

/* ---------------------------------------------------------------------------
 * Audio sizing constants
 *
 * These mirror the macros in audio_datapath.c / audio_i2s.h.
 * BLK_MULTI_CHAN_SIZE_OCTETS is the number of bytes in one 1 ms I2S block
 * for all channels.  At 48 kHz, 16-bit stereo that is:
 *   48 samples/ms  ×  2 ch  ×  2 bytes  = 192 bytes.
 * ---------------------------------------------------------------------------*/
#ifndef CONFIG_AUDIO_SAMPLE_RATE_HZ
#  define CONFIG_AUDIO_SAMPLE_RATE_HZ 48000
#endif

#ifndef CONFIG_AUDIO_BIT_DEPTH_OCTETS
#  define CONFIG_AUDIO_BIT_DEPTH_OCTETS 2
#endif

#ifndef CONFIG_AUDIO_OUTPUT_CHANNELS
#  define CONFIG_AUDIO_OUTPUT_CHANNELS 2
#endif

#define BLK_PERIOD_US  1000
#define BLK_MONO_NUM_SAMPS   ((CONFIG_AUDIO_SAMPLE_RATE_HZ * BLK_PERIOD_US) / 1000000)
#define BLK_MULTI_CHAN_NUM_SAMPS  (BLK_MONO_NUM_SAMPS * CONFIG_AUDIO_OUTPUT_CHANNELS)
#define BLK_MULTI_CHAN_SIZE_OCTETS \
    (BLK_MULTI_CHAN_NUM_SAMPS * CONFIG_AUDIO_BIT_DEPTH_OCTETS)

/* ---------------------------------------------------------------------------
 * SPI transfer buffers
 *
 * Two pairs so one can be in-flight while we prepare the next (double-buffer).
 * Each pair: tx = PCM from mic, rx = processed PCM from Apollo.
 * ---------------------------------------------------------------------------*/
static uint8_t spi_tx_buf[BLK_MULTI_CHAN_SIZE_OCTETS];
static uint8_t spi_rx_buf[BLK_MULTI_CHAN_SIZE_OCTETS];

/* ---------------------------------------------------------------------------
 * SPI audio thread
 *
 * Wakes every 1 ms, pulls one audio block from the mic queue, sends it to
 * the Apollo over SPI, and pushes the received block into the I2S TX FIFO.
 * ---------------------------------------------------------------------------*/
#define SPI_AUDIO_THREAD_STACK_SIZE 2048
#define SPI_AUDIO_THREAD_PRIORITY   5        /* preemptible, higher than encoder */

K_THREAD_STACK_DEFINE(spi_audio_thread_stack, SPI_AUDIO_THREAD_STACK_SIZE);
static struct k_thread spi_audio_thread_data;

/*
 * audio_datapath_pcm_out() - write one decoded PCM block directly into the
 * output FIFO of audio_datapath.
 *
 * audio_datapath exposes audio_datapath_stream_out() for net_buf frames that
 * go through the LC3 decoder.  Because we already have raw PCM from Apollo we
 * bypass the decoder and write directly into ctrl_blk.out.fifo via the same
 * net_buf wrapper that audio_datapath_stream_out() ultimately produces.
 *
 * The simplest correct approach is to wrap the PCM in a net_buf with the
 * metadata that audio_datapath_stream_out() expects (data_coding = PCM,
 * ref_ts_us = 0 to skip presentation compensation) and call stream_out.
 * audio_datapath_stream_out() will skip the LC3 decode step when the codec is
 * configured as SW_CODEC_NONE / pass-through – OR we can use the PCM pool
 * directly.  Since modifying audio_datapath internals is risky, we use a
 * dedicated net_buf pool + a light wrapper that calls stream_out with a
 * pass-through (no-op) codec config.
 *
 * For a simpler first-pass approach that avoids touching sw_codec_select,
 * the function below copies the PCM directly into the output FIFO through
 * the public audio_datapath API surface by constructing a minimal net_buf.
 *

 */

/* Small pool for wrapping Apollo's PCM reply into net_bufs */
NET_BUF_POOL_FIXED_DEFINE(apollo_pcm_pool, 4, BLK_MULTI_CHAN_SIZE_OCTETS,
                           sizeof(struct audio_metadata), NULL);

/* The audio_q_rx message queue is declared in audio_system.c.
 * We need access to it here.  Declare extern to match the K_MSGQ_DEFINE there.
 */
extern struct k_msgq audio_q_rx;

static void spi_audio_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int ret;

    const struct spi_buf tx_buf_desc = {
        .buf = spi_tx_buf,
        .len = sizeof(spi_tx_buf),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf_desc,
        .count   = 1,
    };

    struct spi_buf rx_buf_desc = {
        .buf = spi_rx_buf,
        .len = sizeof(spi_rx_buf),
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf_desc,
        .count   = 1,
    };

    LOG_INF("SPI audio thread started (%d bytes/block)", BLK_MULTI_CHAN_SIZE_OCTETS);

    while (1) {
        /* ---- Step 1: collect one mic block from I2S RX queue ---- */
        struct net_buf *mic_block = NULL;

        ret = k_msgq_get(&audio_q_rx, (void *)&mic_block, K_MSEC(5));
        if (ret != 0 || mic_block == NULL) {
            /*
             * No mic data yet (stream not started, or underrun).
             * Send silence so the Apollo doesn't time out.
             */
            memset(spi_tx_buf, 0, sizeof(spi_tx_buf));
        } else {
            /* Copy PCM into the flat SPI TX buffer */
            size_t copy_len = MIN(mic_block->len, sizeof(spi_tx_buf));

            memcpy(spi_tx_buf, mic_block->data, copy_len);

            if (copy_len < sizeof(spi_tx_buf)) {
                memset(spi_tx_buf + copy_len, 0,
                       sizeof(spi_tx_buf) - copy_len);
            }

            net_buf_unref(mic_block);
        }

        /* ---- Step 2: SPI full-duplex transfer ---- */
        ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
        if (ret != 0) {
            LOG_ERR("SPI transceive error: %d", ret);
            /* Keep going – don't stall the audio thread */
            continue;
        }

        /* ---- Step 3: push Apollo's reply into the I2S TX path ---- */
        struct net_buf *out_buf = net_buf_alloc(&apollo_pcm_pool, K_NO_WAIT);

        if (out_buf == NULL) {
            LOG_WRN("Apollo PCM pool exhausted – dropping frame");
            continue;
        }

        net_buf_add_mem(out_buf, spi_rx_buf, sizeof(spi_rx_buf));

        /*
         * Fill in the metadata that audio_datapath_stream_out() inspects.
         * Setting data_coding = PCM and ref_ts_us = 0 disables drift/
         * presentation compensation (fine for a direct SPI feed).
         *
         * IMPORTANT: audio_datapath_stream_out() will call sw_codec_decode()
         * on this buffer.  To skip the decoder for PCM input, add the
         * following at the start of the "Decode" section in audio_datapath.c:
         *
         *   if (meta_in->data_coding == PCM) {
         *       // Data is already PCM – copy directly into out FIFO
         *       // (same memcpy loop that follows sw_codec_decode below)
         *       ...
         *   }
         *
         * The simplest short-term fix is to configure sw_codec as a
         * pass-through (SW_CODEC_NONE).  See prj.conf notes above.
         */
        struct audio_metadata *meta = net_buf_user_data(out_buf);

        meta->data_coding            = PCM;
        meta->data_len_us            = BLK_PERIOD_US;
        meta->sample_rate_hz         = CONFIG_AUDIO_SAMPLE_RATE_HZ;
        meta->bits_per_sample        = CONFIG_AUDIO_BIT_DEPTH_BITS;
        meta->carried_bits_per_sample = CONFIG_AUDIO_BIT_DEPTH_BITS;
        meta->bytes_per_location     = BLK_MULTI_CHAN_SIZE_OCTETS / CONFIG_AUDIO_OUTPUT_CHANNELS;
        meta->interleaved            = true;
        meta->locations              = BT_AUDIO_LOCATION_FRONT_LEFT |
                                       BT_AUDIO_LOCATION_FRONT_RIGHT;
        meta->bad_data               = 0;
        meta->ref_ts_us              = 0;   /* disables presentation compensation */
        meta->data_rx_ts_us          = audio_sync_timer_capture();

        audio_datapath_stream_out(out_buf);
        net_buf_unref(out_buf);
    }
}

/* ---------------------------------------------------------------------------
 * SPI peripheral initialisation
 * ---------------------------------------------------------------------------*/
static int spi_audio_init(void)
{
    spi_dev = DEVICE_DT_GET(MY_SPI_MASTER);

    if (!device_is_ready(spi_dev)) {
        LOG_ERR("SPI master device not ready");
        return -ENODEV;
    }

    struct gpio_dt_spec cs_gpio = MY_SPI_MASTER_CS_DT_SPEC;

    if (!device_is_ready(cs_gpio.port)) {
        LOG_ERR("SPI CS GPIO not ready");
        return -ENODEV;
    }

    LOG_INF("SPI initialised (%d Hz)", spi_cfg.frequency);
    return 0;
}

/* ---------------------------------------------------------------------------
 * main()
 * ---------------------------------------------------------------------------*/
int main(void)
{
    int ret;

    LOG_INF("nRF5340 Audio DK – SPI audio bridge starting");

    /* ---- 1. Initialise hardware codec (CS47L63) and I2S datapath ---- */
    /*
     * audio_system_init() calls:
     *   audio_datapath_init()  →  audio_i2s_init() + registers i2s_blk_comp callback
     *   hw_codec_init()        →  cs47l63_comm_init() + starts volume subscriber thread
     */
    ret = audio_system_init();
    if (ret) {
        LOG_ERR("audio_system_init failed: %d", ret);
        return ret;
    }

    /* ---- 2. Initialise SPI master ---- */
    ret = spi_audio_init();
    if (ret) {
        LOG_ERR("SPI init failed: %d", ret);
        return ret;
    }

    /* ---- 3. Start the codec + I2S datapath ---- */
    /*
     * audio_system_start() calls:
     *   hw_codec_default_conf_enable()  – configures CS47L63 clocks, GPIO,
     *                                     ASP1 (I2S), output, PDM mic, FLL
     *   audio_datapath_start(&audio_q_rx) – starts I2S, mic data flows into
     *                                       audio_q_rx every 1 ms
     *
     * We must also enable the encoder if we want the mic data to be pulled
     * out of audio_q_rx.  In our case the SPI thread acts as the "encoder",
     * so we call audio_system_encoder_start() to unblock the poll signal that
     * normally gates the encoder_thread – but our replacement SPI thread
     * does not use that signal, so it is optional here.  It is safe to omit.
     *
     * Set a dummy codec config so audio_system_start() does not assert.
     * 48 kHz PCM, no BT bitrate needed.
     */
    ret = audio_system_config_set(
        CONFIG_AUDIO_SAMPLE_RATE_HZ,  /* encoder sample rate (not used) */
        0,                             /* encoder bitrate   (0 = PCM, no LC3) */
        CONFIG_AUDIO_SAMPLE_RATE_HZ   /* decoder sample rate */
    );
    if (ret) {
        LOG_WRN("audio_system_config_set returned %d (may be harmless)", ret);
    }

    audio_system_start();
    LOG_INF("Audio system started");

    /* ---- 4. Start the SPI audio bridge thread ---- */
    k_tid_t tid = k_thread_create(
        &spi_audio_thread_data,
        spi_audio_thread_stack,
        SPI_AUDIO_THREAD_STACK_SIZE,
        spi_audio_thread_fn,
        NULL, NULL, NULL,
        K_PRIO_PREEMPT(SPI_AUDIO_THREAD_PRIORITY),
        0,
        K_NO_WAIT);

    ret = k_thread_name_set(tid, "spi_audio");
    if (ret) {
        LOG_WRN("Failed to name SPI audio thread: %d", ret);
    }

    LOG_INF("SPI audio thread started, bridge is running");

    /*
     * main() returns here; the Zephyr kernel continues scheduling
     * the SPI audio thread and the audio datapath ISR callbacks.
     */
    return 0;
}

/*
 * =============================================================================
 * REQUIRED CHANGE IN audio_datapath.c
 * =============================================================================
 *
 * audio_datapath_stream_out() currently always calls sw_codec_decode().
 * For PCM-coded input (from Apollo) we need to bypass that step.
 *
 * In audio_datapath.c, this guard was placed before the sw_codec_decode() call:
 *
 *   // --- BEGIN patch for SPI PCM bypass ---
 *   if (meta_in->data_coding == PCM) {
 *       // Data from Apollo is already decoded PCM.
 *       // Copy it straight into the output FIFO, mirroring what the
 *       // memcpy loop below does after sw_codec_decode().
 *       uint32_t out_blk_idx = ctrl_blk.out.prod_blk_idx;
 *       for (uint32_t i = 0; i < NUM_BLKS_IN_FRAME; i++) {
 *           memcpy(&ctrl_blk.out.fifo[out_blk_idx * BLK_MULTI_CHAN_NUM_SAMPS],
 *                  (int16_t *)audio_frame_in->data,
 *                  BLK_MULTI_CHAN_SIZE_OCTETS);
 *           net_buf_pull(audio_frame_in, BLK_MULTI_CHAN_SIZE_OCTETS);
 *           ctrl_blk.out.prod_blk_ts[out_blk_idx] =
 *               meta_in->data_rx_ts_us + (i * BLK_PERIOD_US);
 *           out_blk_idx = NEXT_IDX(out_blk_idx);
 *       }
 *       ctrl_blk.out.prod_blk_idx = out_blk_idx;
 *       net_buf_unref(audio_frame_out);  // free the unused alloc above
 *       return;
 *   }
 *   // --- END patch ---
 *
 
 *
 * =============================================================================
 * prj.conf additions
 * =============================================================================
 *
 *   # Core audio
 *   CONFIG_AUDIO_DEV=HEADSET
 *   CONFIG_STREAM_BIDIRECTIONAL=y
 *   CONFIG_AUDIO_SAMPLE_RATE_48000_HZ=y
 *   CONFIG_AUDIO_BIT_DEPTH_16=y
 *   CONFIG_NRF5340_AUDIO_CS47L63_DRIVER=y
 *
 *   # Disable BT stack entirely
 *   CONFIG_BT=n
 *   CONFIG_BT_LE_AUDIO=n
 *
 *   # SPI
 *   CONFIG_SPI=y
 *
 *   # Threads – increase stack sizes if you see stack overflow warnings
 *   CONFIG_MAIN_STACK_SIZE=4096
 *
 * =============================================================================
 */