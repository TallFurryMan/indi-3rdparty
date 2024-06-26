/**
 * Copyright (C) 2013 Ben Gilsrud
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <sys/time.h>
#include <memory>
#include <stdint.h>
#include <arpa/inet.h>
#include <math.h>
#include <sys/time.h>
#include <dc1394/dc1394.h>
#include <indiapi.h>
#include <iostream>

#include "ffmv_ccd.h"
#include "config.h"

std::unique_ptr<FFMVCCD> ffmvCCD(new FFMVCCD());

/**
 * Write to registers in the MT9V022 chip.
 * This can be done by programming the address in 0x1A00 and writing to 0x1A04.
 */
dc1394error_t FFMVCCD::writeMicronReg(unsigned int offset, unsigned int val)
{
    dc1394error_t err;
    err = dc1394_set_control_register(dcam, 0x1A00, offset);
    if (err != DC1394_SUCCESS)
    {
        return err;
    }

    err = dc1394_set_control_register(dcam, 0x1A04, val);
    if (err != DC1394_SUCCESS)
    {
        return err;
    }

    return DC1394_SUCCESS;
}

/**
 * Read a register from the MT9V022 sensor.
 * This writes the MT9V022 register address to 0x1A00 and then reads from 0x1A04
 */
dc1394error_t FFMVCCD::readMicronReg(unsigned int offset, unsigned int *val)
{
    dc1394error_t err;
    err = dc1394_set_control_register(dcam, 0x1A00, offset);
    if (err != DC1394_SUCCESS)
    {
        return err;
    }

    err = dc1394_get_control_register(dcam, 0x1A04, val);
    if (err != DC1394_SUCCESS)
    {
        return err;
    }

    return DC1394_SUCCESS;
}

FFMVCCD::FFMVCCD()
{
    InExposure = false;
    capturing  = false;

    setVersion(FFMV_VERSION_MAJOR, FFMV_VERSION_MINOR);

    SetCCDCapability(CCD_CAN_ABORT);
}

/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool FFMVCCD::Connect()
{
    dc1394camera_list_t *list;
    dc1394error_t err;
    float min, max;

    dc1394 = dc1394_new();
    if (!dc1394)
    {
        return false;
    }

    err = dc1394_camera_enumerate(dc1394, &list);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Could not find DC1394 cameras!");
        return false;
    }
    if (!list->num)
    {
        LOG_ERROR("No DC1394 cameras found!");
        return false;
    }
    dcam = dc1394_camera_new(dc1394, list->ids[0].guid);
    if (!dcam)
    {
        LOG_ERROR("Unable to connect to camera!");
        return false;
    }

    /* Reset camera */
    err = dc1394_camera_reset(dcam);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Unable to reset camera!");
        return false;
    }

    /* Set mode */
    err = dc1394_video_set_mode(dcam, DC1394_VIDEO_MODE_640x480_MONO16);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Unable to connect to set videomode!");
        return false;
    }
    /* Disable Auto exposure control */
    err = dc1394_feature_set_power(dcam, DC1394_FEATURE_EXPOSURE, DC1394_OFF);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Unable to disable auto exposure control");
        return false;
    }

    /* Set frame rate to the lowest possible */
    err = dc1394_video_set_framerate(dcam, DC1394_FRAMERATE_7_5);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Unable to connect to set framerate!");
        return false;
    }
    /* Turn frame rate control off to enable extended exposure (subs of 512ms) */
    err = dc1394_feature_set_power(dcam, DC1394_FEATURE_FRAME_RATE, DC1394_OFF);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Unable to disable framerate!");
        return false;
    }

    /* Get the longest possible exposure length */
    err = dc1394_feature_set_mode(dcam, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Failed to enable manual shutter control.");
    }
    err = dc1394_feature_set_absolute_control(dcam, DC1394_FEATURE_SHUTTER, DC1394_ON);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Failed to enable absolute shutter control.");
    }
    err = dc1394_feature_get_absolute_boundaries(dcam, DC1394_FEATURE_SHUTTER, &min, &max);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Could not get max shutter length");
    }
    else
    {
        max_exposure = max;
    }

    /* Set gain to max. By setting the register directly, we can achieve a
     * gain of 24 dB...compared to a gain of 12 dB which is reported as the
     * max
     */
    err = dc1394_set_control_register(dcam, 0x820, 0x40);
    //err = dc1394_set_control_register(dcam, 0x820, 0x7f);
    if (err != DC1394_SUCCESS)
    {
        return err;
    }
#if 0
    /* Set absolute gain to max */
    err = dc1394_feature_set_absolute_control(dcam, DC1394_FEATURE_GAIN, DC1394_ON);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Failed to enable ansolute gain control.");
    }
    err = dc1394_feature_get_absolute_boundaries(dcam, DC1394_FEATURE_GAIN, &min, &max);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Could not get max gain value");
    }
    else
    {
        err = dc1394_feature_set_absolute_value(dcam, DC1394_FEATURE_GAIN, max);
        if (err != DC1394_SUCCESS)
        {
            LOG_ERROR("Could not set max gain value");
        }
    }
#endif

    /* Set brightness */
    err = dc1394_feature_set_mode(dcam, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Failed to enable manual brightness control.");
    }
    err = dc1394_feature_set_absolute_control(dcam, DC1394_FEATURE_BRIGHTNESS, DC1394_ON);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Failed to enable ansolute brightness control.");
    }
    err = dc1394_feature_set_absolute_value(dcam, DC1394_FEATURE_BRIGHTNESS, 1);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Could not set max brightness value");
    }

    /* Turn gamma control off */
    err = dc1394_feature_set_absolute_value(dcam, DC1394_FEATURE_GAMMA, 1);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Could not set gamma value");
    }
    err = dc1394_feature_set_power(dcam, DC1394_FEATURE_GAMMA, DC1394_OFF);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Unable to disable gamma!");
        return false;
    }

    /* Turn off white balance */
    err = dc1394_feature_set_power(dcam, DC1394_FEATURE_WHITE_BALANCE, DC1394_OFF);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Unable to disable white balance!");
        return false;
    }

    err = dc1394_capture_setup(dcam, 10, DC1394_CAPTURE_FLAGS_DEFAULT);

    LOGF_INFO("Detected camera model: %s vendor: %s (%#04X:%#04X)", dcam->model, dcam->model, dcam->vendor_id, dcam->model_id);

    return true;
}

/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool FFMVCCD::Disconnect()
{
    if (dcam)
    {
        dc1394_capture_stop(dcam);
        dc1394_camera_free(dcam);
    }

    LOG_INFO("Point Grey FireFly MV disconnected successfully!");
    return true;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *FFMVCCD::getDefaultName()
{
    return "FireFly MV";
}

/**************************************************************************************
** INDI is asking us to init our properties.
***************************************************************************************/
bool FFMVCCD::initProperties()
{
    // Must init parent properties first!
    INDI::CCD::initProperties();

    // Add Debug, Simulator, and Configuration controls
    addAuxControls();

    CaptureFormat color = {"INDI_RGB", "RGB", 8, true};
    addCaptureFormat(color);

    /* Add Gain Vref switch */
    IUFillSwitch(&GainS[0], "GAINVREF", "Vref Boost", ISS_OFF);
    IUFillSwitch(&GainS[1], "GAIN2X", "2x Digital Boost", ISS_OFF);
    IUFillSwitchVector(&GainSP, GainS, 2, getDeviceName(), "GAIN", "Gain", IMAGE_SETTINGS_TAB, IP_WO, ISR_NOFMANY, 0,
                       IPS_IDLE);

    setDefaultPollingPeriod(250);

    return true;
}

/********************************************************************************************
** INDI is asking us to update the properties because there is a change in CONNECTION status
** This fucntion is called whenever the device is connected or disconnected.
*********************************************************************************************/
bool FFMVCCD::updateProperties()
{
    // Call parent update properties first
    INDI::CCD::updateProperties();

    if (isConnected())
    {
        // Let's get parameters now from CCD
        setupParams();

        // Start the timer
        SetTimer(getCurrentPollingPeriod());
        defineProperty(&GainSP);
    }
    else
    {
        deleteProperty(GainSP.name);
    }

    return true;
}

/**************************************************************************************
** Setting up CCD parameters
***************************************************************************************/
void FFMVCCD::setupParams()
{
    // Atik GP has Sony ICX445 EXview HAD II CCD
    if (dcam->model_id == 0x2005)
        SetCCDParams(1192, 964, 16, 3.75, 3.75);
    // The FireFly MV has a Micron MT9V022 CMOS sensor
    else
        SetCCDParams(640, 480, 16, 6.0, 6.0);

    // Let's calculate how much memory we need for the primary CCD buffer
    uint32_t nbuf = PrimaryCCD.getXRes() * PrimaryCCD.getYRes() * PrimaryCCD.getBPP() / 8;
    PrimaryCCD.setFrameBufferSize(nbuf);
}

#define IMAGE_FILE_NAME "testimage.pgm"
/**************************************************************************************
** Client is asking us to start an exposure
***************************************************************************************/
bool FFMVCCD::StartExposure(float duration)
{
    dc1394error_t err;
    dc1394video_frame_t *frame;
    int ms;
    float sub_length;
    float fval;

    ms = duration * 1000;

    //LOG_ERROR("Doing %d sub exposures at %f %s each", sub_count, absShutter, prop_info.pUnits);

    ExposureRequest = duration;

    // Since we have only have one CCD with one chip, we set the exposure duration of the primary CCD
    PrimaryCCD.setBPP(16);
    PrimaryCCD.setExposureDuration(duration);

    gettimeofday(&ExpStart, nullptr);

    InExposure = true;
    LOG_ERROR("Exposure has begun.");

    // Let's get a pointer to the frame buffer
    uint8_t *image = PrimaryCCD.getFrameBuffer();

    memset(image, 0, PrimaryCCD.getFrameBufferSize());

    if (duration != last_exposure_length)
    {
        /* Calculate the number of exposures needed */
        sub_count = duration / max_exposure;
        if (ms % ((int)(max_exposure * 1000)))
        {
            ++sub_count;
        }
        sub_length = duration / sub_count;

        LOGF_DEBUG("Triggering a %f second exposure using %d subs of %f seconds", duration, sub_count,
                   sub_length);
        /* Set sub length */
#if 0
        err = dc1394_feature_set_absolute_control(dcam, DC1394_FEATURE_SHUTTER, DC1394_ON);
        if (err != DC1394_SUCCESS)
        {
            LOG_ERROR("Failed to enable ansolute shutter control.");
        }
#endif
        err = dc1394_feature_set_absolute_value(dcam, DC1394_FEATURE_SHUTTER, sub_length);
        if (err != DC1394_SUCCESS)
        {
            LOG_ERROR("Unable to set shutter value.");
        }
        err = dc1394_feature_get_absolute_value(dcam, DC1394_FEATURE_SHUTTER, &fval);
        if (err != DC1394_SUCCESS)
        {
            LOG_ERROR("Unable to get shutter value.");
        }
        LOGF_DEBUG("Shutter value is %f.", fval);
    }

    /* Flush the DMA buffer */
    while (1)
    {
        err = dc1394_capture_dequeue(dcam, DC1394_CAPTURE_POLICY_POLL, &frame);
        if (err != DC1394_SUCCESS)
        {
            LOG_ERROR("Flushing DMA buffer failed!");
            break;
        }
        if (!frame)
        {
            break;
        }
        dc1394_capture_enqueue(dcam, frame);
    }

    /*-----------------------------------------------------------------------
     *  have the camera start sending us data
     *-----------------------------------------------------------------------*/
    LOG_DEBUG("start transmission");
    err = dc1394_video_set_transmission(dcam, DC1394_ON);
    if (err != DC1394_SUCCESS)
    {
        LOG_ERROR("Unable to start transmission");
        return false;
    }

    // We're done
    return true;
}

/**************************************************************************************
** Client is asking us to abort an exposure
***************************************************************************************/
bool FFMVCCD::AbortExposure()
{
    InExposure = false;
    return true;
}

/**************************************************************************************
** How much longer until exposure is done?
***************************************************************************************/
float FFMVCCD::CalcTimeLeft()
{
    struct timeval now;
    gettimeofday(&now, nullptr);

    double timesince = (double)(now.tv_sec * 1000.0 + now.tv_usec / 1000) -
                       (double)(ExpStart.tv_sec * 1000.0 + ExpStart.tv_usec / 1000);
    timesince = timesince / 1000;

    double timeleft = ExposureRequest - timesince;
    return timeleft;
}

/**
 * Set the digital gain in the MT9V022.
 * Sets the tiled digital gain to 2x what the default is.
 */
dc1394error_t FFMVCCD::setDigitalGain(ISState state)
{
    dc1394error_t err;
    unsigned int val;

    if (state == ISS_OFF)
    {
        err = writeMicronReg(0x80, 0xF4);
        err = readMicronReg(0x80, &val);
        if (err == DC1394_SUCCESS)
        {
            LOGF_DEBUG("Turned off digital gain boost. Tiled Digital Gain = 0x%x", val);
        }
        else
        {
            LOG_ERROR("readMicronReg failed!");
        }
    }
    else
    {
        err = writeMicronReg(0x80, 0xF8);
        err = readMicronReg(0x80, &val);
        if (err == DC1394_SUCCESS)
        {
            LOGF_DEBUG("Turned on digital gain boost. Tiled Digital Gain = 0x%x", val);
        }
        else
        {
            LOG_ERROR("readMicronReg failed!");
        }
    }
    return DC1394_SUCCESS;
}

/**
 * Set the ADC reference voltage for the MT9V022.
 * Decreasing the reference voltage will, in effect, increase the gain.
 * The default Vref_ADC is 1.4V (4). We will bump it down to 1.0V (0).
 */
dc1394error_t FFMVCCD::setGainVref(ISState state)
{
    dc1394error_t err;
    unsigned int val;

    if (state == ISS_OFF)
    {
        err = writeMicronReg(0x2C, 4);
        err = readMicronReg(0x2C, &val);
        if (err == DC1394_SUCCESS)
        {
            LOGF_DEBUG("Turned off Gain boost. VREF_ADC = 0x%x", val);
        }
        else
        {
            LOG_ERROR("readMicronReg failed!");
        }
    }
    else
    {
        err = writeMicronReg(0x2C, 0);
        err = readMicronReg(0x2C, &val);
        if (err == DC1394_SUCCESS)
        {
            LOGF_DEBUG("Turned on Gain boost. VREF_ADC = 0x%x", val);
        }
        else
        {
            LOG_ERROR("readMicronReg failed!");
        }
    }
    return DC1394_SUCCESS;
}

bool FFMVCCD::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (strcmp(dev, getDeviceName()) == 0)
    {
        /* Reset */
        if (!strcmp(name, GainSP.name))
        {
            if (IUUpdateSwitch(&GainSP, states, names, n) < 0)
            {
                return false;
            }
            setGainVref(GainS[0].s);
            setDigitalGain(GainS[1].s);
            return true;
        }
    }

    //  Nobody has claimed this, so, ignore it
    return INDI::CCD::ISNewSwitch(dev, name, states, names, n);
}

/**************************************************************************************
** Main device loop. We check for exposure progress
***************************************************************************************/
void FFMVCCD::TimerHit()
{
    if (isConnected() == false)
        return;

    if (InExposure)
    {
        double timeleft = CalcTimeLeft();

        // Less than a 0.1 second away from exposure completion
        // This is an over simplified timing method, check CCDSimulator and ffmvCCD for better timing checks
        if (timeleft < 0.1)
        {
            /* We're done exposing */
            LOG_DEBUG("Exposure done, downloading image...");

            // Set exposure left to zero
            PrimaryCCD.setExposureLeft(0);

            // We're no longer exposing...
            InExposure = false;

            /* grab and save image */
            grabImage();
        }
        else
        {
            // Just update time left in client
            PrimaryCCD.setExposureLeft(timeleft);
        }
    }

    SetTimer(getCurrentPollingPeriod());
    return;
}

/**
 * Download image from FireFly
 */
void FFMVCCD::grabImage()
{
    dc1394error_t err;
    dc1394video_frame_t *frame;
    uint32_t uheight, uwidth;
    int sub;
    uint16_t val;
    struct timeval start, end;

    std::unique_lock<std::mutex> guard(ccdBufferLock);
    // Let's get a pointer to the frame buffer
    uint8_t *image = PrimaryCCD.getFrameBuffer();

    // Get width and height
    int width  = PrimaryCCD.getSubW() / PrimaryCCD.getBinX();
    int height = PrimaryCCD.getSubH() / PrimaryCCD.getBinY();

    memset(image, 0, PrimaryCCD.getFrameBufferSize());

    /*-----------------------------------------------------------------------
    *  stop data transmission
    *-----------------------------------------------------------------------*/

    gettimeofday(&start, nullptr);
    for (sub = 0; sub < sub_count; ++sub)
    {
        LOGF_DEBUG("Getting sub %d of %d", sub, sub_count);
        err = dc1394_capture_dequeue(dcam, DC1394_CAPTURE_POLICY_WAIT, &frame);
        if (err != DC1394_SUCCESS)
        {
            LOG_ERROR("Could not capture frame");
        }
        dc1394_get_image_size_from_video_mode(dcam, DC1394_VIDEO_MODE_640x480_MONO16, &uwidth, &uheight);

        if (DC1394_TRUE == dc1394_capture_is_frame_corrupt(dcam, frame))
        {
            LOG_ERROR("Corrupt frame!");
            continue;
        }
        // Fill buffer with random pattern
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                /* Detect unsigned overflow */
                val = ((uint16_t *)image)[i * width + j] + ntohs(((uint16_t *)(frame->image))[i * width + j]);
                if (val > ((uint16_t *)image)[i * width + j])
                {
                    ((uint16_t *)image)[i * width + j] = val;
                }
                else
                {
                    ((uint16_t *)image)[i * width + j] = 0xFFFF;
                }
            }
        }

        dc1394_capture_enqueue(dcam, frame);
    }
    guard.unlock();
    err = dc1394_video_set_transmission(dcam, DC1394_OFF);
    gettimeofday(&end, nullptr);
    LOGF_DEBUG("Download took %d uS", (int)((end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec)));

    // Let INDI::CCD know we're done filling the image buffer
    ExposureComplete(&PrimaryCCD);
}
