/*
    Moonlite Focuser Fork with humidity and dust cap
    Copyright (C) 2013-2019 Jasem Mutlaq (mutlaqja@ikarustech.com)
    Copyright (C) 2022 Eric Dejouhanet (eric.dejouhanet@gmail.com)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "moonduino.h"

#include <indicom.h>

#include <cmath>
#include <cstring>
#include <memory>

#include <termios.h>
#include <unistd.h>

#include "config.h"

static std::unique_ptr<MoonDuino> moonLite(new MoonDuino());

MoonDuino::MoonDuino():
    Focuser(),
    WeatherInterface(this),
    m_DustCap(this)
{
    setVersion(MOONDUINO_VERSION_MAJOR, MOONDUINO_VERSION_MINOR);
}

bool MoonDuino::initProperties()
{
    INDI::DefaultDevice::initProperties();

    // Can move in Absolute & Relative motions, can AbortFocuser motion, and has variable speed.
    FI::SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_HAS_VARIABLE_SPEED |
                      FOCUSER_CAN_SYNC);

    INDI::Focuser::initProperties();
    INDI::WeatherInterface::initProperties(MAIN_CONTROL_TAB, OPTIONS_TAB);

    FocusSpeedN[0].min   = 1;
    FocusSpeedN[0].max   = 5;
    FocusSpeedN[0].value = 1;

    // Step Mode
    StepModeS[FOCUS_HALF_STEP].fill("FOCUS_HALF_STEP", "Half Step", ISS_OFF);
    StepModeS[FOCUS_FULL_STEP].fill("FOCUS_FULL_STEP", "Full Step", ISS_ON);
    StepModeS.fill(getDeviceName(), "Step Mode", "",
                   OPTIONS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Focuser temperature
    TemperatureN[0].fill("TEMPERATURE", "Celsius", "%6.2f", -50, 70., 0., 0.);
    TemperatureN.fill(getDeviceName(), "FOCUS_TEMPERATURE", "Temperature",
                      MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Focuser humidity
    HumidityN[0].fill("HUMIDITY", "Percent", "%6.2f", 0., 100., 0., 0.);
    HumidityN.fill(getDeviceName(), "FOCUS_HUMIDITY", "Humidity",
                   MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    // Temperature Settings
    TemperatureSettingN[0].fill("Calibration", "", "%6.2f", -100, 100, 0.5, 0);
    TemperatureSettingN[1].fill("Coefficient", "", "%6.2f", -100, 100, 0.5, 0);
    TemperatureSettingN.fill(getDeviceName(), "T. Settings", "",
                             OPTIONS_TAB, IP_RW, 0, IPS_IDLE);

    // Compensate for temperature
    TemperatureCompensateS[0].fill("Enable", "", ISS_OFF);
    TemperatureCompensateS[1].fill("Disable", "", ISS_ON);
    TemperatureCompensateS.fill(getDeviceName(), "T. Compensate", "",
                                MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    /* Relative and absolute movement */
    FocusRelPosN[0].min   = 0.;
    FocusRelPosN[0].max   = 50000.;
    FocusRelPosN[0].value = 0;
    FocusRelPosN[0].step  = 1000;

    FocusAbsPosN[0].min   = 0.;
    FocusAbsPosN[0].max   = 100000.;
    FocusAbsPosN[0].value = 0;
    FocusAbsPosN[0].step  = 1000;

    // Environment
    addParameter("WEATHER_TEMPERATURE", "Temperature (C)", -15, 35, 15);
    addParameter("WEATHER_HUMIDITY", "Humidity %", 0, 100, 15);
    //addParameter("WEATHER_DEWPOINT", "Dew Point (C)", 0, 100, 15);
    //setCriticalParameter("WEATHER_TEMPERATURE");

    //initDustCapProperties(getDeviceName(), MAIN_CONTROL_TAB);
    //initLightBoxProperties(getDeviceName(), MAIN_CONTROL_TAB);

    setDriverInterface(AUX_INTERFACE | FOCUSER_INTERFACE | WEATHER_INTERFACE);

    addAuxControls();

    setDefaultPollingPeriod(1000);
    addDebugControl();

    return true;
}

bool MoonDuino::updateProperties()
{
    INDI::Focuser::updateProperties();
    INDI::WeatherInterface::updateProperties();

    if (isConnected())
    {
        defineProperty(TemperatureN);
        defineProperty(HumidityN);
        defineProperty(StepModeS);
        defineProperty(TemperatureSettingN);
        defineProperty(TemperatureCompensateS);

        GetFocusParams();

        LOG_INFO("MoonDuino parameters updated, focuser ready for use.");

        m_DustCap.setConnected(true);
        m_DustCap.updateProperties();
    }
    else
    {
        deleteProperty(TemperatureN);
        deleteProperty(HumidityN);
        deleteProperty(StepModeS);
        deleteProperty(TemperatureSettingN);
        deleteProperty(TemperatureCompensateS);

        m_DustCap.setConnected(false);
        m_DustCap.updateProperties();
    }

    return true;
}

IPState Moonduino::updateWeather() {
    return IPS_ALERT;
}

bool MoonDuino::Handshake()
{
    if (Ack())
    {
        LOG_INFO("MoonDuino is online. Getting focus parameters...");
        return true;
    }

    LOG_INFO(
        "Error retrieving data from MoonDuino, please ensure MoonDuino controller is powered and the port is correct.");
    return false;
}

const char * MoonDuino::getDefaultName()
{
    return "MoonDuino";
}

bool MoonDuino::Ack()
{
    bool success = false;

    for (int i = 0; i < 3; i++)
    {
        if (readVersion())
        {
            success = true;
            break;
        }

        sleep(1);
    }

    return success;
}

bool MoonDuino::readStepMode()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GH#", res) == false)
        return false;

    if (strcmp(res, "FF#") == 0)
        StepModeS[FOCUS_HALF_STEP].setState(ISS_ON);
    else if (strcmp(res, "00#") == 0)
        StepModeS[FOCUS_FULL_STEP].setState(ISS_ON);
    else
    {
        LOGF_ERROR("Unknown error: focuser step value (%s)", res);
        return false;
    }

    return true;
}

bool MoonDuino::readVersion()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GV#", res, true, 2) == false)
        return false;

    if (res[0] < '0' || '9' < res[0])
        res[0] = '0';
    if (res[1] < '0' || '9' < res[1])
        res[1] = '0';

    LOGF_INFO("Detected firmware version %c.%c", res[0], res[1]);

    return true;
}

bool MoonDuino::readTemperature()
{
    char res[ML_RES] = {0};

    sendCommand(":C#");

    if (sendCommand(":GT#", res) == false)
        return false;

    uint32_t temp = 0;
    int rc = sscanf(res, "%X", &temp);
    if (rc > 0)
        // Signed hex
        TemperatureN[0].setValue(static_cast<int16_t>(temp) / 2.0);
    else
    {
        LOGF_ERROR("Unknown error: focuser temperature value (%s)", res);
        return false;
    }

    return true;
}

bool MoonDuino::readHumidity()
{
    char res[ML_RES] = {0};

    sendCommand(":C#");

    if (sendCommand(":GM#", res) == false)
        return false;

    uint32_t value = 0;
    int rc = sscanf(res, "%X", &value);
    if (rc > 0)
        // Unsigned hex
        HumidityN[0].setValue(static_cast<uint16_t>(value) / 2.0);
    else
    {
        LOGF_ERROR("Unknown error: focuser humidity value (%s)", res);
        return false;
    }

    return true;
}

bool MoonDuino::readPosition()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GP#", res) == false)
        return false;

    int32_t pos;
    int rc = sscanf(res, "%X#", &pos);

    if (rc > 0)
        FocusAbsPosN[0].value = pos;
    else
    {
        LOGF_ERROR("Unknown error: focuser position value (%s)", res);
        return false;
    }

    return true;
}

bool MoonDuino::readSpeed()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GD#", res) == false)
        return false;

    uint16_t speed = 0;
    int rc = sscanf(res, "%hX#", &speed);

    if (rc > 0)
    {
        LOGF_INFO("Focuser speed is coded %d.", speed);
        int focus_speed = 1;
        for (; speed > 2; speed >>= 1)
            focus_speed++;
        FocusSpeedN[0].value = focus_speed;
    }
    else
    {
        LOGF_ERROR("Unknown error: focuser speed value (%s)", res);
        return false;
    }

    return true;
}

bool MoonDuino::isMoving()
{
    char res[ML_RES] = {0};

    if (sendCommand(":GI#", res) == false)
        return false;

    // JM 2020-03-13: 01# and 1# should be both accepted
    if (strstr(res, "1#"))
        return true;
    else if (strstr(res, "0#"))
        return false;

    LOGF_ERROR("Unknown error: isMoving value (%s)", res);
    return false;
}

bool MoonDuino::setTemperatureCalibration(double calibration)
{
    char cmd[ML_RES] = {0};
    uint8_t hex = static_cast<int8_t>(calibration * 2) & 0xFF;
    snprintf(cmd, ML_RES, ":PO%02X#", hex);
    return sendCommand(cmd);
}

bool MoonDuino::setTemperatureCoefficient(double coefficient)
{
    char cmd[ML_RES] = {0};
    uint8_t hex = static_cast<int8_t>(coefficient * 2) & 0xFF;
    snprintf(cmd, ML_RES, ":SC%02X#", hex);
    return sendCommand(cmd);
}

bool MoonDuino::SyncFocuser(uint32_t ticks)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":SP%04X#", ticks);
    return sendCommand(cmd);
}

bool MoonDuino::MoveFocuser(uint32_t position)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":SN%04X#", position);
    // Set Position First
    if (sendCommand(cmd) == false)
        return false;
    // Now start motion toward position
    if (sendCommand(":FG#") == false)
        return false;

    return true;
}

bool MoonDuino::setStepMode(FocusStepMode mode)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":S%c#", (mode == FOCUS_HALF_STEP) ? 'H' : 'F');
    return sendCommand(cmd);
}

bool MoonDuino::setSpeed(int speed)
{
    char cmd[ML_RES] = {0};
    int hex_value = 1;
    hex_value <<= speed;
    snprintf(cmd, ML_RES, ":SD%02X#", hex_value);
    return sendCommand(cmd);
}

bool MoonDuino::setTemperatureCompensation(bool enable)
{
    char cmd[ML_RES] = {0};
    snprintf(cmd, ML_RES, ":%c#", enable ? '+' : '-');
    return sendCommand(cmd);
}

bool MoonDuino::ISNewSwitch(const char * dev, const char * name, ISState * states, char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Focus Step Mode
        if (StepModeS.isNameMatch(name))
        {
            int const current_mode = StepModeS.findOnSwitchIndex();

            StepModeS.update(states, names, n);

            int const target_mode = StepModeS.findOnSwitchIndex();

            if (current_mode == target_mode)
            {
                StepModeS.setState(IPS_OK);
                StepModeS.apply();
                return true;
            }

            bool rc = setStepMode(target_mode == 0 ? FOCUS_HALF_STEP : FOCUS_FULL_STEP);
            if (!rc)
            {
                StepModeS.reset();
                StepModeS[current_mode].setState(ISS_ON);
                StepModeS.setState(IPS_ALERT);
                StepModeS.apply();
                return false;
            }

            StepModeS.setState(IPS_OK);
            StepModeS.apply();
            return true;
        }

        // Temperature Compensation Mode
        if (TemperatureCompensateS.isNameMatch(name))
        {
            int const last_index = TemperatureCompensateS.findOnSwitchIndex();
            TemperatureCompensateS.update(states, names, n);

            bool rc = setTemperatureCompensation(ISS_ON == TemperatureCompensateS[0].getState());

            if (!rc)
            {
                TemperatureCompensateS.reset();
                TemperatureCompensateS[last_index].setState(ISS_ON);
                TemperatureCompensateS.setState(IPS_ALERT);
                TemperatureCompensateS.apply();
                return false;
            }

            TemperatureCompensateS.setState(IPS_OK);
            TemperatureCompensateS.apply();
            return true;
        }

        if (INDI::WeatherInterface::processSwitch(dev, name, states, names, n))
            return true;
    }

    return INDI::Focuser::processSwitch(dev, name, states, names, n);
}

bool MoonDuino::ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Temperature Settings
        if (TemperatureSettingN.isNameMatch(name))
        {
            TemperatureSettingN.update(values, names, n);
            if (!setTemperatureCalibration(TemperatureSettingN[0].getValue()) ||
                !setTemperatureCoefficient(TemperatureSettingN[1].getValue()))
            {
                TemperatureSettingN.setState(IPS_ALERT);
                TemperatureSettingN.apply();
                return false;
            }

            TemperatureSettingN.setState(IPS_OK);
            TemperatureSettingN.apply();
            return true;
        }
        
        if (INDI::WeatherInterface::processNumber(dev, name, values, names, n))
            return true;
    }

    return INDI::Focuser::processNumber(dev, name, values, names, n);
}

void MoonDuino::GetFocusParams()
{
    if (readPosition())
        IDSetNumber(&FocusAbsPosNP, nullptr);
    
    if (readSpeed())
        IDSetNumber(&FocusSpeedNP, nullptr);

    if (readTemperature())
        TemperatureN.apply();

    if (readHumidity())
        HumidityN.apply();

    if (readStepMode())
        StepModeS.apply();
}

bool MoonDuino::SetFocuserSpeed(int speed)
{
    return setSpeed(speed);
}

IPState MoonDuino::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    if (speed != static_cast<int>(FocusSpeedN[0].value))
    {
        if (!setSpeed(speed))
            return IPS_ALERT;
    }

    // either go all the way in or all the way out
    // then use timer to stop
    if (dir == FOCUS_INWARD)
        MoveFocuser(0);
    else
        MoveFocuser(static_cast<uint32_t>(FocusMaxPosN[0].value));

    IEAddTimer(duration, &MoonDuino::timedMoveHelper, this);
    return IPS_BUSY;
}

void MoonDuino::timedMoveHelper(void * context)
{
    static_cast<MoonDuino *>(context)->timedMoveCallback();
}

void MoonDuino::timedMoveCallback()
{
    AbortFocuser();
    FocusAbsPosNP.s = IPS_IDLE;
    FocusRelPosNP.s = IPS_IDLE;
    FocusTimerNP.s = IPS_IDLE;
    FocusTimerN[0].value = 0;
    IDSetNumber(&FocusAbsPosNP, nullptr);
    IDSetNumber(&FocusRelPosNP, nullptr);
    IDSetNumber(&FocusTimerNP, nullptr);
}

IPState MoonDuino::MoveAbsFocuser(uint32_t targetTicks)
{
    targetPos = targetTicks;

    if (!MoveFocuser(targetPos))
        return IPS_ALERT;

    return IPS_BUSY;
}

IPState MoonDuino::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    // Clamp
    int32_t offset = ((dir == FOCUS_INWARD) ? -1 : 1) * static_cast<int32_t>(ticks);
    int32_t newPosition = FocusAbsPosN[0].value + offset;
    newPosition = std::max(static_cast<int32_t>(FocusAbsPosN[0].min), std::min(static_cast<int32_t>(FocusAbsPosN[0].max),
                           newPosition));

    if (!MoveFocuser(newPosition))
        return IPS_ALERT;

    FocusRelPosN[0].value = ticks;
    FocusRelPosNP.s       = IPS_BUSY;

    return IPS_BUSY;
}

void MoonDuino::TimerHit()
{
    if (!isConnected())
        return;

    bool rc = readPosition();
    if (rc)
    {
        if (fabs(lastPos - FocusAbsPosN[0].value) > 5)
        {
            IDSetNumber(&FocusAbsPosNP, nullptr);
            lastPos = static_cast<uint32_t>(FocusAbsPosN[0].value);
        }
    }

    rc = readTemperature();
    if (rc)
    {
        float const currentTemperature = TemperatureN[0].getValue();

        if (fabs(lastTemperature - currentTemperature) >= 0.5)
        {
            TemperatureN.apply();
            lastTemperature = static_cast<uint32_t>(currentTemperature);
        }
    }

    rc = readHumidity();
    if (rc)
    {
        float const currentHumidity = HumidityN[0].getValue();

        if (fabs(lastHumidity - currentHumidity) >= 0.5)
        {
            HumidityN.apply();
            lastHumidity = static_cast<uint32_t>(currentHumidity);
        }
    }

    if (FocusAbsPosNP.s == IPS_BUSY || FocusRelPosNP.s == IPS_BUSY)
    {
        if (!isMoving())
        {
            FocusAbsPosNP.s = IPS_OK;
            FocusRelPosNP.s = IPS_OK;
            IDSetNumber(&FocusAbsPosNP, nullptr);
            IDSetNumber(&FocusRelPosNP, nullptr);
            lastPos = static_cast<uint32_t>(FocusAbsPosN[0].value);
            LOG_INFO("Focuser reached requested position.");
        }
    }

    m_DustCap.TimerHit();

    SetTimer(getCurrentPollingPeriod());
}

bool MoonDuino::AbortFocuser()
{
    return sendCommand(":FQ#");
}

bool MoonDuino::saveConfigItems(FILE * fp)
{
    return Focuser::saveConfigItems(fp);
}

bool MoonDuino::sendCommand(const char * cmd, char * res, bool silent, int nret)
{
    if (m_SerialLock.try_lock_for(std::chrono::milliseconds(getCurrentPollingPeriod()/10)))
    {
        int nbytes_written = 0, nbytes_read = 0, rc = -1;

        tcflush(PortFD, TCIOFLUSH);

        LOGF_DEBUG("CMD <%s>", cmd);

        if (isSimulation())
        {
            static int cap_counter = 0, uncap_counter = 0;
            if (!strcasecmp(cmd, ":GV#"))
            {
                LOG_INFO("Firmware check.");
                res[0] = MOONDUINO_VERSION_MAJOR;
                res[1] = MOONDUINO_VERSION_MINOR;
            }
            else if (!strcasecmp(cmd, ":FQ#"))
            {
                LOG_INFO("Abort motors.");
            }
            else if (!strcasecmp(cmd, ":DC0#"))
            {
                LOG_INFO("Dustcap close request.");
                cap_counter = 10;
            }
            else if (!strcasecmp(cmd, ":DC1#"))
            {
                LOG_INFO("Dustcap open request.");
                uncap_counter = 10;
            }
            else if (!strcasecmp(cmd, ":DC2#"))
            {
                LOG_INFO("Dustcap current movement.");
                if (res)
                {
                    if (cap_counter == 0)
                        strcpy(res, "0#");
                    else if (uncap_counter == 0)
                        strcpy(res, "1#");
                    else
                    {
                        strcpy(res, "2#");
                        if (cap_counter > 0) cap_counter--;
                        if (uncap_counter > 0) uncap_counter--;
                    }
                }
            }
            else if (!strcasecmp(cmd, ":GH#"))
            {
                LOG_INFO("Focuser half-step state.");
                if (res)
                    strcpy(res, "00#");
            }
            else if (!strcasecmp(cmd, ":GT#"))
            {
                if (res)
                    strcpy(res, "14#");
            }
            else if (!strcasecmp(cmd, ":C#"))
            {
            }
            else if (!strcasecmp(cmd, ":GM#"))
            {
                if (res)
                    strcpy(res, "38#");
            }
            else if (!strcasecmp(cmd, ":GP#"))
            {
                //LOG_INFO("Focuser position.");
                if (res)
                    strcpy(res, "C350#");
            }
            else if (!strcasecmp(cmd, ":GD#"))
            {
                LOG_INFO("Focuser current speed.");
                if (res)
                    strcpy(res, "10#");
            }
            else if (!strcasecmp(cmd, ":GI#"))
            {
                LOG_INFO("Focuser current movement.");
                if (res)
                    strcpy(res, "0#");
            }
            else
            {
                LOGF_DEBUG("Unsupported <%s>", cmd);
                m_SerialLock.unlock();
                return false;
            }
        }
        else
        {
            if ((rc = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
            {
                char errstr[MAXRBUF] = { 0 };
                tty_error_msg(rc, errstr, MAXRBUF);
                if (!silent)
                    LOGF_ERROR("Serial write error: %s.", errstr);
                m_SerialLock.unlock();
                return false;
            }

            if (res == nullptr)
            {
                tcdrain(PortFD);
                m_SerialLock.unlock();
                return true;
            }

            // this is to handle the GV command which doesn't return the terminator, use the number of chars expected
            if (nret == 0)
            {
                rc = tty_nread_section(PortFD, res, ML_RES, ML_DEL, ML_TIMEOUT, &nbytes_read);
            }
            else
            {
                rc = tty_read(PortFD, res, nret, ML_TIMEOUT, &nbytes_read);
            }
            if (rc != TTY_OK)
            {
                char errstr[MAXRBUF] = { 0 };
                tty_error_msg(rc, errstr, MAXRBUF);
                if (!silent)
                    LOGF_ERROR("Serial read error: %s.", errstr);
                m_SerialLock.unlock();
                return false;
            }

            tcflush(PortFD, TCIOFLUSH);
        }

        LOGF_DEBUG("RES <%s>", res);
        m_SerialLock.unlock();
        return true;
    }
    else return false;
}

MoonDuino::DustCap::DustCap(MoonDuino* parent):
    INDI::DefaultDevice(),
    INDI::DustCapInterface(this),
    m_Parent(parent)
{};

const char * MoonDuino::DustCap::getDefaultName()
{
    return "MoonDuino DustCap";
}

bool MoonDuino::DustCap::initProperties()
{
    INDI::DefaultDevice::initProperties();

    // Status
    StatusT[0].fill("Cover", "", nullptr);
    StatusT[1].fill("Motor", "", nullptr);
    StatusT.fill(getDeviceName(), "Status", "",
                 MAIN_CONTROL_TAB, IP_RO, 60, IPS_IDLE);

    INDI::DustCapInterface::initProperties(MAIN_CONTROL_TAB, CAN_ABORT);
    return true;
}

bool MoonDuino::DustCap::updateProperties()
{
    bool result = INDI::DefaultDevice::updateProperties();
    
    if (isConnected())
        defineProperty(StatusT);
    else
        deleteProperty(StatusT);

    result &= INDI::DustCapInterface::updateProperties();

    return result;
}

IPState MoonDuino::DustCap::ParkCap()
{
    IPState e = IPS_BUSY;

    if (m_Parent == nullptr || m_Parent->sendCommand(":DC1#") == false)
        e = IPS_ALERT;

    StatusT[0].setText(e == IPS_BUSY ? "Capping" : "Serial error");
    StatusT.setState(e);
    StatusT.apply();

    return e;
}

IPState MoonDuino::DustCap::UnParkCap()
{
    IPState e = IPS_BUSY;

    if (m_Parent == nullptr || m_Parent->sendCommand(":DC0#") == false)
        e = IPS_ALERT;

    StatusT[0].setText(e == IPS_BUSY ? "Uncapping" : "Serial error");
    StatusT.setState(e);
    StatusT.apply();

    return e;
}

IPState MoonDuino::DustCap::AbortCap()
{
    return m_Parent->AbortFocuser() ? IPS_OK : IPS_ALERT;
}

void MoonDuino::DustCap::readState()
{
    char res[ML_RES] = {0};
    if (m_Parent->sendCommand(":DC2#", res))
    {
        if (strcmp(res,"2#"))
        {
            if (!strcmp(res, "0#")) // Unparked
            {
                ParkCapSP[CAP_PARK].setState(ISS_OFF);
                ParkCapSP[CAP_UNPARK].setState(ISS_ON);
                StatusT[0].setText("Uncapped");
            }
            else if (!strcmp(res, "1#")) // Parked
            {
                ParkCapSP[CAP_PARK].setState(ISS_ON);
                ParkCapSP[CAP_UNPARK].setState(ISS_OFF);
                StatusT[0].setText("Capped");
            }
            if (IPS_BUSY == ParkCapSP.getState()) // Busy will change to ok now
                LOG_INFO("DustCap reached requested position.");
            ParkCapSP.setState(IPS_OK);
            ParkCapSP.apply();
            StatusT.setState(IPS_OK);
            StatusT.apply();
        }
        else
        {
            ParkCapSP.setState(IPS_BUSY);
            ParkCapSP.apply();
        }
    }
}

void MoonDuino::DustCap::TimerHit()
{
    IPState const pc = ParkCapSP.getState();
    if (IPS_BUSY == pc || IPS_IDLE == pc)
        readState();
}

bool MoonDuino::DustCap::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if(processSwitch(dev, name, states, names, n))
        return true;

    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}
