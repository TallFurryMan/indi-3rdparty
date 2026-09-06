/*******************************************************************************
  ESPHome Observatory Controller
  Copyright(c) 2026. All rights reserved.

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
*******************************************************************************/

#include "esphome_observatory.h"

#include "connectionplugins/connectiontcp.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <memory>
#include <sys/select.h>
#include <unistd.h>

static std::unique_ptr<ESPHomeObservatory> esphomeObservatory(new ESPHomeObservatory());

namespace
{
std::string lowerASCII(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char oneChar)
    {
        return static_cast<char>(std::tolower(oneChar));
    });
    return value;
}

bool contains(const std::string &haystack, const std::string &needle)
{
    return haystack.find(needle) != std::string::npos;
}

std::string trimASCII(const std::string &value)
{
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char oneChar)
    {
        return std::isspace(oneChar) != 0;
    });

    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char oneChar)
    {
        return std::isspace(oneChar) != 0;
    }).base();

    return first >= last ? std::string() : std::string(first, last);
}

INDI::ESPHome::EntityType endpointTypeFromPrefix(const std::string &prefix)
{
    const auto type = lowerASCII(prefix);
    if (type == "binary_sensor")
        return INDI::ESPHome::EntityType::BinarySensor;
    if (type == "button")
        return INDI::ESPHome::EntityType::Button;
    if (type == "cover")
        return INDI::ESPHome::EntityType::Cover;
    if (type == "switch")
        return INDI::ESPHome::EntityType::Switch;

    return INDI::ESPHome::EntityType::Unknown;
}
}

ESPHomeObservatory::ESPHomeObservatory()
    : INDI::ESPHomeInterface(this),
      INDI::InputInterface(this),
      INDI::OutputInterface(this),
      INDI::WeatherInterface(this)
{
    setDomeConnection(INDI::Dome::CONNECTION_NONE);
    SetDomeCapability(DOME_CAN_PARK);
    setVersion(1, 0);
}

const char *ESPHomeObservatory::getDefaultName()
{
    return "ESPHome Observatory";
}

bool ESPHomeObservatory::initProperties()
{
    INDI::Dome::initProperties();
    SetParkDataType(PARK_NONE);

    setDriverInterface(getDriverInterface() | AUX_INTERFACE | INPUT_INTERFACE | OUTPUT_INTERFACE | WEATHER_INTERFACE);

    INDI::InputInterface::initProperties(MAIN_CONTROL_TAB, MAX_DIGITAL_INPUTS, 0, "Binary Sensor");
    INDI::OutputInterface::initProperties(MAIN_CONTROL_TAB, MAX_DIGITAL_OUTPUTS, "Switch");

    INDI::WeatherInterface::initProperties("Environment", "Environment");
    addParameter("WEATHER_TEMPERATURE", "Temperature (C)", -15, 35, 15);
    addParameter("WEATHER_HUMIDITY", "Humidity %", 0, 100, 15);
    addParameter("WEATHER_DEWPOINT", "Dew Point (C)", -25, 35, 15);
    addParameter("WEATHER_PRESSURE", "Pressure (hPa)", 800, 1100, 10);
    addParameter("WEATHER_ILLUMINANCE", "Illuminance (lx)", 0, 200000, 10);
    addParameter("WEATHER_WIND_SPEED", "Wind Speed (m/s)", 0, 20, 20);
    addParameter("WEATHER_RAIN", "Rain", 0, 0.5, 20);
    setCriticalParameter("WEATHER_HUMIDITY");
    setCriticalParameter("WEATHER_WIND_SPEED");
    setCriticalParameter("WEATHER_RAIN");

    LegacyPasswordTP[0].fill("PASSWORD", "Password", "");
    LegacyPasswordTP.fill(getDeviceName(), "ESPHOME_API_PASSWORD", "Legacy API Password", CONNECTION_TAB, IP_RW, 60, IPS_IDLE);
    LegacyPasswordTP.load();

    DeviceInfoTP[0].fill("NAME", "Name", "");
    DeviceInfoTP[1].fill("FRIENDLY_NAME", "Friendly Name", "");
    DeviceInfoTP[2].fill("VERSION", "Version", "");
    DeviceInfoTP[3].fill("MODEL", "Model", "");
    DeviceInfoTP[4].fill("MAC_ADDRESS", "MAC", "");
    DeviceInfoTP[5].fill("ENTITIES", "Entities", "0");
    DeviceInfoTP.fill(getDeviceName(), "ESPHOME_DEVICE_INFO", "ESPHome", INFO_TAB, IP_RO, 60, IPS_IDLE);

    DomeEndpointTP[DOME_COVER].fill("COVER", "Cover", "");
    DomeEndpointTP[DOME_PARK_COMMAND].fill("PARK_COMMAND", "Park/Close", "");
    DomeEndpointTP[DOME_UNPARK_COMMAND].fill("UNPARK_COMMAND", "Unpark/Open", "");
    DomeEndpointTP[DOME_PARKED_STATE].fill("PARKED_STATE", "Parked Sensor", "");
    DomeEndpointTP[DOME_UNPARKED_STATE].fill("UNPARKED_STATE", "Unparked Sensor", "");
    DomeEndpointTP.fill(getDeviceName(), "ESPHOME_DOME_ENDPOINTS", "Dome Endpoints", OPTIONS_TAB, IP_RW, 60, IPS_IDLE);
    DomeEndpointTP.load();

    tcpConnection = new Connection::TCP(this);
    tcpConnection->setDefaultHost("esphome.local");
    tcpConnection->setDefaultPort(DEFAULT_ESPHOME_PORT);
    tcpConnection->setConnectionType(Connection::TCP::TYPE_TCP);
    tcpConnection->registerHandshake([this]()
    {
        return Handshake();
    });
    registerConnection(tcpConnection);

    addAuxControls();
    setDefaultPollingPeriod(500);

    return true;
}

void ESPHomeObservatory::ISGetProperties(const char *dev)
{
    INDI::Dome::ISGetProperties(dev);
    defineProperty(LegacyPasswordTP);
    defineProperty(DomeEndpointTP);
}

bool ESPHomeObservatory::updateProperties()
{
    INDI::Dome::updateProperties();
    INDI::InputInterface::updateProperties();
    INDI::OutputInterface::updateProperties();
    INDI::WeatherInterface::updateProperties();

    if (isConnected())
    {
        InitPark();
        updateDeviceInfoProperty();
        defineProperty(DeviceInfoTP);
        SetTimer(getCurrentPollingPeriod());
    }
    else
    {
        deleteProperty(DeviceInfoTP);
    }

    return true;
}

bool ESPHomeObservatory::Disconnect()
{
    disconnectESPHome();
    PortFD = -1;
    return INDI::DefaultDevice::Disconnect();
}

bool ESPHomeObservatory::Handshake()
{
    resetBindings();
    resetDomeEndpoints();
    PortFD = tcpConnection->getPortFD();

    if (!connectESPHome(PortFD, "INDI ESPHome Observatory", LegacyPasswordTP[0].getText()))
        return false;

    if (!subscribeESPHomeStates())
        return false;

    resolveDomeEndpoints();
    updateDeviceInfoProperty();
    return true;
}

void ESPHomeObservatory::TimerHit()
{
    if (!isConnected())
        return;

    for (uint8_t i = 0; i < MAX_FRAMES_PER_TIMER && hasPendingESPHomeData(); ++i)
    {
        if (!processESPHomeState())
            break;
    }

    SetTimer(getCurrentPollingPeriod());
}

bool ESPHomeObservatory::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && !strcmp(dev, getDeviceName()))
    {
        if (LegacyPasswordTP.isNameMatch(name))
        {
            LegacyPasswordTP.update(texts, names, n);
            LegacyPasswordTP.setState(IPS_OK);
            LegacyPasswordTP.apply();
            saveConfig(LegacyPasswordTP);
            return true;
        }

        if (DomeEndpointTP.isNameMatch(name))
        {
            DomeEndpointTP.update(texts, names, n);
            DomeEndpointTP.setState(isConnected() && !resolveDomeEndpoints() ? IPS_ALERT : IPS_OK);
            DomeEndpointTP.apply();
            saveConfig(DomeEndpointTP);
            return true;
        }
    }

    if (INDI::InputInterface::processText(dev, name, texts, names, n))
        return true;

    if (INDI::OutputInterface::processText(dev, name, texts, names, n))
        return true;

    return INDI::Dome::ISNewText(dev, name, texts, names, n);
}

bool ESPHomeObservatory::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (INDI::OutputInterface::processSwitch(dev, name, states, names, n))
        return true;

    if (INDI::WeatherInterface::processSwitch(dev, name, states, names, n))
        return true;

    return INDI::Dome::ISNewSwitch(dev, name, states, names, n);
}

bool ESPHomeObservatory::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (INDI::OutputInterface::processNumber(dev, name, values, names, n))
        return true;

    if (INDI::WeatherInterface::processNumber(dev, name, values, names, n))
        return true;

    return INDI::Dome::ISNewNumber(dev, name, values, names, n);
}

bool ESPHomeObservatory::saveConfigItems(FILE *fp)
{
    INDI::Dome::saveConfigItems(fp);
    INDI::InputInterface::saveConfigItems(fp);
    INDI::OutputInterface::saveConfigItems(fp);
    INDI::WeatherInterface::saveConfigItems(fp);
    LegacyPasswordTP.save(fp);
    DomeEndpointTP.save(fp);
    return true;
}

IPState ESPHomeObservatory::Move(DomeDirection dir, DomeMotionCommand operation)
{
    if (operation == MOTION_STOP)
    {
        if (m_DomeCoverKey != 0 && commandESPHomeCover(m_DomeCoverKey, INDI::ESPHome::CoverCommand::Stop))
            return IPS_OK;

        return IPS_ALERT;
    }

    return dir == DOME_CCW ? Park() : UnPark();
}

IPState ESPHomeObservatory::Park()
{
    return commandDome(true);
}

IPState ESPHomeObservatory::UnPark()
{
    return commandDome(false);
}

bool ESPHomeObservatory::UpdateDigitalInputs()
{
    return true;
}

bool ESPHomeObservatory::UpdateAnalogInputs()
{
    return true;
}

bool ESPHomeObservatory::UpdateDigitalOutputs()
{
    return true;
}

bool ESPHomeObservatory::CommandOutput(uint32_t index, OutputState command)
{
    if (index >= m_OutputKeys.size() || m_OutputKeys[index] == 0)
    {
        LOGF_ERROR("No ESPHome switch is bound to output %u.", index + 1);
        return false;
    }

    return commandESPHomeSwitch(m_OutputKeys[index], command == OutputState::On);
}

IPState ESPHomeObservatory::updateWeather()
{
    return IPS_OK;
}

void ESPHomeObservatory::ESPHomeDeviceInfoAvailable(const INDI::ESPHome::DeviceInfo &info)
{
    INDI_UNUSED(info);
    updateDeviceInfoProperty();
}

void ESPHomeObservatory::ESPHomeEntityDiscovered(const INDI::ESPHome::EntityInfo &entity)
{
    switch (entity.type)
    {
        case INDI::ESPHome::EntityType::Switch:
            bindOutputEntity(entity);
            break;

        case INDI::ESPHome::EntityType::BinarySensor:
            bindInputEntity(entity);
            break;

        case INDI::ESPHome::EntityType::Sensor:
            bindWeatherEntity(entity);
            break;

        default:
            break;
    }
}

void ESPHomeObservatory::ESPHomeStateChanged(const INDI::ESPHome::State &state)
{
    if (state.type == INDI::ESPHome::EntityType::Cover && state.key == m_DomeCoverKey)
    {
        if (state.position <= 0.01F)
            syncDomeState(true);
        else if (state.position >= 0.99F)
            syncDomeState(false);
        return;
    }

    if (state.type == INDI::ESPHome::EntityType::Switch)
    {
        const int index = outputIndexForKey(state.key);
        if (index < 0)
            return;

        auto &output = DigitalOutputsSP[static_cast<size_t>(index)];
        const auto oldState = output.findOnSwitchIndex();
        const auto newState = state.boolValue ? OutputInterface::On : OutputInterface::Off;
        if (oldState != newState)
        {
            output.reset();
            output[OutputInterface::Off].setState(state.boolValue ? ISS_OFF : ISS_ON);
            output[OutputInterface::On].setState(state.boolValue ? ISS_ON : ISS_OFF);
            output.setState(IPS_OK);
            output.apply();
        }
        return;
    }

    if (state.type == INDI::ESPHome::EntityType::BinarySensor)
    {
        if (state.key == m_ParkedStateKey && state.boolValue)
            syncDomeState(true);
        else if (state.key == m_UnparkedStateKey && state.boolValue)
            syncDomeState(false);

        const int index = inputIndexForKey(state.key);
        if (index < 0)
            return;

        auto &input = DigitalInputsSP[static_cast<size_t>(index)];
        const auto oldState = input.findOnSwitchIndex();
        const auto newState = state.boolValue ? InputInterface::On : InputInterface::Off;
        if (oldState != newState)
        {
            input.reset();
            input[InputInterface::Off].setState(state.boolValue ? ISS_OFF : ISS_ON);
            input[InputInterface::On].setState(state.boolValue ? ISS_ON : ISS_OFF);
            input.setState(IPS_OK);
            input.apply();
        }
        return;
    }

    if (state.type == INDI::ESPHome::EntityType::Sensor && !state.missingState)
    {
        const auto binding = weatherBindingForKey(state.key);
        if (binding == nullptr)
            return;

        setParameterValue(binding->parameter, state.floatValue);
        if (syncCriticalParameters())
            critialParametersLP.apply();

        ParametersNP.setState(IPS_OK);
        ParametersNP.apply();
    }
}

void ESPHomeObservatory::resetBindings()
{
    m_InputKeys.fill(0);
    m_OutputKeys.fill(0);
    m_WeatherBindings.clear();
}

void ESPHomeObservatory::resetDomeEndpoints()
{
    m_DomeCoverKey = 0;
    m_ParkCommand = {};
    m_UnparkCommand = {};
    m_ParkedStateKey = 0;
    m_UnparkedStateKey = 0;
}

bool ESPHomeObservatory::resolveDomeEndpoints()
{
    resetDomeEndpoints();
    bool resolved = true;

    const auto cover = findDomeEndpoint(DomeEndpointTP[DOME_COVER].getText(),
                                        {INDI::ESPHome::EntityType::Cover});
    if (cover != nullptr)
        m_DomeCoverKey = cover->key;
    else if (!trimASCII(DomeEndpointTP[DOME_COVER].getText()).empty())
        resolved = false;

    const auto parkCommand = findDomeEndpoint(DomeEndpointTP[DOME_PARK_COMMAND].getText(),
                             {INDI::ESPHome::EntityType::Switch, INDI::ESPHome::EntityType::Button});
    if (parkCommand != nullptr)
        m_ParkCommand = {parkCommand->type, parkCommand->key};
    else if (!trimASCII(DomeEndpointTP[DOME_PARK_COMMAND].getText()).empty())
        resolved = false;

    const auto unparkCommand = findDomeEndpoint(DomeEndpointTP[DOME_UNPARK_COMMAND].getText(),
                               {INDI::ESPHome::EntityType::Switch, INDI::ESPHome::EntityType::Button});
    if (unparkCommand != nullptr)
        m_UnparkCommand = {unparkCommand->type, unparkCommand->key};
    else if (!trimASCII(DomeEndpointTP[DOME_UNPARK_COMMAND].getText()).empty())
        resolved = false;

    const auto parkedState = findDomeEndpoint(DomeEndpointTP[DOME_PARKED_STATE].getText(),
                             {INDI::ESPHome::EntityType::BinarySensor});
    if (parkedState != nullptr)
        m_ParkedStateKey = parkedState->key;
    else if (!trimASCII(DomeEndpointTP[DOME_PARKED_STATE].getText()).empty())
        resolved = false;

    const auto unparkedState = findDomeEndpoint(DomeEndpointTP[DOME_UNPARKED_STATE].getText(),
                               {INDI::ESPHome::EntityType::BinarySensor});
    if (unparkedState != nullptr)
        m_UnparkedStateKey = unparkedState->key;
    else if (!trimASCII(DomeEndpointTP[DOME_UNPARKED_STATE].getText()).empty())
        resolved = false;

    DomeEndpointTP.setState(resolved ? IPS_OK : IPS_ALERT);
    if (isConnected())
        DomeEndpointTP.apply();

    return resolved;
}

void ESPHomeObservatory::updateDeviceInfoProperty()
{
    const auto &info = getESPHomeDeviceInfo();
    DeviceInfoTP[0].setText(info.name);
    DeviceInfoTP[1].setText(info.friendlyName);
    DeviceInfoTP[2].setText(info.esphomeVersion);
    DeviceInfoTP[3].setText(info.model);
    DeviceInfoTP[4].setText(info.macAddress);
    DeviceInfoTP[5].setText(std::to_string(getESPHomeEntities().size()));
    DeviceInfoTP.setState(isESPHomeConnected() ? IPS_OK : IPS_IDLE);
}

bool ESPHomeObservatory::hasPendingESPHomeData() const
{
    if (PortFD < 0)
        return false;

    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(PortFD, &readSet);

    timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const auto result = select(PortFD + 1, &readSet, nullptr, nullptr, &timeout);
    return result > 0 && FD_ISSET(PortFD, &readSet);
}

IPState ESPHomeObservatory::commandDome(bool park)
{
    if (park && INDI::Dome::isLocked())
    {
        LOG_INFO("Cannot park ESPHome dome when mount is locking. See Mount Policy in Options tab.");
        return IPS_ALERT;
    }

    if (!resolveDomeEndpoints())
    {
        LOG_ERROR("ESPHome dome endpoints are not resolved.");
        return IPS_ALERT;
    }

    if (m_DomeCoverKey != 0)
    {
        const auto command = park ? INDI::ESPHome::CoverCommand::Close : INDI::ESPHome::CoverCommand::Open;
        if (!commandESPHomeCover(m_DomeCoverKey, command))
            return IPS_ALERT;

        LOGF_INFO("ESPHome dome %s command sent to cover endpoint.", park ? "park" : "unpark");
        return IPS_BUSY;
    }

    const auto &endpoint = park ? m_ParkCommand : m_UnparkCommand;
    if (endpoint.key == 0)
    {
        LOGF_ERROR("No ESPHome %s endpoint is configured.", park ? "park/close" : "unpark/open");
        return IPS_ALERT;
    }

    if (!commandDomeEndpoint(endpoint))
        return IPS_ALERT;

    LOGF_INFO("ESPHome dome %s command sent.", park ? "park" : "unpark");
    return hasDomeStateFeedback(park) ? IPS_BUSY : IPS_OK;
}

bool ESPHomeObservatory::commandDomeEndpoint(const CommandEndpoint &endpoint)
{
    switch (endpoint.type)
    {
        case INDI::ESPHome::EntityType::Switch:
            return commandESPHomeSwitch(endpoint.key, true);

        case INDI::ESPHome::EntityType::Button:
            return commandESPHomeButton(endpoint.key);

        default:
            return false;
    }
}

bool ESPHomeObservatory::hasDomeStateFeedback(bool park) const
{
    if (m_DomeCoverKey != 0)
        return true;

    return park ? m_ParkedStateKey != 0 : m_UnparkedStateKey != 0;
}

void ESPHomeObservatory::syncDomeState(bool parked)
{
    if (isParked() == parked && ParkSP.getState() == IPS_OK)
        return;

    LOGF_INFO("ESPHome dome is %s.", parked ? "parked" : "unparked");
    SetParked(parked);
}

const INDI::ESPHome::EntityInfo *ESPHomeObservatory::findDomeEndpoint(const std::string &endpoint,
        const std::vector<INDI::ESPHome::EntityType> &allowedTypes) const
{
    auto value = trimASCII(endpoint);
    if (value.empty())
        return nullptr;

    const auto separator = value.find(':');
    if (separator != std::string::npos)
    {
        const auto requestedType = endpointTypeFromPrefix(trimASCII(value.substr(0, separator)));
        value = trimASCII(value.substr(separator + 1));

        if (value.empty() || requestedType == INDI::ESPHome::EntityType::Unknown)
            return nullptr;

        if (std::find(allowedTypes.begin(), allowedTypes.end(), requestedType) == allowedTypes.end())
            return nullptr;

        return findESPHomeEntity(requestedType, value);
    }

    const auto type = std::find_if(allowedTypes.begin(), allowedTypes.end(), [this, &value](const auto oneType)
    {
        return findESPHomeEntity(oneType, value) != nullptr;
    });

    return type == allowedTypes.end() ? nullptr : findESPHomeEntity(*type, value);
}

std::string ESPHomeObservatory::entityLabel(const INDI::ESPHome::EntityInfo &entity)
{
    if (!entity.name.empty())
        return entity.name;

    if (!entity.objectId.empty())
        return entity.objectId;

    return INDI::ESPHome::NativeAPIClient::entityTypeName(entity.type);
}

std::string ESPHomeObservatory::weatherParameterForEntity(const INDI::ESPHome::EntityInfo &entity)
{
    const auto deviceClass = lowerASCII(entity.deviceClass);
    const auto objectId = lowerASCII(entity.objectId);
    const auto name = lowerASCII(entity.name);
    const auto unit = lowerASCII(entity.unitOfMeasurement);
    const auto combined = deviceClass + " " + objectId + " " + name + " " + unit;

    if (contains(combined, "dew"))
        return "WEATHER_DEWPOINT";
    if (contains(combined, "humid"))
        return "WEATHER_HUMIDITY";
    if (contains(combined, "temp") || unit == "c" || unit == "f")
        return "WEATHER_TEMPERATURE";
    if (contains(combined, "pressure") || contains(unit, "hpa") || contains(unit, "mbar"))
        return "WEATHER_PRESSURE";
    if (contains(combined, "illuminance") || contains(combined, "lux") || unit == "lx")
        return "WEATHER_ILLUMINANCE";
    if (contains(combined, "wind") && (contains(combined, "speed") || contains(unit, "m/s") || contains(unit, "km/h")))
        return "WEATHER_WIND_SPEED";
    if (contains(combined, "rain") || contains(combined, "precipitation"))
        return "WEATHER_RAIN";

    return {};
}

void ESPHomeObservatory::bindOutputEntity(const INDI::ESPHome::EntityInfo &entity)
{
    const auto slot = std::find(m_OutputKeys.begin(), m_OutputKeys.end(), 0);
    if (slot == m_OutputKeys.end())
        return;

    const auto index = static_cast<size_t>(std::distance(m_OutputKeys.begin(), slot));
    m_OutputKeys[index] = entity.key;

    const auto label = entityLabel(entity);
    DigitalOutputsSP[index].setLabel(label);
    DigitalOutputLabelsTP[index].setText(label);
    PulseDurationNP[index].setLabel(label);
}

void ESPHomeObservatory::bindInputEntity(const INDI::ESPHome::EntityInfo &entity)
{
    const auto slot = std::find(m_InputKeys.begin(), m_InputKeys.end(), 0);
    if (slot == m_InputKeys.end())
        return;

    const auto index = static_cast<size_t>(std::distance(m_InputKeys.begin(), slot));
    m_InputKeys[index] = entity.key;

    const auto label = entityLabel(entity);
    DigitalInputsSP[index].setLabel(label);
    DigitalInputLabelsTP[index].setText(label);
}

void ESPHomeObservatory::bindWeatherEntity(const INDI::ESPHome::EntityInfo &entity)
{
    const auto parameter = weatherParameterForEntity(entity);
    if (parameter.empty())
        return;

    const auto duplicate = std::find_if(m_WeatherBindings.begin(), m_WeatherBindings.end(), [&parameter](const auto & binding)
    {
        return binding.parameter == parameter;
    });

    if (duplicate == m_WeatherBindings.end())
        m_WeatherBindings.push_back({entity.key, parameter});
}

int ESPHomeObservatory::outputIndexForKey(uint32_t key) const
{
    const auto it = std::find(m_OutputKeys.begin(), m_OutputKeys.end(), key);
    return it == m_OutputKeys.end() ? -1 : static_cast<int>(std::distance(m_OutputKeys.begin(), it));
}

int ESPHomeObservatory::inputIndexForKey(uint32_t key) const
{
    const auto it = std::find(m_InputKeys.begin(), m_InputKeys.end(), key);
    return it == m_InputKeys.end() ? -1 : static_cast<int>(std::distance(m_InputKeys.begin(), it));
}

const ESPHomeObservatory::WeatherBinding *ESPHomeObservatory::weatherBindingForKey(uint32_t key) const
{
    const auto it = std::find_if(m_WeatherBindings.begin(), m_WeatherBindings.end(), [key](const auto & binding)
    {
        return binding.key == key;
    });

    return it == m_WeatherBindings.end() ? nullptr : &(*it);
}
