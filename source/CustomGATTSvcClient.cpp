#include "movesense.h"

#include "CustomGATTSvcClient.h"
#include "common/core/debug.h"
#include "oswrapper/thread.h"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"
#include <meas_temp/resources.h>
#include "meas_acc/resources.h"
#include <meas_imu/resources.h>
#include "whiteboard/builtinTypes/UnknownStructure.h"

#define SPAN 4000.0
#define NR_SIZE 65535.0

const char* const CustomGATTSvcClient::LAUNCHABLE_NAME = "CstGattS";

// Nytt
const size_t BLINK_PERIOD_MS = 800;
whiteboard::TimerId mTimer;
whiteboard::ResourceId	mMeasAccResourceId;

const uint16_t measCharUUID16 = 0x2A1C;
const uint16_t intervalCharUUID16 = 0x2A21;

const int DEFAULT_MEASUREMENT_INTERVAL_SECS = 1;

CustomGATTSvcClient::CustomGATTSvcClient()
    : ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
      LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION),
      mMeasIntervalSecs(DEFAULT_MEASUREMENT_INTERVAL_SECS),
      mTemperatureSvcHandle(0),
      mMeasCharHandle(0),
      mIntervalCharHandle(0),
      mIntervalCharResource(whiteboard::ID_INVALID_RESOURCE),
      mMeasCharResource(whiteboard::ID_INVALID_RESOURCE),
      mMeasurementTimer(whiteboard::ID_INVALID_TIMER)
{
    mTimer = whiteboard::ID_INVALID_TIMER;
}

CustomGATTSvcClient::~CustomGATTSvcClient()
{
}

bool CustomGATTSvcClient::initModule()
{
    mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void CustomGATTSvcClient::deinitModule()
{
    mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool CustomGATTSvcClient::startModule()
{
    mModuleState = WB_RES::ModuleStateValues::STARTED;

    // Configure custom gatt service
    configGattSvc();
    //mTimer = startTimer(BLINK_PERIOD_MS, true);
    //asyncSubscribe(WB_RES::LOCAL::MEAS_IMU9_SAMPLERATE::ID, AsyncRequestOptions::Empty, 26);
    whiteboard::ResourceId	mMeasAccResourceId;
    wb::Result result = getResource("Meas/Acc/52", mMeasAccResourceId);
    if (!wb::RETURN_OKC(result))
    {
        return whiteboard::HTTP_CODE_BAD_REQUEST;
    }
    result = asyncSubscribe(mMeasAccResourceId, AsyncRequestOptions::Empty);

    return true;
}

/** @see whiteboard::ILaunchableModule::startModule */
void CustomGATTSvcClient::stopModule()
{
    // Stop timer if running
    if (mMeasurementTimer != whiteboard::ID_INVALID_TIMER)
        stopTimer(mMeasurementTimer);

    // Nytt
    if (mTimer != whiteboard::ID_INVALID_TIMER)
        stopTimer(mTimer);

    mTimer = whiteboard::ID_INVALID_TIMER;

    // Unsubscribe if needed
    if (mIntervalCharResource != whiteboard::ID_INVALID_RESOURCE)
        asyncUnsubscribe(mIntervalCharResource);
    if (mMeasCharResource != whiteboard::ID_INVALID_RESOURCE)
        asyncUnsubscribe(mMeasCharResource);

    mMeasurementTimer = whiteboard::ID_INVALID_TIMER;
}

#include "ui_ind/resources.h"
void CustomGATTSvcClient::configGattSvc() {
    WB_RES::GattSvc customGattSvc;
    WB_RES::GattChar characteristics[2];
    WB_RES::GattChar &measChar = characteristics[0];
    WB_RES::GattChar &intervalChar = characteristics[1];

    const uint16_t healthThermometerSvcUUID16 = 0x1809;
    
    // Define the CMD characteristics
    //WB_RES::GattProperty measCharProp = WB_RES::GattProperty::INDICATE;
    WB_RES::GattProperty measCharProp[3] = {WB_RES::GattProperty::NOTIFY, WB_RES::GattProperty::READ, WB_RES::GattProperty::WRITE};
    WB_RES::GattProperty intervalCharProps[2] = {WB_RES::GattProperty::READ, WB_RES::GattProperty::WRITE};

    //measChar.props = whiteboard::MakeArray<WB_RES::GattProperty>( &measCharProp, 1);
    measChar.props = whiteboard::MakeArray<WB_RES::GattProperty>( measCharProp, 3);
    measChar.uuid = whiteboard::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&measCharUUID16), 2);

    intervalChar.props = whiteboard::MakeArray<WB_RES::GattProperty>( intervalCharProps, 2);
    intervalChar.uuid = whiteboard::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&intervalCharUUID16), 2);
    intervalChar.initial_value = whiteboard::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&mMeasIntervalSecs), 2);

    // Combine chars to service
    customGattSvc.uuid = whiteboard::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&healthThermometerSvcUUID16), 2);
    customGattSvc.chars = whiteboard::MakeArray<WB_RES::GattChar>(characteristics, 2);

    // Create custom service
    asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions::Empty, customGattSvc);
}

void CustomGATTSvcClient::onTimer(whiteboard::TimerId timerId)
{
    DEBUGLOG("CustomGATTSvcClient::onTimer");

    if (timerId == mTimer)
    {
        uint16_t indicationType = 2; // SHORT_VISUAL_INDICATION, defined in ui/ind.yaml
        // Make PUT request to trigger led blink
        asyncPut(WB_RES::LOCAL::UI_IND_VISUAL::ID, AsyncRequestOptions::Empty, indicationType);
        ////////////
    }

    // Take temperature reading
    //asyncGet(WB_RES::LOCAL::MEAS_TEMP(), NULL);
    //asyncSubscribe(WB_RES::LOCAL::MEAS_IMU9_SAMPLERATE(), AsyncRequestOptions::Empty, 13);
}

#include <math.h>
#include <cstdint>

static void floatToFLOAT(float value, uint8_t* bufferOut)
{
    bool bNegative = (value < 0.0f);
    if (bNegative) value = fabs(value);

    // Calc exponent
    int exponent = ceil(log10(value));
    DEBUGLOG("exponent: %d", exponent);

    float mantissa = value * pow(10.0, -exponent + 6); // Use up to 10^6 in mantissa (+-1000,000)
    int32_t mantInt24 = (int32_t)round(mantissa);
    if (bNegative)
        mantInt24 = -mantInt24;
    DEBUGLOG("mantInt24: %d", mantInt24);

    bufferOut[0] = (uint8_t)(mantInt24 & 0xff);
    bufferOut[1] = (uint8_t)((mantInt24>>8) & 0xff);
    bufferOut[2] = (uint8_t)((mantInt24>>16) & 0xff);
    bufferOut[3] = (int8_t)exponent-6;
}

void CustomGATTSvcClient::onGetResult(whiteboard::RequestId requestId, whiteboard::ResourceId resourceId, whiteboard::Result resultCode, const whiteboard::Value& rResultData)
{
    DEBUGLOG("CustomGATTSvcClient::onGetResult");
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            const WB_RES::GattSvc &svc = rResultData.convertTo<const WB_RES::GattSvc &>();
            for (size_t i=0; i<svc.chars.size(); i++) {
                const WB_RES::GattChar &c = svc.chars[i];
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[0]));
                
                if(uuid16 == measCharUUID16)
                    mMeasCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                else if(uuid16 == intervalCharUUID16)
                    mIntervalCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
            }

            if (!mIntervalCharHandle || !mMeasCharHandle)
            {
                DEBUGLOG("ERROR: Not all chars were configured!");
                return;
            }

            char pathBuffer[32]= {'\0'};
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mTemperatureSvcHandle, mIntervalCharHandle);
            getResource(pathBuffer, mIntervalCharResource);
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mTemperatureSvcHandle, mMeasCharHandle);
            getResource(pathBuffer, mMeasCharResource);

            // Subscribe to listen to intervalChar notifications (someone writes new value to intervalChar) 
            asyncSubscribe(mIntervalCharResource, AsyncRequestOptions::Empty);
            // Subscribe to listen to measChar notifications (someone enables/disables the INDICATE characteristic) 
            asyncSubscribe(mMeasCharResource, AsyncRequestOptions::Empty);
        }
        break;

        case WB_RES::LOCAL::MEAS_TEMP::LID:
        {
            // Temperature result or error
            if (resultCode == whiteboard::HTTP_CODE_OK)
            {
                WB_RES::TemperatureValue value = rResultData.convertTo<WB_RES::TemperatureValue>();
                float temperature = value.measurement;

                // Convert K to C
                temperature -= 273.15;

                // Return data
                uint8_t buffer[5]; // 1 byte or flags, 4 for FLOAT "in Celsius" value
                buffer[0]=0;

                // convert normal float to IEEE-11073 "medical" FLOAT type into buffer
                floatToFLOAT(temperature, &buffer[1]);

                // Write the result to measChar. This results INDICATE to be triggered in GATT service
                WB_RES::Characteristic newMeasCharValue;
                newMeasCharValue.bytes = whiteboard::MakeArray<uint8_t>(buffer, sizeof(buffer));
                asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty, mTemperatureSvcHandle, mMeasCharHandle, newMeasCharValue);
            }
        }
        break;
    }
}

uint16_t convertFloatTo16bitInt (float &num) {
    return (uint16_t) round(NR_SIZE * (num + SPAN/2) / SPAN);
}


#include <string>
void CustomGATTSvcClient::onNotify(whiteboard::ResourceId resourceId, const whiteboard::Value& value, const whiteboard::ParameterList& rParameters)
{
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::LID:
        {
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::SUBSCRIBE::ParameterListRef parameterRef(rParameters);
            if (parameterRef.getCharHandle() == mIntervalCharHandle) 
            {
                const WB_RES::Characteristic &charValue = value.convertTo<const WB_RES::Characteristic &>();
                uint8_t commandValueLower = *reinterpret_cast<const uint8_t*>(&charValue.bytes[0]);
                uint8_t commandValueHigher = *reinterpret_cast<const uint8_t*>(&charValue.bytes[1]);

                if (commandValueHigher == 255 && commandValueLower == 255) {
                    // Blink to acknowledge command (SHORT_VISUAL_INDICATION)
                    uint16_t indicationType = 2;
                    asyncPut(WB_RES::LOCAL::UI_IND_VISUAL::ID, AsyncRequestOptions::Empty, indicationType);
                }

                // uint16_t commandValue = ((uint16_t)commandValueHigher << 8) | commandValueLower;
                uint16_t commandValue = ((uint16_t)commandValueLower << 8) | commandValueHigher;

                uint16_t commandValueType = commandValue / 100;

                switch(commandValueType) {
                    case 0 :
                        // General command settings
                        switch(commandValue % 100) {
                            case 0:
                                // Shutdown - i.e. stop subscribing to chosen resource.
                                break;
                            case 1:
                                // Start subscribing - i.e. to the chosen resource.
                                std::string name = "John";
                                int age = 21;
                                std::string result;

                                // 2. with C++11
                                result = name + std::to_string(age);
                            default:
                                // Not valid command. Do nothing.
                                break;
                        }
                        break;
                    case 1 :
                        // Accelerometer
                        if (commandValue % 100 > 9) {
                            // Record data with DataLogger <- implement later
                        } else {
                            /*uint16_t sampleRateValue = commandValue % 10;
                            uint16_t sampleRate = 13 * (2^sampleRateValue);
                            wb::Result resultAcc = asyncUnsubscribe(WB_RES::LOCAL::MEAS_ACC_SAMPLERATE::ID);
                            wb::Result resultImu9 = asyncUnsubscribe(WB_RES::LOCAL::MEAS_IMU9_SAMPLERATE::ID);
                            wb::Result result = asyncSubscribe(WB_RES::LOCAL::MEAS_ACC_SAMPLERATE::ID, AsyncRequestOptions::Empty, sampleRate);*/
                            wb::Result result = asyncUnsubscribe(mMeasAccResourceId, NULL);
                            if (wb::RETURN_OKC(result)) {
                                DEBUGLOG("asyncUnsubscribe threw error: %u", result);

                                wb::Result result = getResource("Meas/Acc/13", mMeasAccResourceId);
                                if (wb::RETURN_OKC(result)) {
                                    // Blink to acknowledge command (SHORT_VISUAL_INDICATION)
                                    uint16_t indicationType = 2;
                                    asyncPut(WB_RES::LOCAL::UI_IND_VISUAL::ID, AsyncRequestOptions::Empty,
                                             indicationType);
                                }
                            }
                        }
                        break;
                    case 2 :
                        // Gyroscope
                        break;
                    case 3 :
                        // Magnetometer
                        break;
                    case 6 :
                        // IMU6
                        break;
                    case 9 :
                        // IMU9
                        if (commandValue % 100 > 9) {
                            // Record data with DataLogger <- implement later
                        } else {
                            uint16_t sampleRateValue = commandValue % 10;
                            uint16_t sampleRate = 13 * (2^sampleRateValue);
                            wb::Result resultAcc = asyncUnsubscribe(WB_RES::LOCAL::MEAS_ACC_SAMPLERATE::ID);
                            wb::Result resultImu9 = asyncUnsubscribe(WB_RES::LOCAL::MEAS_IMU9_SAMPLERATE::ID);
                            wb::Result result = asyncSubscribe(WB_RES::LOCAL::MEAS_IMU9_SAMPLERATE::ID, AsyncRequestOptions::Empty, sampleRate);
                            if (wb::RETURN_OKC(result))
                            {
                                // Blink to acknowledge command (SHORT_VISUAL_INDICATION)
                                uint16_t indicationType = 2;
                                asyncPut(WB_RES::LOCAL::UI_IND_VISUAL::ID, AsyncRequestOptions::Empty, indicationType);
                            }
                        }
                        break;
                    default :
                        // Not implemented, do nothing
                        break;
                }

                /*if (interval >= 0 && interval <= 4) {
                    int sampleRate = 13 * (2^interval);
                    wb::Result result = asyncUnsubscribe(WB_RES::LOCAL::MEAS_IMU9_SAMPLERATE::ID, NULL);
                    if (wb::RETURN_OKC(result))
                    {
                        wb::Result result2 = asyncSubscribe(WB_RES::LOCAL::MEAS_IMU9_SAMPLERATE::ID, AsyncRequestOptions::Empty, sampleRate);
                        if (wb::RETURN_OKC(result2)) {
                            uint16_t indicationType = 2; // SHORT_VISUAL_INDICATION, defined in ui/ind.yaml
                            // Make PUT request to trigger led blink
                            asyncPut(WB_RES::LOCAL::UI_IND_VISUAL::ID, AsyncRequestOptions::Empty, indicationType);
                        }
                    }
                }*/
            }
            else if (parameterRef.getCharHandle() == mMeasCharHandle) 
            {
                const WB_RES::Characteristic &charValue = value.convertTo<const WB_RES::Characteristic &>();
                bool bNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;
                DEBUGLOG("onNotify: mMeasCharHandle. bNotificationsEnabled: %d", bNotificationsEnabled);
                // Start or stop the timer
                if (mMeasurementTimer != whiteboard::ID_INVALID_TIMER)
                {
                    stopTimer(mMeasurementTimer);
                    mMeasurementTimer = whiteboard::ID_INVALID_TIMER;
                }
                if (bNotificationsEnabled)
                    mMeasurementTimer = startTimer(mMeasIntervalSecs*1000, true);
            }
        }
        break;

        case WB_RES::LOCAL::MEAS_IMU9_SAMPLERATE::LID:
        {
            // Temperature result or error
            const WB_RES::IMU9Data& imu9Data = value.convertTo<const WB_RES::IMU9Data&>();

            if (imu9Data.arrayAcc.size() <= 0 || imu9Data.arrayGyro.size() <= 0 || imu9Data.arrayMagn.size() <= 0)
            {
                // Some values are missing, do nothing...
                return;
            }

            const whiteboard::Array<whiteboard::FloatVector3D>& arrayDataAcc = imu9Data.arrayAcc;
            const whiteboard::Array<whiteboard::FloatVector3D>& arrayDataGyro = imu9Data.arrayGyro;
            const whiteboard::Array<whiteboard::FloatVector3D>& arrayDataMagn = imu9Data.arrayMagn;

            // For later use
            uint32_t relativeTime = imu9Data.timestamp;

            // Just in case arrays would prove to be of different length (probably highly unlikely..)
            size_t max = arrayDataAcc.size();
            if (arrayDataGyro.size() > max) { max = arrayDataGyro.size(); }
            if (arrayDataMagn.size() > max) { max = arrayDataMagn.size(); }

            uint8_t buffer[20]; // 1 byte or flags, 4 for FLOAT "in Celsius" value
            buffer[0]=0;

            for (size_t i = 0; i < max; i++)
            {
                if (arrayDataAcc.size() >= max) {
                    whiteboard::FloatVector3D accValue = arrayDataAcc[i];

                    uint16_t accValueX = convertFloatTo16bitInt(accValue.mX);
                    uint16_t accValueY = convertFloatTo16bitInt(accValue.mY);
                    uint16_t accValueZ = convertFloatTo16bitInt(accValue.mZ);

                    // Big-endian
                    buffer[0] = (uint8_t)(accValueX & 0xff);
                    buffer[1] = (uint8_t)((accValueX >> 8) & 0xff);
                    buffer[2] = (uint8_t)(accValueY & 0xff);
                    buffer[3] = (uint8_t)((accValueY >> 8) & 0xff);
                    buffer[4] = (uint8_t)(accValueZ & 0xff);
                    buffer[5] = (uint8_t)((accValueZ >> 8) & 0xff);
                }

                if (arrayDataGyro.size() >= max) {
                    whiteboard::FloatVector3D gyroValue = arrayDataGyro[i];

                    uint16_t gyroValueX = convertFloatTo16bitInt(gyroValue.mX);
                    uint16_t gyroValueY = convertFloatTo16bitInt(gyroValue.mY);
                    uint16_t gyroValueZ = convertFloatTo16bitInt(gyroValue.mZ);

                    buffer[6] = (uint8_t)(gyroValueX & 0xff);
                    buffer[7] = (uint8_t)((gyroValueX >> 8) & 0xff);
                    buffer[8] = (uint8_t)(gyroValueY & 0xff);
                    buffer[9] = (uint8_t)((gyroValueY >> 8) & 0xff);
                    buffer[10] = (uint8_t)(gyroValueZ & 0xff);
                    buffer[11] = (uint8_t)((gyroValueZ >> 8) & 0xff);
                }

                if (arrayDataMagn.size() >= max) {
                    whiteboard::FloatVector3D magnValue = arrayDataMagn[i];

                    uint16_t magnValueX = convertFloatTo16bitInt(magnValue.mX);
                    uint16_t magnValueY = convertFloatTo16bitInt(magnValue.mY);
                    uint16_t magnValueZ = convertFloatTo16bitInt(magnValue.mZ);

                    buffer[12] = (uint8_t)(magnValueX & 0xff);
                    buffer[13] = (uint8_t)((magnValueX >> 8) & 0xff);
                    buffer[14] = (uint8_t)(magnValueY & 0xff);
                    buffer[15] = (uint8_t)((magnValueY >> 8) & 0xff);
                    buffer[16] = (uint8_t)(magnValueZ & 0xff);
                    buffer[17] = (uint8_t)((magnValueZ >> 8) & 0xff);
                }

                buffer[18] = (uint8_t)(relativeTime & 0xff);
                buffer[19] = (uint8_t)((relativeTime >> 8) & 0xff);

                // Write the result to measChar. This results INDICATE to be triggered in GATT service
                WB_RES::Characteristic newMeasCharValue;
                newMeasCharValue.bytes = whiteboard::MakeArray<uint8_t>(buffer, sizeof(buffer));
                asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty, mTemperatureSvcHandle, mMeasCharHandle, newMeasCharValue);
            }
        }
        break;

        case WB_RES::LOCAL::MEAS_ACC_SAMPLERATE::LID:
        {
            // Temperature result or error
            const WB_RES::AccData& accData = value.convertTo<const WB_RES::AccData&>();

            if (accData.arrayAcc.size() <= 0)
            {
                // Some values are missing, do nothing...
                return;
            }

            const whiteboard::Array<whiteboard::FloatVector3D>& arrayDataAcc = accData.arrayAcc;

            uint32_t relativeTime = accData.timestamp;

            uint8_t buffer[20]; // 1 byte or flags, 4 for FLOAT "in Celsius" value
            buffer[0]=0;

            for (size_t i = 0; i < arrayDataAcc.size(); i++)
            {
                whiteboard::FloatVector3D accValue = arrayDataAcc[i];

                uint16_t accValueX = convertFloatTo16bitInt(accValue.mX);
                uint16_t accValueY = convertFloatTo16bitInt(accValue.mY);
                uint16_t accValueZ = convertFloatTo16bitInt(accValue.mZ);

                // Big-endian
                buffer[0] = (uint8_t)(accValueX & 0xff);
                buffer[1] = (uint8_t)((accValueX >> 8) & 0xff);
                buffer[2] = (uint8_t)(accValueY & 0xff);
                buffer[3] = (uint8_t)((accValueY >> 8) & 0xff);
                buffer[4] = (uint8_t)(accValueZ & 0xff);
                buffer[5] = (uint8_t)((accValueZ >> 8) & 0xff);

                buffer[6] = (uint8_t)(accValueX & 0xff);
                buffer[7] = (uint8_t)((accValueX >> 8) & 0xff);
                buffer[8] = (uint8_t)(accValueY & 0xff);
                buffer[9] = (uint8_t)((accValueY >> 8) & 0xff);
                buffer[10] = (uint8_t)(accValueZ & 0xff);
                buffer[11] = (uint8_t)((accValueZ >> 8) & 0xff);
                buffer[12] = (uint8_t)(accValueX & 0xff);
                buffer[13] = (uint8_t)((accValueX >> 8) & 0xff);
                buffer[14] = (uint8_t)(accValueY & 0xff);
                buffer[15] = (uint8_t)((accValueY >> 8) & 0xff);
                buffer[16] = (uint8_t)(accValueZ & 0xff);
                buffer[17] = (uint8_t)((accValueZ >> 8) & 0xff);


                buffer[18] = (uint8_t)(relativeTime & 0xff);
                buffer[19] = (uint8_t)((relativeTime >> 8) & 0xff);

                // Write the result to measChar. This results INDICATE to be triggered in GATT service
                WB_RES::Characteristic newMeasCharValue;
                newMeasCharValue.bytes = whiteboard::MakeArray<uint8_t>(buffer, sizeof(buffer));
                asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty, mTemperatureSvcHandle, mMeasCharHandle, newMeasCharValue);
            }
        }
    }
}

void CustomGATTSvcClient::onPostResult(whiteboard::RequestId requestId, 
                                  whiteboard::ResourceId resourceId, 
                                  whiteboard::Result resultCode, 
                                  const whiteboard::Value& rResultData) {
    DEBUGLOG("CustomGATTSvcClient::onPostResult: %d", resultCode);

    if (resultCode == whiteboard::HTTP_CODE_CREATED) {
        // Custom Gatt service was created
        mTemperatureSvcHandle = (int32_t)rResultData.convertTo<uint16_t>();
        DEBUGLOG("Custom Gatt service was created. handle: %d", mTemperatureSvcHandle);
        
        // Request more info about created svc so we get the char handles
        asyncGet(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE(), AsyncRequestOptions::Empty, mTemperatureSvcHandle);
    }
}