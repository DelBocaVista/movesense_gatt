#include "movesense.h"

#include "CustomGATTSvcClient.h"
#include "common/core/debug.h"
#include "oswrapper/thread.h"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"
#include <meas_temp/resources.h>
#include <meas_acc/resources.h>
#include <meas_gyro/resources.h>
#include <meas_magn/resources.h>
#include <meas_imu/resources.h>
#include <cstdint>
#include "whiteboard/builtinTypes/UnknownStructure.h"

#define SPAN 4000.0
#define NR_SIZE 65535.0

const char* const CustomGATTSvcClient::LAUNCHABLE_NAME = "CstGattS";

whiteboard::ResourceId	mMeasResourceId;
char resourceFullString[30] = "Meas/Acc/13";
char measAccResourceBase[] = "Meas/Acc/";
char measGyroResourceBase[] = "Meas/Gyro/";
char measMagnResourceBase[] = "Meas/Magn/";
char measIMU6ResourceBase[] = "Meas/IMU6/";
char measIMU9ResourceBase[] = "Meas/IMU9/";
bool isRunning = false;

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

    return true;
}

/** @see whiteboard::ILaunchableModule::startModule */
void CustomGATTSvcClient::stopModule()
{
    // Unsubscribe if needed
    if (mIntervalCharResource != whiteboard::ID_INVALID_RESOURCE)
        asyncUnsubscribe(mIntervalCharResource);
    if (mMeasCharResource != whiteboard::ID_INVALID_RESOURCE)
        asyncUnsubscribe(mMeasCharResource);
}

#include "ui_ind/resources.h"
void CustomGATTSvcClient::configGattSvc() {
    WB_RES::GattSvc customGattSvc;
    WB_RES::GattChar characteristics[2];
    WB_RES::GattChar &measChar = characteristics[0];
    WB_RES::GattChar &intervalChar = characteristics[1];

    const uint16_t healthThermometerSvcUUID16 = 0x1809;
    
    // Define the CMD characteristics
    WB_RES::GattProperty measCharProp[3] = {WB_RES::GattProperty::NOTIFY, WB_RES::GattProperty::READ, WB_RES::GattProperty::WRITE};
    WB_RES::GattProperty intervalCharProps[2] = {WB_RES::GattProperty::READ, WB_RES::GattProperty::WRITE};

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

#include <math.h>

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
    }
}

uint16_t CustomGATTSvcClient::convertFloatTo16bitInt (float &num) {
    return (uint16_t) round(NR_SIZE * (num + SPAN/2) / SPAN);
}

void CustomGATTSvcClient::buildResourceString(char *base, uint16_t samplerate, char destination[]) {

    int i;
    for(i = 0; *base != '\0'; base++) {
        destination[i] = *base;
        i++;
    }

    int reversed = 0;

    while (samplerate != 0) {
        reversed = reversed * 10;
        reversed = reversed + samplerate % 10;
        samplerate /= 10;
    }

    while(reversed != 0) {
        destination[i] = 48 + (reversed % 10);
        i++;
        reversed /= 10;
    }

    destination[i] = '\0';
}

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

                uint16_t commandValue = ((uint16_t)commandValueLower << 8) | commandValueHigher;

                handleCommand(commandValue);
            }
            else if (parameterRef.getCharHandle() == mMeasCharHandle) 
            {
                const WB_RES::Characteristic &charValue = value.convertTo<const WB_RES::Characteristic &>();
                bool bNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;

                handleNotification(bNotificationsEnabled);
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

            // Timestamp
            uint32_t relativeTime = imu9Data.timestamp;

            handleSensorDataIMU9(relativeTime, arrayDataAcc, arrayDataGyro, arrayDataMagn);
        }
        break;

        case WB_RES::LOCAL::MEAS_ACC_SAMPLERATE::LID:
        {
            // Accelerometer result or error
            const WB_RES::AccData& accData = value.convertTo<const WB_RES::AccData&>();

            if (accData.arrayAcc.size() <= 0)
            {
                // Some values are missing, do nothing...
                return;
            }

            const whiteboard::Array<whiteboard::FloatVector3D>& arrayDataAcc = accData.arrayAcc;

            uint32_t relativeTime = accData.timestamp;

            handleSingleSensorData(relativeTime, arrayDataAcc);
        }
        break;

        case WB_RES::LOCAL::MEAS_GYRO_SAMPLERATE::LID:
        {
            // Accelerometer result or error
            const WB_RES::GyroData& gyroData = value.convertTo<const WB_RES::GyroData&>();

            if (gyroData.arrayGyro.size() <= 0)
            {
                // Some values are missing, do nothing...
                return;
            }

            const whiteboard::Array<whiteboard::FloatVector3D>& arrayDataGyro = gyroData.arrayGyro;

            uint32_t relativeTime = gyroData.timestamp;

            handleSingleSensorData(relativeTime, arrayDataGyro);
        }
        break;

        case WB_RES::LOCAL::MEAS_MAGN_SAMPLERATE::LID:
        {
            // Accelerometer result or error
            const WB_RES::MagnData& magnData = value.convertTo<const WB_RES::MagnData&>();

            if (magnData.arrayMagn.size() <= 0)
            {
                // Some values are missing, do nothing...
                return;
            }

            const whiteboard::Array<whiteboard::FloatVector3D>& arrayDataMagn = magnData.arrayMagn;

            uint32_t relativeTime = magnData.timestamp;

            handleSingleSensorData(relativeTime, arrayDataMagn);
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

void CustomGATTSvcClient::showLedIndication(uint16_t indicationType) {
    asyncPut(WB_RES::LOCAL::UI_IND_VISUAL::ID, AsyncRequestOptions::Empty,
             indicationType);
}

uint16_t CustomGATTSvcClient::calculateSampleRate(uint16_t sampleRateValue) {
    return 13 * pow(2, sampleRateValue);
}

void CustomGATTSvcClient::handleCommand(uint16_t command) {
    uint16_t commandValueType = command / 100;

    switch(commandValueType) {
        case 0 :
            // General command settings
            switch(command % 100) {
                case 0:
                    // Shutdown - i.e. stop subscribing to chosen resource.
                    if (isRunning)
                    {
                        // Blink to acknowledge command (SHORT_VISUAL_INDICATION)
                        wb::Result result = asyncUnsubscribe(mMeasResourceId, NULL);
                        if (wb::RETURN_OKC(result)) {
                            showLedIndication(2);
                            isRunning = false;
                        }
                    }
                    break;
                default:
                    // Not valid command. Do nothing.
                    break;
            }
            break;
        case 1 :
            // Accelerometer
            if (command % 100 > 9) {
                // Record data with DataLogger <- implement later
            } else {
                uint16_t sampleRate = calculateSampleRate(command % 10);
                buildResourceString(measAccResourceBase, sampleRate, resourceFullString);
                showLedIndication(2);
            }
            break;
        case 2 :
            // Gyroscope
            if (command % 100 > 9) {
                // Record data with DataLogger <- implement later
            } else {
                uint16_t sampleRate = calculateSampleRate(command % 10);
                buildResourceString(measGyroResourceBase, sampleRate, resourceFullString);
                showLedIndication(2);
            }
            break;
        case 3 :
            // Magnetometer
            if (command % 100 > 9) {
                // Record data with DataLogger <- implement later
            } else {
                uint16_t sampleRate = calculateSampleRate(command % 10);
                buildResourceString(measMagnResourceBase, sampleRate, resourceFullString);
                showLedIndication(2);
            }
            break;
        case 6 :
            // IMU6
            if (command % 100 > 9) {
                // Record data with DataLogger <- implement later
            } else {
                uint16_t sampleRate = calculateSampleRate(command % 10);
                buildResourceString(measIMU6ResourceBase, sampleRate, resourceFullString);
                showLedIndication(2);
            }
            break;
        case 9 :
            // IMU9
            if (command % 100 > 9) {
                // Record data with DataLogger <- implement later
            } else {
                uint16_t sampleRate = calculateSampleRate(command % 10);
                buildResourceString(measIMU9ResourceBase, sampleRate, resourceFullString);
                showLedIndication(2);
            }
            break;
        default :
            // Not implemented, do nothing
            break;
    }
}

void CustomGATTSvcClient::handleNotification(bool isNotificationEnabled) {
    if (isNotificationEnabled) {
        if (isRunning) {
            wb::Result unsubResult = asyncUnsubscribe(mMeasResourceId, NULL);
            if (wb::RETURN_OKC(unsubResult)) {

                wb::Result resResult = getResource(resourceFullString, mMeasResourceId);
                if (wb::RETURN_OKC(resResult)) {
                    // Blink to acknowledge command (SHORT_VISUAL_INDICATION)
                    wb::Result subResult = asyncSubscribe(mMeasResourceId, AsyncRequestOptions::Empty);
                    if (wb::RETURN_OKC(subResult)) {
                        showLedIndication(2);
                        isRunning = true;
                    }
                }
            }
        } else {
            wb::Result resResult = getResource(resourceFullString, mMeasResourceId);
            if (wb::RETURN_OKC(resResult)) {
                // Blink to acknowledge command (SHORT_VISUAL_INDICATION)
                wb::Result subResult = asyncSubscribe(mMeasResourceId, AsyncRequestOptions::Empty);
                if (wb::RETURN_OKC(subResult)) {
                    // Blink to acknowledge command (SHORT_VISUAL_INDICATION)
                    showLedIndication(2);
                    isRunning = true;
                }
            }
        }
    } else {
        if (isRunning) {
            wb::Result unsubResult = asyncUnsubscribe(mMeasResourceId, NULL);
            if (wb::RETURN_OKC(unsubResult)) {
                // Blink to acknowledge command (SHORT_VISUAL_INDICATION)
                showLedIndication(2);
                isRunning = false;
            }
        }
    }
}

void CustomGATTSvcClient::handleSensorDataIMU9(uint32_t timestamp, const whiteboard::Array<whiteboard::FloatVector3D>& accData, const whiteboard::Array<whiteboard::FloatVector3D>& gyroData, const whiteboard::Array<whiteboard::FloatVector3D>& magnData) {
    // Just in case arrays would prove to be of different length (probably highly unlikely..)
    size_t max = accData.size();
    if (gyroData.size() > max) { max = gyroData.size(); }
    if (magnData.size() > max) { max = magnData.size(); }

    uint8_t buffer[20]; // 1 byte or flags, 4 for FLOAT "in Celsius" value
    buffer[0]=0;

    for (size_t i = 0; i < max; i++)
    {
        buffer[0] = (uint8_t)(timestamp & 0xff);
        buffer[1] = (uint8_t)((timestamp >> 8) & 0xff);

        if (accData.size() >= max) {
            whiteboard::FloatVector3D accValue = accData[i];

            uint16_t accValueX = convertFloatTo16bitInt(accValue.mX);
            uint16_t accValueY = convertFloatTo16bitInt(accValue.mY);
            uint16_t accValueZ = convertFloatTo16bitInt(accValue.mZ);

            // Big-endian
            buffer[2] = (uint8_t)(accValueX & 0xff);
            buffer[3] = (uint8_t)((accValueX >> 8) & 0xff);
            buffer[4] = (uint8_t)(accValueY & 0xff);
            buffer[5] = (uint8_t)((accValueY >> 8) & 0xff);
            buffer[6] = (uint8_t)(accValueZ & 0xff);
            buffer[7] = (uint8_t)((accValueZ >> 8) & 0xff);
        }

        if (gyroData.size() >= max) {
            whiteboard::FloatVector3D gyroValue = gyroData[i];

            uint16_t gyroValueX = convertFloatTo16bitInt(gyroValue.mX);
            uint16_t gyroValueY = convertFloatTo16bitInt(gyroValue.mY);
            uint16_t gyroValueZ = convertFloatTo16bitInt(gyroValue.mZ);

            buffer[8] = (uint8_t)(gyroValueX & 0xff);
            buffer[9] = (uint8_t)((gyroValueX >> 8) & 0xff);
            buffer[10] = (uint8_t)(gyroValueY & 0xff);
            buffer[11] = (uint8_t)((gyroValueY >> 8) & 0xff);
            buffer[12] = (uint8_t)(gyroValueZ & 0xff);
            buffer[13] = (uint8_t)((gyroValueZ >> 8) & 0xff);
        }

        if (magnData.size() >= max) {
            whiteboard::FloatVector3D magnValue = magnData[i];

            uint16_t magnValueX = convertFloatTo16bitInt(magnValue.mX);
            uint16_t magnValueY = convertFloatTo16bitInt(magnValue.mY);
            uint16_t magnValueZ = convertFloatTo16bitInt(magnValue.mZ);

            buffer[14] = (uint8_t)(magnValueX & 0xff);
            buffer[15] = (uint8_t)((magnValueX >> 8) & 0xff);
            buffer[16] = (uint8_t)(magnValueY & 0xff);
            buffer[17] = (uint8_t)((magnValueY >> 8) & 0xff);
            buffer[18] = (uint8_t)(magnValueZ & 0xff);
            buffer[19] = (uint8_t)((magnValueZ >> 8) & 0xff);
        }

        // Write the result to measChar. This results INDICATE to be triggered in GATT service
        WB_RES::Characteristic newMeasCharValue;
        newMeasCharValue.bytes = whiteboard::MakeArray<uint8_t>(buffer, sizeof(buffer));
        asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty, mTemperatureSvcHandle, mMeasCharHandle, newMeasCharValue);
    }
}

void CustomGATTSvcClient::handleSingleSensorData(uint32_t timestamp, const whiteboard::Array<whiteboard::FloatVector3D>& arrayData) {
    uint8_t buffer[20];
    buffer[0]=0;

    for (size_t i = 0; i < arrayData.size(); i++)
    {
        whiteboard::FloatVector3D value = arrayData[i];

        uint16_t valueX = convertFloatTo16bitInt(value.mX);
        uint16_t valueY = convertFloatTo16bitInt(value.mY);
        uint16_t valueZ = convertFloatTo16bitInt(value.mZ);

        // Big-endian
        buffer[0] = (uint8_t)(timestamp & 0xff);
        buffer[1] = (uint8_t)((timestamp >> 8) & 0xff);
        buffer[2] = (uint8_t)(valueX & 0xff);
        buffer[3] = (uint8_t)((valueX >> 8) & 0xff);
        buffer[4] = (uint8_t)(valueY & 0xff);
        buffer[5] = (uint8_t)((valueY >> 8) & 0xff);
        buffer[6] = (uint8_t)(valueZ & 0xff);
        buffer[7] = (uint8_t)((valueZ >> 8) & 0xff);

        // Write the result to measChar. This results INDICATE to be triggered in GATT service
        WB_RES::Characteristic newMeasCharValue;
        newMeasCharValue.bytes = whiteboard::MakeArray<uint8_t>(buffer, sizeof(buffer));
        asyncPut(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE(), AsyncRequestOptions::Empty, mTemperatureSvcHandle, mMeasCharHandle, newMeasCharValue);
    }
}