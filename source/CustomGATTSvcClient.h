#pragma once

#include <whiteboard/LaunchableModule.h>
#include <whiteboard/ResourceClient.h>

class CustomGATTSvcClient FINAL : private whiteboard::ResourceClient,
                           public whiteboard::LaunchableModule

{
public:
    /** Name of this class. Used in StartupProvider list. */
    static const char* const LAUNCHABLE_NAME;
    CustomGATTSvcClient();
    ~CustomGATTSvcClient();

private:
    /** @see whiteboard::ILaunchableModule::initModule */
    virtual bool initModule() OVERRIDE;

    /** @see whiteboard::ILaunchableModule::deinitModule */
    virtual void deinitModule() OVERRIDE;

    /** @see whiteboard::ILaunchableModule::startModule */
    virtual bool startModule() OVERRIDE;

    /** @see whiteboard::ILaunchableModule::stopModule */
    virtual void stopModule() OVERRIDE;

    /***
    * Callback for POST operation result
    *
    * @param requestId ID of the request
    * @param resourceId Successful request contains ID of the resource
    * @param resultCode Result code of the request
    * @param rResultData Successful result contains the request result
    *
    * @see whiteboard::ResourceClient::asyncPost
    */
    virtual void onPostResult(whiteboard::RequestId requestId, whiteboard::ResourceId resourceId, whiteboard::Result resultCode, const whiteboard::Value& rResultData) OVERRIDE;

    /**
    *	Callback for asynchronous resource GET requests
    *
    *	@param requestId ID of the request
    *	@param resourceId Successful request contains ID of the resource
    *	@param resultCode Result code of the request
    *	@param rResultData Successful result contains the request result
    */
    virtual void onGetResult(whiteboard::RequestId requestId, whiteboard::ResourceId resourceId, whiteboard::Result resultCode, const whiteboard::Value& rResultData);

    /**
    *	Callback for resource notifications.
    *
    *	@param resourceId Resource id associated with the update
    *	@param rValue Current value of the resource
    *	@param rParameters Notification parameters
    */
    virtual void onNotify(whiteboard::ResourceId resourceId, const whiteboard::Value& rValue, const whiteboard::ParameterList& rParameters);

    /**
    *   Make led light blink. Three different types supported by sensor.
    *   2 = SHORT_VISUAL_INDICATION
    *
    *   @param indicationType Indication type (1-3)
    */
    virtual void showLedIndication(uint16_t indicationType);

    /**
    *   Calculate compatible samplerate from level.
    *
    *   @param sampleRateValue Command value representing desired level of samplerate.
    *   @return Sensor adjusted samplerate.
    */
    virtual uint16_t calculateSampleRate(uint16_t sampleRateValue);

    /**
    *   Build resource string to char array.
    *
    *   @param base Base rest URL
    *   @param samplerate Chosen samplerate
    *   @param destination Char array to hold completed resource string
    */
    virtual void buildResourceString(char *base, uint16_t samplerate, char destination[]);

    /**
    *   Handle received command
    *
    *   @param command Received command
    */
    virtual void handleCommand(uint16_t command);

    /**
    *   Handle update on notifications
    *
    *   @param isNotificationEnabled Whether there are notifications subscribers
    */
    virtual void handleNotification(bool isNotificationEnabled);

    /**
    *   Convert 4 byte float to 2 byte unsigned int (with small accuracy loss)
    *
    * @param num Float number to convert
    * @return Converted 2 byte unsigned int
    */
    virtual uint16_t convertFloatTo16bitInt (float &num);

    /**
    *   Handle received IMU9 sensor data
    *
    *   @param data IMU9 sensor data
    */
    virtual void handleSensorDataIMU9(uint32_t timestamp, const whiteboard::Array<whiteboard::FloatVector3D>& accData, const whiteboard::Array<whiteboard::FloatVector3D>& gyroData, const whiteboard::Array<whiteboard::FloatVector3D>& magnData);

    /**
    *   Handle received single sensor data
    *
    * @param timestamp Timestamp of data
    * @param arrayData Array of sensor data
    */
    virtual void handleSingleSensorData(uint32_t timestamp, const whiteboard::Array<whiteboard::FloatVector3D>& arrayData);

private:
    void configGattSvc();

    whiteboard::ResourceId mIntervalCharResource;
    whiteboard::ResourceId mMeasCharResource;
    whiteboard::TimerId mMeasurementTimer;

    uint16_t mMeasIntervalSecs;
    int32_t mTemperatureSvcHandle;
    int32_t mMeasCharHandle;
    int32_t mIntervalCharHandle;
};
