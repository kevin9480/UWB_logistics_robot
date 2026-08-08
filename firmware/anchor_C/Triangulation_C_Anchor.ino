/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSAnchorMain_TWR.ino
 * 
 * This is an example master anchor in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

#include <DW1000Jang.hpp>
#include <DW1000JangUtils.hpp>
#include <DW1000JangRanging.hpp>
#include <DW1000JangRTLS.hpp>


// connection pins
#if defined(ESP8266)
const uint8_t PIN_SS = 15;
#else
const uint8_t PIN_RST = 7;
const uint8_t PIN_SS = 10; // spi select pin
#endif



device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_128,
    PreambleCode::CODE_3
};

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
    false,
    false,
    true,
    false,
    false,
    false,
    false,
    false /* This allows blink frames */
};

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    Serial.println(F("### DW1000Jang-arduino-ranging-anchorC ###"));
    // initialize the driver
    #if defined(ESP8266)
    DW1000Jang::initializeNoInterrupt(PIN_SS);
    #else
    DW1000Jang::initializeNoInterrupt(PIN_SS, PIN_RST);
    #endif
    Serial.println(F("DW1000Jang initialized ..."));
    // general configuration
    DW1000Jang::applyConfiguration(DEFAULT_CONFIG);
    DW1000Jang::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);


    DW1000Jang::setPreambleDetectionTimeout(64);
    DW1000Jang::setSfdDetectionTimeout(273);
    DW1000Jang::setReceiveFrameWaitTimeoutPeriod(6000);

    DW1000Jang::setNetworkId(9);
    DW1000Jang::setDeviceAddress(3);
	
    DW1000Jang::setAntennaDelay(16436);
    
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Jang::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Jang::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Jang::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Jang::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);    
}

void loop() {

    RangeAcceptResult result = DW1000JangRTLS::Anchor_Distance_Response();
    
}