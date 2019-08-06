/*
 * Copyright (c) 2015 ARM Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include "ble/BLE.h"

#define def_fn(T, postfix) \
    SimpleChar<T> readOnly_##postfix \
        (uint16_t serviceUuid, const UUID& charUuid, bool enableNotify = true, T defaultValue = T()) { \
        return readOnly<T>(serviceUuid, charUuid, enableNotify, defaultValue); \
    }\
    SimpleChar<T> readOnly_##postfix \
        (const char* serviceUuid, const UUID& charUuid, bool enableNotify = true, T defaultValue = T()) { \
        return readOnly<T>(serviceUuid, charUuid, enableNotify, defaultValue); \
    }\
    \
    SimpleChar<T> readWrite_##postfix \
        (uint16_t serviceUuid, const UUID& charUuid, bool enableNotify = true, T defaultValue = T(), void(*callback)(T) = NULL) { \
        return readWrite<T>(serviceUuid, charUuid, enableNotify, defaultValue, callback); \
    }\
    SimpleChar<T> readWrite_##postfix \
    (const char* serviceUuid, const UUID& charUuid, bool enableNotify = true, T defaultValue = T(), void(*callback)(T) = NULL) { \
        return readWrite<T>(serviceUuid, charUuid, enableNotify, defaultValue, callback); \
    }\
    SimpleChar<T> readWrite_##postfix \
        (uint16_t serviceUuid, const UUID& charUuid, void(*callback)(T) = NULL) { \
        return readWrite<T>(serviceUuid, charUuid, callback); \
    }\
    SimpleChar<T> readWrite_##postfix \
    (const char* serviceUuid, const UUID& charUuid, void(*callback)(T) = NULL) { \
        return readWrite<T>(serviceUuid, charUuid, callback); \
    }\
    \
    SimpleChar<T> writeOnly_##postfix \
    (uint16_t serviceUuid, const UUID& charUuid, void(*callback)(T) = NULL) { \
        return writeOnly<T>(serviceUuid, charUuid, callback); \
    }\
    SimpleChar<T> writeOnly_##postfix \
    (const char* serviceUuid, const UUID& charUuid, void(*callback)(T) = NULL) { \
        return writeOnly<T>(serviceUuid, charUuid, callback); \
    }

using namespace std;

/**
 * Class so we can call onDataWritten on any SimpleCharInternal regardless of <T,U>
 */
class Updatable {
public:
    virtual void onDataWritten(const uint8_t* data, size_t len) = 0;
};

/**
 * Class that we wrap in SimpleChar so we can just implement operators,
 * without having to type the full type of U when using this code.
 * Whenever we get 'auto' we can get rid of this.
 */
template <class T>
class SimpleCharBase {
public:
    virtual void update(T newValue) = 0;
    virtual T* getValue(void) = 0;  
};

/**
 * Actual implementation of the char
 * T is the underlying type, U is the GattCharacteristic it's wrapping
 */
template <class T, template <typename T2> class U>
class SimpleCharInternal : public Updatable, public SimpleCharBase<T> {
public:
    SimpleCharInternal(BLE* aBle, 
               const UUID &uuid, 
               GattCharacteristic::Properties_t aGattChar, 
               T aDefaultValue,
               void(*aCallback)(T) = NULL) :
        ble(aBle), value(new T(aDefaultValue)), callback(aCallback)
    {
        state = new U<T>(uuid, value, aGattChar);
    }
    
    ~SimpleCharInternal() {
        if (state) {
            free(state);
        }
        if (value) {
            free(value);
        }
    }
    
    virtual void update(T newValue) {
        *value = newValue;
        ble->gattServer().write(state->getValueHandle(), (uint8_t *)value, sizeof(T));
    }
    
    U<T>* getChar(void) {
        return state;
    }

    virtual T* getValue(void) {
        return value;
    }
    
    virtual void onDataWritten(const uint8_t* data, size_t len) {
        *value = ((T*)data)[0];
        if (callback) {
            callback(*value);
        }
    }

private:
    BLE* ble;
    T* value;
    U<T>* state;
    void(*callback)(T);
};

/**
 * This is what the user gets back. it's nice and short so don't have to type much.
 * If we get 'auto' we can get rid of this.
 */
template <class T>
class SimpleChar {
public:
    SimpleChar(SimpleCharBase<T>* aBase) : base(aBase) {
    }
    ~SimpleChar() {
        if (base) {
            delete base;
        }
    }

    T operator=(const T& newValue) {
        base->update(newValue);
        return newValue;
    };
    operator T() const {
        return *(base->getValue());
    };
    
private:
    SimpleCharBase<T>* base;
};


class SimpleBLE {
public:
    SimpleBLE(const char* aName, uint16_t aInterval = 1000, bool aLogging = true) 
        : name(aName), interval(aInterval), logging(aLogging) 
    {
        ble = &BLE::Instance();
    }
    ~SimpleBLE() {}
    
    void start() {
        ble->init(this, &SimpleBLE::bleInitComplete);
    
        /* SpinWait for initialization to complete. This is necessary because the
         * BLE object is used in the main loop below. */
        while (ble->hasInitialized()  == false) { /* spin loop */ }
    }
    
    // Start up the BLE service and just run with it!
    void waitForEvent() {
        ble->waitForEvent();
    }    
    
    void onDisconnection(Gap::DisconnectionEventCallback_t callback) {
        ble->gap().onDisconnection(callback);
    }
    
    void onConnection(Gap::ConnectionEventCallback_t callback) {
        ble->gap().onConnection(callback);
    }
    
    BLE* getBle(void) {
        return ble;
    }
    
    def_fn(uint8_t, u8)
    def_fn(uint16_t, u16)
    def_fn(uint32_t, u32)
    def_fn(int8_t, i8)
    def_fn(int16_t, i16)
    def_fn(int32_t, i32)
    def_fn(bool, bool)
    def_fn(float, float)
    
private:
    void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
    {
        if (logging) printf("bleInitComplete\r\n");
        
        BLE&        ble   = params->ble;
        ble_error_t error = params->error;
    
        if (error != BLE_ERROR_NONE) {
            if (logging) printf("BLE Init error %d\r\n", error);
            return;
        }
    
        /* Ensure that it is the default instance of BLE */
        if(ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
            return;
        }

        ble.gattServer().onDataWritten(this, &SimpleBLE::onDataWrittenCallback);
        
        // let's add some services yo (why is there no 'auto' in mbed?)
        uint16_t uuid16_list[uint16_services.size()];
        size_t uuid16_counter = 0;
        {   
            typedef std::map<uint16_t, vector<GattCharacteristic*>* >::iterator it_type;
            for(it_type it = uint16_services.begin(); it != uint16_services.end(); it++) {
                if (logging) printf("Creating service 0x%x\n", it->first);
                uuid16_list[uuid16_counter++] = it->first;            
    
                GattCharacteristic* charTable[it->second->size()];
                for (size_t git = 0; git < it->second->size(); git++) {
                    charTable[git] = it->second->at(git);
                }
    
                GattService service(it->first, charTable, it->second->size());
                ble.gattServer().addService(service);
            }
        }

        // 128 Bit services
        const char* uuid128_list[uint128_services.size()];
        size_t uuid128_counter = 0;
        {
            typedef std::map<string, vector<GattCharacteristic*>* >::iterator it_type;
            for(it_type it = uint128_services.begin(); it != uint128_services.end(); it++) {
                if (logging) printf("Creating service %s\n", it->first.c_str());
                uuid128_list[uuid128_counter++] = it->first.c_str();            
    
                GattCharacteristic* charTable[it->second->size()];
                for (size_t git = 0; git < it->second->size(); git++) {
                    charTable[git] = it->second->at(git);
                }
    
                GattService service(UUID(it->first.c_str()), charTable, it->second->size());
                ble.gattServer().addService(service);
            }
        }

        ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
        ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, uuid16_counter);
        ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS, (uint8_t *)uuid128_list, uuid128_counter);
        ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)name, strlen(name));
        ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
        ble.gap().setAdvertisingInterval(interval);
        ble.gap().startAdvertising();
        
        if (logging) printf("Started advertising\r\n");
    }
    
    void onDataWrittenCallback(const GattWriteCallbackParams *params) {
        // see if we know for which char this message is...
        typedef std::map<GattCharacteristic*, Updatable* >::iterator it_type;
        for(it_type it = writeCallbacks.begin(); it != writeCallbacks.end(); it++) {
            if (it->first->getValueHandle() == params->handle) {
                it->second->onDataWritten(params->data, params->len);
            }
        }
    }
    
    void addToServices(uint16_t uuid, GattCharacteristic* c) {
        if (uint16_services.count(uuid) == 0) {
            uint16_services[uuid] = new vector<GattCharacteristic*>();
        }

        uint16_services[uuid]->push_back(c);
    }
    
    void addToServices(const char* aUuid, GattCharacteristic* c) {
        string uuid(aUuid);
        if (uint128_services.count(uuid) == 0) {
            uint128_services[uuid] = new vector<GattCharacteristic*>();
        }

        uint128_services[uuid]->push_back(c);
    }
    
    // === START READONLY ===
    
    template <typename T>
    SimpleChar<T> readOnly(uint16_t serviceUuid, 
                                     const UUID& charUuid, 
                                     bool enableNotify = true,
                                     T defaultValue = T()) {
        GattCharacteristic::Properties_t gattChar = enableNotify ? 
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY :
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ;

        SimpleCharInternal<T, ReadOnlyGattCharacteristic>* c = 
            new SimpleCharInternal<T, ReadOnlyGattCharacteristic>(ble, charUuid, gattChar, defaultValue);
        
        addToServices(serviceUuid, c->getChar());
        
        return *(new SimpleChar<T>(c));
    }

    template <typename T>
    SimpleChar<T> readOnly(const char* serviceUuid, 
                                     const UUID& charUuid, 
                                     bool enableNotify = true,
                                     T defaultValue = T()) {
        GattCharacteristic::Properties_t gattChar = enableNotify ? 
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY :
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ;

        SimpleCharInternal<T, ReadOnlyGattCharacteristic>* c = 
            new SimpleCharInternal<T, ReadOnlyGattCharacteristic>(ble, charUuid, gattChar, defaultValue);
        
        addToServices(serviceUuid, c->getChar());

        return *(new SimpleChar<T>(c));
    }

    // === END READONLY ===
    
    // === START READWRITE ===

    template <typename T>
    SimpleChar<T> readWrite(uint16_t serviceUuid, 
                                      const UUID& charUuid, 
                                      bool enableNotify = true,
                                      T defaultValue = T(),
                                      void(*callback)(T) = NULL) {
        GattCharacteristic::Properties_t gattChar = enableNotify ? 
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY :
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ;

        SimpleCharInternal<T, ReadWriteGattCharacteristic>* c = 
            new SimpleCharInternal<T, ReadWriteGattCharacteristic>(ble, charUuid, gattChar, defaultValue, callback);

        addToServices(serviceUuid, c->getChar());

        writeCallbacks[c->getChar()] = c;

        return *(new SimpleChar<T>(c));
    }
    
    

    template <typename T>
    SimpleChar<T> readWrite(const char* serviceUuid, 
                                      const UUID& charUuid, 
                                      bool enableNotify = true,
                                      T defaultValue = T(),
                                      void(*callback)(T) = NULL) {
        GattCharacteristic::Properties_t gattChar = enableNotify ? 
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY :
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ;

        SimpleCharInternal<T, ReadWriteGattCharacteristic>* c = 
            new SimpleCharInternal<T, ReadWriteGattCharacteristic>(ble, charUuid, gattChar, defaultValue, callback);

        addToServices(serviceUuid, c->getChar());

        writeCallbacks[c->getChar()] = c;
        
        return *(new SimpleChar<T>(c));
    }
    
    template <typename T>
    SimpleChar<T> readWrite(uint16_t serviceUuid, 
                                      const UUID& charUuid, 
                                      void(*callback)(T) = NULL) {
        return readWrite(serviceUuid, charUuid, true, T(), callback);
    }

    template <typename T>
    SimpleChar<T> readWrite(const char* serviceUuid, 
                                      const UUID& charUuid, 
                                      void(*callback)(T) = NULL) {
        return readWrite(serviceUuid, charUuid, true, T(), callback);
    }
    
    // === END READWRITE ===
    
    // === START WRITEONLY ===

    template <typename T>
    SimpleChar<T> writeOnly(uint16_t serviceUuid, 
                                      const UUID& charUuid,
                                      void(*callback)(T) = NULL) {
        SimpleCharInternal<T, WriteOnlyGattCharacteristic>* c = 
            new SimpleCharInternal<T, WriteOnlyGattCharacteristic>(ble, charUuid, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NONE, T(), callback);

        addToServices(serviceUuid, c->getChar());

        writeCallbacks[c->getChar()] = c;

        return *(new SimpleChar<T>(c));
    }

    template <typename T>
    SimpleChar<T> writeOnly(const char* serviceUuid, 
                                      const UUID& charUuid,
                                      void(*callback)(T) = NULL) {

        SimpleCharInternal<T, WriteOnlyGattCharacteristic>* c = 
            new SimpleCharInternal<T, WriteOnlyGattCharacteristic>(ble, charUuid, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NONE, T(), callback);

        addToServices(serviceUuid, c->getChar());

        writeCallbacks[c->getChar()] = c;
        
        return *(new SimpleChar<T>(c));
    }
    
    // === END WRITEONLY ===

    BLE* ble;
    const char* name;
    uint16_t interval;
    bool logging;
    map<uint16_t, vector<GattCharacteristic*>* > uint16_services;
    map<string, vector<GattCharacteristic*>* > uint128_services;
    map<GattCharacteristic*, Updatable*> writeCallbacks;
};

