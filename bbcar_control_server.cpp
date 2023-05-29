/*
 * Generated by erpcgen 1.10.0 on Sun May 28 20:10:47 2023.
 *
 * AUTOGENERATED - DO NOT EDIT
 */


#include "bbcar_control_server.h"
#if ERPC_ALLOCATION_POLICY == ERPC_ALLOCATION_POLICY_DYNAMIC
#include <new>
#include "erpc_port.h"
#endif
#include "erpc_manually_constructed.hpp"

#if 11000 != ERPC_VERSION_NUMBER
#error "The generated shim code version is different to the rest of eRPC code."
#endif

using namespace erpc;
using namespace std;

#if ERPC_NESTED_CALLS_DETECTION
extern bool nestingDetection;
#endif

ERPC_MANUALLY_CONSTRUCTED_STATIC(BBCarService_service, s_BBCarService_service);



// Call the correct server shim based on method unique ID.
erpc_status_t BBCarService_service::handleInvocation(uint32_t methodId, uint32_t sequence, Codec * codec, MessageBufferFactory *messageFactory)
{
    erpc_status_t erpcStatus;
    switch (methodId)
    {
        case kBBCarService_stop_id:
        {
            erpcStatus = stop_shim(codec, messageFactory, sequence);
            break;
        }

        case kBBCarService_goStraight_id:
        {
            erpcStatus = goStraight_shim(codec, messageFactory, sequence);
            break;
        }

        case kBBCarService_turn_id:
        {
            erpcStatus = turn_shim(codec, messageFactory, sequence);
            break;
        }

        case kBBCarService_ctl_mode_id:
        {
            erpcStatus = ctl_mode_shim(codec, messageFactory, sequence);
            break;
        }

        case kBBCarService_RemoteControlAction_id:
        {
            erpcStatus = RemoteControlAction_shim(codec, messageFactory, sequence);
            break;
        }

        case kBBCarService_RemoteTuneSpeed_id:
        {
            erpcStatus = RemoteTuneSpeed_shim(codec, messageFactory, sequence);
            break;
        }

        case kBBCarService_RemoteShowSpeed_id:
        {
            erpcStatus = RemoteShowSpeed_shim(codec, messageFactory, sequence);
            break;
        }

        case kBBCarService_RemoteShowPattern_id:
        {
            erpcStatus = RemoteShowPattern_shim(codec, messageFactory, sequence);
            break;
        }

        default:
        {
            erpcStatus = kErpcStatus_InvalidArgument;
            break;
        }
    }

    return erpcStatus;
}

// Server shim for stop of BBCarService interface.
erpc_status_t BBCarService_service::stop_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    uint8_t cars;

    // startReadMessage() was already called before this shim was invoked.

    codec->read(&cars);

    err = codec->getStatus();
    if (err == kErpcStatus_Success)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        stop(cars);
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (err == kErpcStatus_Success)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kBBCarService_service_id, kBBCarService_stop_id, sequence);

        err = codec->getStatus();
    }

    return err;
}

// Server shim for goStraight of BBCarService interface.
erpc_status_t BBCarService_service::goStraight_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    uint8_t cars;
    int32_t speed;

    // startReadMessage() was already called before this shim was invoked.

    codec->read(&cars);

    codec->read(&speed);

    err = codec->getStatus();
    if (err == kErpcStatus_Success)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        goStraight(cars, speed);
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (err == kErpcStatus_Success)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kBBCarService_service_id, kBBCarService_goStraight_id, sequence);

        err = codec->getStatus();
    }

    return err;
}

// Server shim for turn of BBCarService interface.
erpc_status_t BBCarService_service::turn_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    uint8_t cars;
    int32_t speed;
    double factor;

    // startReadMessage() was already called before this shim was invoked.

    codec->read(&cars);

    codec->read(&speed);

    codec->read(&factor);

    err = codec->getStatus();
    if (err == kErpcStatus_Success)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        turn(cars, speed, factor);
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (err == kErpcStatus_Success)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kBBCarService_service_id, kBBCarService_turn_id, sequence);

        err = codec->getStatus();
    }

    return err;
}

// Server shim for ctl_mode of BBCarService interface.
erpc_status_t BBCarService_service::ctl_mode_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;


    // startReadMessage() was already called before this shim was invoked.

    err = codec->getStatus();
    if (err == kErpcStatus_Success)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        ctl_mode();
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (err == kErpcStatus_Success)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kBBCarService_service_id, kBBCarService_ctl_mode_id, sequence);

        err = codec->getStatus();
    }

    return err;
}

// Server shim for RemoteControlAction of BBCarService interface.
erpc_status_t BBCarService_service::RemoteControlAction_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    uint8_t mode;
    int32_t value;
    double factor;

    // startReadMessage() was already called before this shim was invoked.

    codec->read(&mode);

    codec->read(&value);

    codec->read(&factor);

    err = codec->getStatus();
    if (err == kErpcStatus_Success)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        RemoteControlAction(mode, value, factor);
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (err == kErpcStatus_Success)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kBBCarService_service_id, kBBCarService_RemoteControlAction_id, sequence);

        err = codec->getStatus();
    }

    return err;
}

// Server shim for RemoteTuneSpeed of BBCarService interface.
erpc_status_t BBCarService_service::RemoteTuneSpeed_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;

    double max;
    double min;

    // startReadMessage() was already called before this shim was invoked.

    codec->read(&max);

    codec->read(&min);

    err = codec->getStatus();
    if (err == kErpcStatus_Success)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        RemoteTuneSpeed(max, min);
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (err == kErpcStatus_Success)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kBBCarService_service_id, kBBCarService_RemoteTuneSpeed_id, sequence);

        err = codec->getStatus();
    }

    return err;
}

// Server shim for RemoteShowSpeed of BBCarService interface.
erpc_status_t BBCarService_service::RemoteShowSpeed_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;


    // startReadMessage() was already called before this shim was invoked.

    err = codec->getStatus();
    if (err == kErpcStatus_Success)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        RemoteShowSpeed();
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (err == kErpcStatus_Success)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kBBCarService_service_id, kBBCarService_RemoteShowSpeed_id, sequence);

        err = codec->getStatus();
    }

    return err;
}

// Server shim for RemoteShowPattern of BBCarService interface.
erpc_status_t BBCarService_service::RemoteShowPattern_shim(Codec * codec, MessageBufferFactory *messageFactory, uint32_t sequence)
{
    erpc_status_t err = kErpcStatus_Success;


    // startReadMessage() was already called before this shim was invoked.

    err = codec->getStatus();
    if (err == kErpcStatus_Success)
    {
        // Invoke the actual served function.
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = true;
#endif
        RemoteShowPattern();
#if ERPC_NESTED_CALLS_DETECTION
        nestingDetection = false;
#endif

        // preparing MessageBuffer for serializing data
        err = messageFactory->prepareServerBufferForSend(codec->getBuffer());
    }

    if (err == kErpcStatus_Success)
    {
        // preparing codec for serializing data
        codec->reset();

        // Build response message.
        codec->startWriteMessage(kReplyMessage, kBBCarService_service_id, kBBCarService_RemoteShowPattern_id, sequence);

        err = codec->getStatus();
    }

    return err;
}

erpc_service_t create_BBCarService_service(void)
{
    erpc_service_t service;

#if ERPC_ALLOCATION_POLICY == ERPC_ALLOCATION_POLICY_DYNAMIC
    service = new (nothrow) BBCarService_service();
#else
    if (s_BBCarService_service.isUsed())
    {
        service = NULL;
    }
    else
    {
        s_BBCarService_service.construct();
        service = s_BBCarService_service.get();
    }
#endif

    return service;
}

void destroy_BBCarService_service(erpc_service_t service)
{
#if ERPC_ALLOCATION_POLICY == ERPC_ALLOCATION_POLICY_DYNAMIC
    erpc_assert(service != NULL);
    delete (BBCarService_service *)service;
#else
    (void)service;
    erpc_assert(service == s_BBCarService_service.get());
    s_BBCarService_service.destroy();
#endif
}

